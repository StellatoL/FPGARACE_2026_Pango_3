"""Run a fixed-size PDS synthesis netlist with the installed Pango models."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
PDS = Path('C:/pango/PDS_2022.2-SP6.4')
TB = ROOT/'sim/tb'


def packed(values, bits):
    return sum(int(value) << (index*bits) for index, value in enumerate(values))


def model_dependencies(netlist, directory):
    """Resolve the GTP/INT models recursively, including LUT helper UDPs."""
    pending = set(re.findall(r'\b(?:GTP|INT)_[A-Za-z0-9_]+\b', netlist.read_text()))
    pending.add('GTP_GRS')
    models = {}
    while pending:
        name = pending.pop()
        if name in models:
            continue
        path = directory/(name+'.v')
        if not path.is_file():
            raise RuntimeError('Missing installed primitive model: '+str(path))
        text = path.read_text(encoding='utf-8', errors='replace')
        # Comments can mention unrelated primitive names and are not dependencies.
        text = re.sub(r'/\*.*?\*/|//[^\n]*', '', text, flags=re.S)
        models[name] = path
        pending.update(set(re.findall(r'\b(?:GTP|INT)_[A-Za-z0-9_]+\b', text))-models.keys())
    return [models[name] for name in sorted(models)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, required=True, help='PDS output directory containing summary.json and prj/')
    parser.add_argument('--case', default='small', choices=['small','synthetic720','synthetic720-wide','public'])
    parser.add_argument('--pds-home', type=Path, default=PDS)
    parser.add_argument('--timeout', type=int, default=1200)
    parser.add_argument('--compile-only', action='store_true', help='Compile primitive models and netlist only; no frame validation')
    args = parser.parse_args()
    build = args.build.resolve()
    fixture = ROOT/'sim/data'/args.case
    data = json.loads((fixture/'frame.json').read_text(encoding='utf-8'))
    synthesis = json.loads((build/'summary.json').read_text(encoding='utf-8'))
    if not synthesis.get('success'):
        raise RuntimeError('The selected PDS build did not report successful synthesis')
    build_hashes=json.loads((build/'input-sha256.json').read_text(encoding='utf-8'))
    for source in sorted((ROOT/'src/rtl').glob('*.v')):
        key='rtl/'+source.name
        current=hashlib.sha256(source.read_bytes()).hexdigest()
        snapshot=hashlib.sha256((build/key).read_bytes()).hexdigest()
        if current != snapshot or snapshot != build_hashes.get(key):
            raise RuntimeError('Synthesis snapshot does not match current RTL: '+source.name)
    netlist = build/'prj/synthesize/avm_core_syn.vm'
    if not netlist.is_file():
        raise RuntimeError('Missing synthesis netlist: '+str(netlist))
    if not args.compile_only:
        dimensions = (synthesis.get('width'), synthesis.get('height'))
        expected_dimensions = (data['width'], data['height'])
        if dimensions != expected_dimensions:
            raise RuntimeError(f'Fixed netlist dimensions {dimensions} do not match fixture {expected_dimensions}; '
                               'synthesize with matching --width/--height first')
    models = model_dependencies(netlist, args.pds_home/'arch/vendor/pango/verilog/simulation')
    out = ROOT/'build/avm/gate-simulation'/f'{build.name}-{time.strftime("%Y%m%d-%H%M%S")}'
    out.mkdir(parents=True)
    print('Gate simulation output:', out, flush=True)
    print('Installed primitive models:', ', '.join(path.stem for path in models), flush=True)
    commands = {name: shutil.which(name) for name in ('vmap', 'vlib', 'vlog', 'vsim')}
    if not all(commands.values()):
        raise RuntimeError(f'Missing ModelSim tools: {commands}')
    env = dict(os.environ)
    env.pop('MODELSIM', None)
    source_paths = models+sorted((build/'rtl').glob('*.v'))+[netlist, build/'summary.json',build/'input-sha256.json', fixture/'frame.json',
                           fixture/'memory.hex', fixture/'expected.hex',
                           TB/'tb_avm_gate.sv', ROOT/'sim/models/avm_memory_bfm.sv', Path(__file__).resolve()]
    manifest = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in source_paths}
    (out/'input-sha256.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    invocations = []
    summary = dict(success=False, build=str(build), fixture=args.case,
                   width=data['width'], height=data['height'], compile_only=args.compile_only,
                   synthesized_width=synthesis.get('width'), synthesized_height=synthesis.get('height'),
                   stage='netlist compilation probe' if args.compile_only else 'post-synthesis functional simulation',
                   sdf_annotated=False,
                   current_rtl_snapshot_verified=True,
                   post_route_timing=False, primitive_models=[str(path) for path in models],
                   log_directory=str(out))

    def execute(name, argv, marker=None):
        invocations.append(dict(stage=name, command=argv))
        (out/'commands.json').write_text(json.dumps(invocations, indent=2), encoding='utf-8')
        started = time.monotonic()
        with (out/(name+'.log')).open('wb') as log:
            result = subprocess.run(argv, cwd=out, env=env, stdout=log,
                                    stderr=subprocess.STDOUT, timeout=args.timeout)
        text = (out/(name+'.log')).read_text(encoding='utf-8', errors='replace')
        # Some vendor models use $display(ERROR) followed by $finish, returning 0.
        errors = re.findall(r'(?im)^.*(?:\*\*\s+(?:Error|Fatal):|\bERROR\s*:|'
                            r'Error loading design|failed to find design unit|'
                            r'Instantiation of .* failed).*$' , text)
        if result.returncode or errors or (marker and marker not in text):
            raise RuntimeError(f'{name} failed (exit={result.returncode}; marker={marker!r}): '
                               f'{out/(name+".log")}\n'+text[-5000:])
        print(name, round(time.monotonic()-started, 2), 's', flush=True)
        return text

    started = time.monotonic()
    try:
        execute('vmap', [commands['vmap'], '-c'])
        execute('vlib', [commands['vlib'], 'work'])
        # Delay controls are zeroed only for the design/models, never the TB clock.
        execute('compile-primitives', [commands['vlog'], '+delay_mode_zero', *map(str, models)])
        execute('compile-netlist', [commands['vlog'], '+delay_mode_zero', str(netlist)])
        if args.compile_only:
            summary.update(success=True, result='COMPILE_ONLY', pass_bit_exact=False)
        else:
            edges = {(entry['cameras'][0], entry['cameras'][1]): entry for entry in data['statistics']}
            counts, sums = [], []
            for edge in [(0, 2), (0, 3), (1, 2), (1, 3)]:
                counts.append(edges[edge]['count'])
                sums.extend(value for row in edges[edge]['sums'] for value in row)
            values = dict(WIDTH=data['width'], HEIGHT=data['height'], SRC_WIDTH=data['src_width'],
                          SRC_HEIGHT=data['src_height'], STRIDE=data['src_stride'],
                          DESC_BASE=data['desc_base'], ROUTE_BASE=data['route_base'],
                          MEMORY_WORDS=data['memory_words'],
                          BASES=f"128'h{packed(data['camera_bases'],32):032x}",
                          GAINS=f"192'h{packed([v for camera in data['gains'] for v in camera],16):048x}",
                          STAT_COUNTS=f"128'h{packed(counts,32):032x}",
                          STAT_SUMS=f"768'h{packed(sums,32):0192x}")
            (out/'gate_fixture.vh').write_text(''.join(f'`define AVM_{key} {value}\n' for key,value in values.items()), encoding='ascii')
            execute('compile-tb', [commands['vlog'], '-sv', '+incdir+'+str(out),
                                   str(ROOT/'sim/models/avm_memory_bfm.sv'), str(TB/'tb_avm_gate.sv')])
            text = execute('simulation', [commands['vsim'], '-c', '-t', '1ps', '+notimingchecks',
                               '-onfinish', 'exit', 'work.tb_avm_gate',
                               '+MEMHEX='+str(fixture/'memory.hex'),
                               '+EXPECTED='+str(fixture/'expected.hex'),
                               '+OUTPUT='+str(out/'gate_output.hex'), '+STATS='+str(out/'statistics.json'),
                               '-do', 'onerror {quit -code 1}; onbreak {quit -code 1}; run -all; quit -code 0'],
                           'PASS: tb_avm_gate')
            stats = json.loads((out/'statistics.json').read_text(encoding='utf-8'))
            actual = [int(line, 16) for line in (out/'gate_output.hex').read_text().splitlines()]
            expected = [int(line, 16) for line in (fixture/'expected.hex').read_text().splitlines()]
            if actual != expected or len(actual) != data['width']*data['height']:
                raise RuntimeError('Gate output artifact failed independent full-frame comparison')
            if stats['read_beats'] <= 0 or stats['source_requests'] <= 0:
                raise RuntimeError('No external DDR source traffic was measured')
            summary.update(success=True, result='PASS', pass_bit_exact=True, statistics=stats)
            print(next(line for line in text.splitlines() if 'PASS: tb_avm_gate' in line), flush=True)
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as error:
        summary['error'] = str(error)
        raise
    finally:
        summary['elapsed_seconds'] = round(time.monotonic()-started, 2)
        (out/'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    print(json.dumps(summary, indent=2), flush=True)
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as error:
        print(error, file=sys.stderr)
        raise SystemExit(1)
