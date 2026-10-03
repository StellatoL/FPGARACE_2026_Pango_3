"""Build real portable RTL, run scoreboards, retain exact logs and artifacts."""
from __future__ import annotations
import argparse, hashlib, json, os, shutil, subprocess, sys, time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
RTL=ROOT/'src/rtl'
TB=ROOT/'sim/tb'

def packed(values,bits):
    return sum(int(v)<<(i*bits) for i,v in enumerate(values))

def run(args):
    if args.seed is not None and not 1 <= args.seed <= 0xffffffff:
        raise ValueError('--seed must be a nonzero unsigned 32-bit integer')
    output_seed = 0x12345678 if args.seed is None else args.seed
    memory_seed = 0x51ac35a7 if args.seed is None else (args.seed ^ 0x51ac35a7) or 1
    fixture=ROOT/'sim/data'/args.case
    data=json.loads((fixture/'frame.json').read_text(encoding='utf-8'))
    if not (1<=data['width']<=4095 and 1<=data['height']<=4095 and
            2<=data['src_width']<=4095 and 2<=data['src_height']<=4095):
        raise ValueError('Dimensions exceed the 12-bit coordinate interface')
    if data['src_stride']%16 or data['src_stride']<data['src_width']*2 or data['src_stride']>65535:
        raise ValueError('Source stride must fit16 bits, be aligned16, and cover a row')
    if any(x%16 for x in data['camera_bases']+[data['desc_base'],data['route_base']]):
        raise ValueError('All memory base addresses must be16-byte aligned')
    suffix='-feedback' if args.feedback else ''
    if args.no_latest:
        suffix += f'-p{args.prefetch}-seed{output_seed:08x}'
    out=ROOT/'build/avm/simulation'/f'{args.case}-{args.profile}{suffix}-{time.strftime("%Y%m%d-%H%M%S")}'
    out.mkdir(parents=True)
    edges={(s['cameras'][0],s['cameras'][1]):s for s in data['statistics']}
    counts=[];sums=[]
    for edge in [(0,2),(0,3),(1,2),(1,3)]:
        st=edges[edge];counts.append(st['count']);sums.extend(v for row in st['sums'] for v in row)
    feedback=json.loads((fixture/'frame_feedback.json').read_text(encoding='utf-8')) if args.feedback else data
    values={'WIDTH':data['width'],'HEIGHT':data['height'],'SRC_WIDTH':data['src_width'],
            'SRC_HEIGHT':data['src_height'],'STRIDE':data['src_stride'],'DESC_BASE':data['desc_base'],
            'ROUTE_BASE':data['route_base'],'MEMORY_WORDS':data['memory_words'],
            'BASES':f"128'h{packed(data['camera_bases'],32):032x}",
            'GAINS':f"192'h{packed([v for c in data['gains'] for v in c],16):048x}",
            'FEEDBACK_GAINS':f"192'h{packed([v for c in feedback['gains'] for v in c],16):048x}",
            'FEEDBACK':int(args.feedback),'PREFETCH':args.prefetch,
            'STAT_COUNTS':f"128'h{packed(counts,32):032x}",
            'STAT_SUMS':f"768'h{packed(sums,32):0192x}",
            'CAPTURE_MBPS':221.184 if args.capture is None else args.capture,
            'OUTPUT_SEED':f"32'h{output_seed:08x}",'MEMORY_SEED':f"32'h{memory_seed:08x}",
            'STRESS':int(args.profile=='stress')}
    (out/'fixture.vh').write_text(''.join(f'`define AVM_{k} {v}\n' for k,v in values.items()),encoding='ascii')
    tools={k:shutil.which(k) for k in ('vlib','vmap','vlog','vsim')}
    if not all(tools.values()):raise RuntimeError(f'Missing ModelSim tools: {tools}')
    env=dict(os.environ);env.pop('MODELSIM',None)
    def command(name,argv,timeout=120):
        log=out/f'{name}.log';t=time.monotonic()
        with log.open('wb') as stream:
            p=subprocess.run(argv,cwd=out,env=env,stdout=stream,stderr=subprocess.STDOUT,timeout=timeout)
        text=log.read_text(encoding='utf-8',errors='replace')
        if p.returncode or '** Error:' in text or '** Fatal:' in text:
            raise RuntimeError(f'{name} failed ({log})\n{text[-4500:]}')
        print(name,round(time.monotonic()-t,2),'s',flush=True)
        return text
    command('vmap',[tools['vmap'],'-c']);command('vlib',[tools['vlib'],'work'])
    sources=sorted(RTL.glob('*.v'))
    compile_rtl=sources;compile_tb=[ROOT/'sim/models/avm_memory_bfm.sv',TB/'tb_avm_frame.sv']
    if args.coverage:
        from simulation_coverage import prepare
        compile_rtl,compile_tb=prepare(out,sources,compile_tb)
    command('compile-rtl',[tools['vlog'],'-vlog01compat',*(['+cover=bcesft'] if args.coverage else []),*map(str,compile_rtl)])
    command('compile-tb',[tools['vlog'],'-sv','+incdir+'+str(out),*map(str,compile_tb)])
    direct_inputs=sources+[TB/'tb_avm_frame.sv',ROOT/'sim/models/avm_memory_bfm.sv',Path(__file__).resolve(),
                          fixture/'frame.json',fixture/'memory.hex',fixture/'expected.hex']
    if args.feedback:direct_inputs += [fixture/'frame_feedback.json',fixture/'expected_feedback.hex']
    if args.coverage:direct_inputs += [ROOT/'tools/avm/simulation_coverage.py']
    manifest={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in direct_inputs}
    (out/'input-sha256.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    plus=['+MEMHEX='+str(fixture/'memory.hex'),'+EXPECTED='+str(fixture/'expected.hex'),
          '+OUTPUT='+str(out/'rtl_output.hex'),'+STATS='+str(out/'statistics.json')]
    if args.wave:plus+=['+VCD='+str(out/'core.vcd')]
    if args.feedback:plus+=['+FEEDBACK_EXPECTED='+str(fixture/'expected_feedback.hex')]
    script='onerror {quit -code 1}; onbreak {quit -code 1}; '
    if args.coverage:script+='coverage save -onexit coverage.ucdb; '
    script+='run -all; quit -code 0'
    vopt=['+acc'] if args.wave else []
    text=command('simulation',[tools['vsim'],'-c','-onfinish','exit',*(['-coverage'] if args.coverage else []),*(['-voptargs='+' '.join(vopt)] if vopt else []),'work.tb_avm_frame',*plus,
        '-do',script],args.timeout)
    if 'PASS: tb_avm_frame' not in text:raise RuntimeError('Missing explicit PASS marker')
    if args.coverage and not (out/'coverage.ucdb').is_file():raise RuntimeError('Missing coverage UCDB')
    if args.coverage:
        report=command('coverage-report',[shutil.which('vcover'),'report','-bydu','-code','bcesft',str(out/'coverage.ucdb')])
        if 'No matching coverage data' in report or 'covdut_core' not in report:
            raise RuntimeError('UCDB contains no AVM RTL coverage')
    result=json.loads((out/'statistics.json').read_text())
    result.update(case=args.case,profile=args.profile,source='actual ModelSim RTL+DDR BFM',
                  pass_bit_exact=True,meets_30fps_model=result['processing_ms']<=1000/30,
                  capture_mbps=values['CAPTURE_MBPS'],prefetch_tiles=args.prefetch,log_directory=str(out),
                  output_seed=output_seed,memory_seed=memory_seed,coverage=args.coverage,
                  fixture_case=data['case'],seam_strategy=data.get('seam_strategy','coverage_distance_wide'),
                  feather_width_pixels=data.get('feather_width_pixels',0),
                  measurement_interval='last frame start-to-done; bus/cache counters are frame deltas',
                  fixture_sha256=manifest[str((fixture/'frame.json').relative_to(ROOT))],
                  input_manifest=str(out/'input-sha256.json'))
    (out/'summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    if not args.no_latest:
        (fixture/f'latest-{args.profile}{suffix}.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    # A reviewable image is decoded from actual simulator output, not copied
    # from the software golden image. Each frame has its own artifact.
    import numpy as np
    import cv2
    pixels=np.fromiter((int(line,16) for line in (out/'rtl_output.hex').read_text().splitlines()),dtype=np.uint16)
    for index,frame in enumerate(pixels.reshape(-1,data['height'],data['width'])):
        r=frame>>11;g=(frame>>5)&63;b=frame&31
        image=np.stack(((b<<3)|(b>>2),(g<<2)|(g>>4),(r<<3)|(r>>2)),-1).astype(np.uint8)
        cv2.imwrite(str(out/f'rtl_frame_{index}.png'),image)
    print(json.dumps(result,indent=2),flush=True)
    return out

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--case',default='small',choices=['small','synthetic720','synthetic720-wide','public'])
    p.add_argument('--profile',default='nominal',choices=['nominal','stress']);p.add_argument('--wave',action='store_true')
    p.add_argument('--timeout',type=int,default=1800);p.add_argument('--capture',type=float)
    p.add_argument('--feedback',action='store_true');p.add_argument('--prefetch',type=int,default=2,choices=[1,2,4])
    p.add_argument('--seed',type=lambda x:int(x,0),help='Nonzero 32-bit seed; omitted preserves baseline seeds')
    p.add_argument('--coverage',action='store_true',help='Collect RTL code coverage in coverage.ucdb')
    p.add_argument('--no-latest',action='store_true',help='Keep standard regression pointers unchanged')
    try:run(p.parse_args())
    except (RuntimeError,subprocess.TimeoutExpired) as e:print(e,file=sys.stderr);sys.exit(1)
