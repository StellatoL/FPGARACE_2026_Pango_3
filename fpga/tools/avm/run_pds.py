"""Synthesize the AVM core in an isolated workspace build using native PDS ADS."""
from __future__ import annotations
import argparse
import ctypes
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


def parse_report(text):
    resources = {}
    for name, pattern in [('lut',r'Total LUTs: (\d+) of (\d+)'),
                          ('register',r'Total Registers: (\d+) of (\d+)'),
                          ('drm36k',r'Total DRM = ([\d.]+) of (\d+)'),
                          ('apm',r'Total APMs = ([\d.]+) of (\d+)')]:
        found = re.search(pattern,text)
        if found:
            used, capacity = map(float,found.groups())
            resources[name] = dict(used=used,capacity=capacity,percent=round(100*used/capacity,3))
    result = dict(resources=resources,reported_capacity_fit=bool(resources) and all(x['used']<=x['capacity'] for x in resources.values()))
    latch = re.search(r'Total Latches: (\d+)',text)
    if latch:result['latches']=int(latch[1])
    timing = re.search(r'core_clk\s+([\d.]+) MHz\s+([\d.]+) MHz\s+([\d.]+)\s+([\d.]+)\s+([-\d.]+)',text)
    if timing:
        requested, estimated, period, estimated_period, slack = map(float,timing.groups())
        result['timing']=dict(requested_mhz=requested,estimated_mhz=estimated,
                              requested_period_ns=period,estimated_period_ns=estimated_period,
                              setup_slack_ns=slack,synthesis_target_met=slack>=0,
                              external_io_budgets_defined=False,post_route=False)
    result['worst_paths']=re.findall(r'Startpoint\s*:\s*(.+)\nEndpoint\s*:\s*(.+)',text)[:5]
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--label', default='baseline')
    ap.add_argument('--rtl-source', type=Path, default=ROOT/'src/rtl')
    ap.add_argument('--pds-home', type=Path, default=PDS)
    ap.add_argument('--timeout', type=int, default=3600)
    ap.add_argument('--width',type=int,default=1280,help='Small values are for gate-level functional tests only')
    ap.add_argument('--height',type=int,default=720)
    ap.add_argument('--retiming', action='store_true')
    ap.add_argument('--no-sharing', action='store_true')
    ap.add_argument('--subst', action=argparse.BooleanOptionalAction, default=True,
                    help='Temporary workspace alias; required by this PDS build on Desktop paths')
    args = ap.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_-]+', args.label):
        ap.error('--label must contain only letters, digits, hyphens and underscores')
    if not 1<=args.width<=4095 or not 1<=args.height<=4095:
        ap.error('Dimensions must fit the core 12-bit interface')
    shell = args.pds_home/'bin/pds_shell.exe'
    if not shell.is_file():
        raise RuntimeError('Missing PDS executable: '+str(shell))
    out = ROOT/'build/avm/pds'/(args.label+'-'+time.strftime('%Y%m%d-%H%M%S'))
    prj = out/'prj'
    prj.mkdir(parents=True)
    shutil.copytree(args.rtl_source, out/'rtl', ignore=shutil.ignore_patterns('__pycache__'))
    shutil.copy2(ROOT/'constraints/avm_core.fdc', prj/'avm_core.fdc')
    script = (ROOT/'project/pds/build.tcl').read_text(encoding='utf-8')
    script = script.replace('WIDTH 1280 HEIGHT 720',f'WIDTH {args.width} HEIGHT {args.height}')
    options = (['-retiming'] if args.retiming else [])+(['-resource_sharing', 'false'] if args.no_sharing else [])
    script = script.replace('set avm_synth_options {} ;# @SYNTH_OPTIONS@',
                            'set avm_synth_options {'+' '.join(options)+'}')
    (prj/'build.tcl').write_text(script, encoding='utf-8')
    inputs = sorted((out/'rtl').glob('*.v'))+[prj/'avm_core.fdc', prj/'build.tcl']
    hashes = {p.relative_to(out).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    (out/'input-sha256.json').write_text(json.dumps(hashes, indent=2), encoding='utf-8')
    drive = None
    try:
        work = str(prj)
        if args.subst:
            for letter in reversed('PQRSTUVWXYZ'):
                if not ctypes.windll.kernel32.GetLogicalDrives() & (1 << (ord(letter)-ord('A'))):
                    drive = letter+':'
                    break
            if drive is None:
                raise RuntimeError('No free letter for temporary workspace alias')
            subprocess.run(['subst',drive,str(ROOT)],check=True)
            work = drive+'/'+prj.relative_to(ROOT).as_posix()
        command = [str(shell),'-file','build.tcl','-work_dir',work,'-project_name','avm_core']
        (out/'command.json').write_text(json.dumps(command,indent=2),encoding='utf-8')
        print('PDS output:',out,flush=True)
        started = time.time()
        timed_out = False
        with (out/'console.log').open('wb') as log:
            try:
                result = subprocess.run(command,cwd=work,stdout=log,stderr=subprocess.STDOUT,timeout=args.timeout)
                code = result.returncode
            except subprocess.TimeoutExpired:
                timed_out = True
                code = -1
        log = (out/'console.log').read_text(encoding='utf-8',errors='replace')
        errors = re.findall(r'(?m)^E:.*|^.*Program Error Out.*|^.*License checkout failed.*|^.*couldn.t read file.*',log)
        reports = list(prj.rglob('*.snr'))
        fresh = [p for p in reports if p.stat().st_mtime >= started-2]
        success = code == 0 and not timed_out and not errors and bool(fresh)
        summary = dict(label=args.label,success=success,exit_code=code,timeout=timed_out,
                       elapsed_seconds=round(time.time()-started,2),errors=errors,
                       reports=[str(p) for p in fresh],directory=str(out),
                       target='PG2L50H-6IFBG484',clock_mhz=100,io_insertion=False,
                       width=args.width,height=args.height,
                       options=options,rtl_source=str(args.rtl_source.resolve()),
                       stage='ADS synthesis',post_route_timing=False)
        if fresh:summary.update(parse_report(fresh[0].read_text(encoding='utf-8',errors='replace')))
        content=json.dumps(summary,indent=2)
        (out/'summary.json').write_text(content,encoding='utf-8')
        (ROOT/'build/avm/pds'/('latest-'+args.label+'.json')).write_text(content,encoding='utf-8')
        print(content,flush=True)
        if not success:print(log[-5000:],flush=True)
        return 0 if success else 1
    finally:
        if drive:
            subprocess.run(['subst',drive,'/D'],check=True)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (RuntimeError,subprocess.CalledProcessError) as error:
        print(error,file=sys.stderr)
        raise SystemExit(1)
