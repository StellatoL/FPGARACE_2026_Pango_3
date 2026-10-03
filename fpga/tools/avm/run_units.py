"""Run every AVM RTL unit scoreboard in one disposable ModelSim library."""
from pathlib import Path
import argparse,hashlib,json,os,shutil,subprocess,time

ROOT=Path(__file__).resolve().parents[2]
RTL=ROOT/'src/rtl';TB=ROOT/'sim/tb'
TESTS=['tb_arithmetic','tb_reorder','tb_patch_store','tb_config','tb_cache','tb_b_only']

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--coverage',action='store_true')
    args=parser.parse_args()
    out=ROOT/'build/avm/units'/time.strftime('%Y%m%d-%H%M%S');out.mkdir(parents=True)
    commands={x:shutil.which(x) for x in ['vmap','vlib','vlog','vsim']}
    if not all(commands.values()):raise RuntimeError('ModelSim executables missing from PATH')
    env=dict(os.environ);env.pop('MODELSIM',None)
    def execute(label,args,passing=False):
        r=subprocess.run(args,cwd=out,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=180)
        s=r.stdout.decode('utf-8',errors='replace');(out/f'{label}.log').write_text(s,encoding='utf-8')
        if r.returncode or '** Error:' in s or '** Fatal:' in s or (passing and 'PASS' not in s):
            raise RuntimeError(f'{label} failed: {out}\n{s[-4000:]}')
        if passing:print(next((line for line in s.splitlines() if 'PASS' in line),'PASS'),flush=True)
    execute('vmap',[commands['vmap'],'-c']);execute('vlib',[commands['vlib'],'work'])
    rtl=sorted(RTL.glob('*.v'))
    tbs=[TB/f'{n}.sv' for n in TESTS+['tb_cache_prefetch']]+[ROOT/'sim/models/avm_memory_bfm.sv']
    compile_rtl,compile_tb=rtl,tbs
    if args.coverage:
        from simulation_coverage import prepare
        compile_rtl,compile_tb=prepare(out,rtl,tbs)
    execute('compile-rtl',[commands['vlog'],'-vlog01compat',*(['+cover=bcesft'] if args.coverage else []),*map(str,compile_rtl)])
    execute('compile-tb',[commands['vlog'],'-sv',*map(str,compile_tb)])
    script='onerror {quit -code 1}; onbreak {quit -code 1}; run -all; quit -code 0'
    runs=[]
    def run_test(name,top,parameters=()):
        test_script=('coverage save -onexit '+name+'.ucdb; ' if args.coverage else '')+script
        execute(name,[commands['vsim'],'-c','-onfinish','exit',*(['-coverage'] if args.coverage else []),
                      'work.'+top,*parameters,'-do',test_script],True)
        if args.coverage and not (out/(name+'.ucdb')).is_file():
            raise RuntimeError('Missing coverage UCDB: '+name)
        if args.coverage:
            report=subprocess.run([shutil.which('vcover'),'report','-bydu','-code','bcesft',str(out/(name+'.ucdb'))],
                                  cwd=out,env=env,capture_output=True,text=True,timeout=120)
            (out/(name+'-coverage.log')).write_text(report.stdout+report.stderr,encoding='utf-8')
            if report.returncode or 'covdut_' not in report.stdout:
                raise RuntimeError('UCDB contains no AVM RTL coverage: '+name)
    for test in TESTS:
        run_test(test,test)
        runs.append(dict(test=test,status='PASS'))
    for size in [1,2,4]:
        name=f'tb_cache_prefetch_{size}'
        run_test(name,'tb_cache_prefetch',['-GPREFETCH_TILES='+str(size)])
        runs.append(dict(test=name,status='PASS'))
    extra=[ROOT/'tools/avm/simulation_coverage.py'] if args.coverage else []
    result=dict(tests=runs,coverage=args.coverage,log_directory=str(out),source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in rtl+tbs+[Path(__file__).resolve()]+extra})
    (out/'summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    (ROOT/'build/avm/latest-units.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('Unit logs:',out)

if __name__=='__main__':main()
