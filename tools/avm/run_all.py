"""One command for reproducible algorithm and actual RTL verification."""
from pathlib import Path
import argparse,hashlib,json,subprocess,sys,time
ROOT=Path(__file__).resolve().parents[2]

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--regenerate',action='store_true')
    ap.add_argument('--matlab',action='store_true');ap.add_argument('--full-stress',action='store_true')
    args=ap.parse_args()
    logs=ROOT/'build/avm/regression'/time.strftime('%Y%m%d-%H%M%S')
    logs.mkdir(parents=True)
    (ROOT/'doc/avm').mkdir(parents=True,exist_ok=True)
    sources=[ROOT/'tools/avm'/name for name in ['model.py','generate.py','test_model.py',
             'exposure.py','apply_gains.py','verify_mapping.m','run_all.py']]
    summary=dict(status='RUNNING',steps=[],log_directory=str(logs),
                 source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
    def save():
        content=json.dumps(summary,indent=2)
        (logs/'summary.json').write_text(content,encoding='utf-8')
        (ROOT/'build/avm/latest-regression.json').write_text(content,encoding='utf-8')
    save()
    def call(*argv):
        label=f'{len(summary["steps"])+1:02d}-'+('unittest' if argv[0]=='-m' else Path(argv[0]).stem)
        print('Running',label,flush=True)
        result=subprocess.run([sys.executable,'-X','utf8',*argv],cwd=ROOT,
                              stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        output=result.stdout.decode('utf-8',errors='replace')
        (logs/f'{label}.log').write_text(output,encoding='utf-8')
        print(output,end='',flush=True)
        summary['steps'].append(dict(command=list(argv),log=label+'.log',exit_code=result.returncode))
        if result.returncode:summary['status']='FAIL'
        save()
        result.check_returncode()
    if args.regenerate or any(not (ROOT/f'sim/data/{c}/frame.json').exists() for c in ['small','synthetic720','synthetic720-wide','public']):
        for case in ['small','synthetic720','synthetic720-wide','public']:
            call('tools/avm/generate.py','--case',case,*(['--matlab'] if args.matlab else []))
    call('-m','unittest','discover','-s','tools/avm','-p','test_*.py','-v')
    call('tools/avm/run_units.py')
    for case in ['small','synthetic720','public']:
        call('tools/avm/run_modelsim.py','--case',case)
    if (ROOT/'sim/data/synthetic720-wide/frame.json').exists():
        call('tools/avm/run_modelsim.py','--case','synthetic720-wide')
    call('tools/avm/run_modelsim.py','--case','small','--profile','stress','--wave')
    if args.full_stress:
        call('tools/avm/run_modelsim.py','--case','synthetic720','--profile','stress')
    # The feedback JSON is derived from the actual hardware statistics. Its
    # quantized values are then programmed into the next RTL frame.
    call('tools/avm/exposure.py','--statistics','sim/data/synthetic720/latest-nominal.json',
         '--output','doc/avm/exposure-feedback.json')
    feedback=json.loads((ROOT/'doc/avm/exposure-feedback.json').read_text())
    (ROOT/'doc/avm/gains.json').write_text(json.dumps({'gains_q4_12':feedback['gains_q4_12']},indent=2))
    for case in ['small','synthetic720']:
        call('tools/avm/apply_gains.py','--fixture',f'sim/data/{case}/frame.json','--gains-json','doc/avm/gains.json')
    call('tools/avm/run_modelsim.py','--case','small','--profile','stress','--feedback')
    call('tools/avm/run_modelsim.py','--case','synthetic720','--feedback')
    summary['status']='PASS';save()
    print('Regression logs:',logs,flush=True)

if __name__=='__main__':main()
