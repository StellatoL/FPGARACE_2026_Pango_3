"""Package audited AVM sources and evidence without generated simulator libraries."""
from datetime import datetime
import hashlib
import json
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[2]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    report = json.loads((ROOT/'doc/avm/pds-results.json').read_text(encoding='utf-8'))
    simulation = json.loads((ROOT/'doc/avm/validation-results.json').read_text(encoding='utf-8'))
    if not report['current_rtl_verified'] or not report['gate']['success'] or not simulation['current_run_inputs_verified']:
        raise RuntimeError('Passing audited PDS and simulation reports required')
    for rel, expected in report['selected']['input_sha256'].items():
        if rel.startswith('rtl/') and digest(ROOT/'src'/rel) != expected:
            raise RuntimeError('Changed RTL after report generation: '+rel)
    sources = json.loads((ROOT/'doc/avm/validation/delivery-source-sha256.json').read_text(encoding='utf-8'))
    for rel, expected in sources.items():
        if digest(ROOT/rel) != expected:
            raise RuntimeError('Changed source after simulation report collection: '+rel)
    paths = [ROOT/'README.md', ROOT/'.gitignore']
    for directory in ['src', 'sim', 'constraints', 'project', 'tools', 'doc']:
        paths.extend(p for p in (ROOT/directory).rglob('*')
                     if p.is_file() and '__pycache__' not in p.parts and p.suffix != '.pyc'
                     and not p.is_relative_to(ROOT/'sim/data'))
    paths = sorted(set(paths))
    manifest = {p.relative_to(ROOT).as_posix(): digest(p) for p in paths}
    name = 'avm-fpga-delivery-'+datetime.now().strftime('%Y%m%d-%H%M%S')+'.zip'
    archive = ROOT/'archive/releases'/name
    archive.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(archive, 'x', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        for path in paths:
            z.write(path, path.relative_to(ROOT).as_posix())
        z.writestr('DELIVERY-SHA256.json', json.dumps(manifest, indent=2)+'\n')
    with zipfile.ZipFile(archive) as z:
        bad = z.testzip()
        if bad:
            raise RuntimeError('ZIP CRC failure: '+bad)
        for path, expected in manifest.items():
            if hashlib.sha256(z.read(path)).hexdigest() != expected:
                raise RuntimeError('ZIP digest mismatch: '+path)
    checksum = digest(archive)
    archive.with_suffix('.zip.sha256').write_text(checksum+'  '+name+'\n', encoding='ascii')
    print(f'PASS: {len(manifest)} files, CRC and SHA256 checked')
    print(archive)
    print(f'{archive.stat().st_size/1024**2:.2f} MiB; SHA256 {checksum}')


if __name__ == '__main__':
    main()
