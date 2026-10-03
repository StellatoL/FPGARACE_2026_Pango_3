"""Audit the exact system Git index; never print detected secret values."""
import argparse
import hashlib
import importlib.util
import io
import json
from pathlib import Path, PurePosixPath
import posixpath
import re
import struct
import subprocess
from urllib.parse import unquote

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('fpga_scan', ROOT/'fpga/tools/release/check_publication.py')
legacy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(legacy)
EXTENSIONS = {'.md', '.py', '.sh', '.cpp', '.h', '.hpp', '.ui', '.pro', '.qss', '.ico',
              '.v', '.sv', '.f', '.fdc', '.tcl', '.m', '.do', '.json', '.txt', '.png'}
MANIFEST = 'tools/release/publication-files.json'
SOURCE_MANIFEST = 'doc/validation/source-sha256.json'
DRIVER = 'arm/drivers/prebuilt/pango_pci_driver_mul.ko'
DRIVER_SHA = '173904a078cd8ff166c350a24c950044a3dbab73800ee6460524c33f18db5277'


def forbidden(name):
    p = PurePosixPath(name)
    blocked_parts = {'build', 'archive', '__pycache__', '.git', '.venv', '.codex', '.claude', 'history'}
    return (bool(set(p.parts) & blocked_parts) or name.startswith('tools/maintenance/') or
            name.startswith('fpga/doc/reference/') or
            (name.startswith('fpga/sim/data/') and name != 'fpga/sim/data/README.md') or
            name.lower().endswith('/rknn_api.h') or
            (p.name.startswith('.env') or re.search(r'(?i)credential|id_rsa|id_ed25519', p.name)))


def image_metadata_errors(name, data):
    payloads = []
    if name.endswith('.png'):
        payloads = [data]
    elif name.endswith('.ico'):
        reserved, kind, count = struct.unpack_from('<HHH', data)
        if reserved or kind != 1 or not 1 <= count <= 256:
            raise ValueError('Invalid ICO directory')
        for index in range(count):
            length, offset = struct.unpack_from('<II', data, 6+index*16+8)
            if offset < 6+16*count or offset+length > len(data):
                raise ValueError('Invalid ICO image extent')
            payload = data[offset:offset+length]
            if payload.startswith(b'\x89PNG'):
                payloads.append(payload)
        from PIL import Image
        with Image.open(io.BytesIO(data)) as icon:
            icon.load()
    return any(kind not in {'IHDR', 'IDAT', 'IEND'} for payload in payloads for kind in legacy.png_chunks(payload))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--self-test', action='store_true')
    ap.add_argument('--report', type=Path)
    args = ap.parse_args()
    if args.self_test:
        legacy.self_test()
        for name in ['arm/third_party/rknn/include/rknn_api.h', 'arm/build/app', 'archive/backup.zip',
                     'fpga/sim/data/small/frame.json', 'arm/doc/history/plan.md']:
            assert forbidden(name), name
        assert not forbidden('arm/src/main.cpp')
        print('PASS: system exclusion rules')
        return
    def git(*argv):
        return subprocess.check_output(['git', *argv], cwd=ROOT)
    assert Path(git('rev-parse', '--show-toplevel').decode().strip()).resolve() == ROOT
    blobs, errors = {}, []
    for entry in git('ls-files', '--stage', '-z').split(b'\0'):
        if not entry:
            continue
        info, raw = entry.split(b'\t', 1)
        mode, oid, stage = info.decode().split()
        name = raw.decode('utf-8')
        if mode not in {'100644', '100755'} or stage != '0':
            errors.append({'path': name, 'rule': 'mode-or-conflict'}); continue
        data = git('cat-file', 'blob', oid)
        blobs[name] = data
        if forbidden(name):
            errors.append({'path': name, 'rule': 'excluded-local-or-vendor-asset'})
        if name not in {'.gitignore', '.gitattributes', 'fpga/.gitignore', 'LICENSE', 'NOTICE', DRIVER} and PurePosixPath(name).suffix not in EXTENSIONS:
            errors.append({'path': name, 'rule': 'unreviewed-file-type'})
        if name == DRIVER and (hashlib.sha256(data).hexdigest() != DRIVER_SHA or data[:6] != b'\x7fELF\x02\x01'):
            errors.append({'path': name, 'rule': 'driver-not-the-reviewed-debug-redacted-elf'})
        if len(data) > 5*1024*1024:
            errors.append({'path': name, 'rule': 'file-over-5MiB'})
        # Byte scan includes icons so printable metadata is covered too.
        errors.extend(legacy.findings(name, data))
        if name.endswith(('.png', '.ico')):
            try:
                if image_metadata_errors(name, data):
                    errors.append({'path': name, 'rule': 'image-metadata'})
            except (ValueError, OSError, struct.error):
                errors.append({'path': name, 'rule': 'invalid-image'})
    if MANIFEST not in blobs:
        errors.append({'path': MANIFEST, 'rule': 'missing-file-allowlist'})
    elif set(json.loads(blobs[MANIFEST])['files']) != set(blobs):
        errors.append({'path': MANIFEST, 'rule': 'file-allowlist-mismatch'})
    if SOURCE_MANIFEST not in blobs:
        errors.append({'path': SOURCE_MANIFEST, 'rule': 'missing-source-hashes'})
    else:
        for name, expected in json.loads(blobs[SOURCE_MANIFEST]).items():
            if name not in blobs or hashlib.sha256(blobs[name]).hexdigest() != expected:
                errors.append({'path': name, 'rule': 'source-hash-mismatch'})
    for name, data in blobs.items():
        if name.endswith('.md'):
            for link in re.findall(r'\]\(([^)]+)\)', data.decode('utf-8')):
                link = unquote(link.split('#', 1)[0].split(' "', 1)[0]).strip('<>')
                if not link or re.match(r'(?:https?://|mailto:)', link):
                    continue
                target = posixpath.normpath(posixpath.join(posixpath.dirname(name), link))
                if target not in blobs and not any(k.startswith(target.rstrip('/')+'/') for k in blobs):
                    errors.append({'path': name, 'rule': 'link-not-in-commit', 'target': target})
    result = {'status': 'FAIL' if errors else 'PASS', 'files': len(blobs),
              'bytes': sum(map(len, blobs.values())), 'findings': errors,
              'tree': git('write-tree').decode().strip(),
              'staged_sha256': {k: hashlib.sha256(v).hexdigest() for k,v in sorted(blobs.items())}}
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print(f"{result['status']}: {result['files']} staged files, {result['bytes']/1024**2:.2f} MiB, {len(errors)} findings")
    for error in errors:
        print(json.dumps(error, ensure_ascii=False))
    if errors:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
