"""Inspect the exact staged Git blobs without printing potential secret values."""
from __future__ import annotations
import argparse
import hashlib
import json
import posixpath
from pathlib import Path
import re
import struct
import subprocess

PATTERNS = {
    'private-key': rb'-----BEGIN (?:[A-Z0-9]+ )*PRIVATE KEY-----',
    'github-token': rb'(?:gh[pousr]_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{30,})',
    'cloud-access-key': rb'\b(?:AKIA|ASIA)[A-Z0-9]{16}\b',
    'service-token': rb'\b(?:xox[baprs]-[A-Za-z0-9-]{20,}|sk_live_[A-Za-z0-9]{20,})',
    'authorization-value': rb'(?i)Authorization\s*:\s*(?:Bearer|Basic)\s+[A-Za-z0-9+/=_-]{12,}',
    'credential-in-url': rb'(?i)https?://[^\s/<>"\x27]+:[^\s/<>"\x27]+@',
    'personal-directory': rb'(?i)(?:[A-Z]:[/\\]+Users[/\\]+[^/\\\s"\x27]+|/(?:home|Users)/[A-Za-z0-9_.-]+)',
    'private-network': rb'\b(?:10(?:\.\d{1,3}){3}|192\.168(?:\.\d{1,3}){2}|172\.(?:1[6-9]|2\d|3[01])(?:\.\d{1,3}){2})\b',
    'assigned-secret': rb'''(?im)\b(?:password|passwd|client_secret|api_key|access_token|refresh_token)\s*[=:]\s*["']([^"'\r\n]{12,})["']''',
    'license-material': rb'(?im)^\s*(?:SERVER\s+\S+\s+[0-9a-f]{12}|(?:FEATURE|INCREMENT)\s+\S+\s+\S+\s+\d)',
}
ALLOWED_EXTENSIONS = {'.md','.py','.m','.v','.sv','.f','.fdc','.tcl','.do','.json','.txt','.png'}


def findings(name, data):
    results = []
    for rule, expression in PATTERNS.items():
        for match in re.finditer(expression, data):
            results.append(dict(path=name, line=data[:match.start()].count(b'\n')+1, rule=rule))
    return results


def png_chunks(data):
    if not data.startswith(b'\x89PNG\r\n\x1a\n'):
        raise ValueError('Invalid PNG')
    pos, kinds = 8, []
    while pos < len(data):
        size = struct.unpack('>I', data[pos:pos+4])[0]
        kind = data[pos+4:pos+8].decode('ascii')
        kinds.append(kind)
        pos += size+12
        if kind == 'IEND':
            break
    return kinds


def self_test():
    samples = [b'gh' + b'p_' + b'Z'*36,
               b'C:' + b'/Users/' + b'private-user/project',
               b'-----BEGIN ' + b'PRIVATE KEY-----',
               b'https://' + b'user:secret@example.invalid/repo']
    for sample in samples:
        assert findings('controlled-sample.txt', sample), 'Scanner failed its controlled test'
    assert not findings('safe.txt', b'public documentation and 100 MHz RTL')
    print('PASS: controlled secret/path detections; sample values not printed')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--report', type=Path, help='Local report path; never include it in the commit being checked')
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    root = Path(subprocess.check_output(['git','rev-parse','--show-toplevel'], text=True).strip())
    def git(*args):
        return subprocess.check_output(['git',*args], cwd=root)
    entries = git('ls-files','--stage','-z').split(b'\0')
    blobs, errors = {}, []
    for entry in entries:
        if not entry:
            continue
        info, raw_name = entry.split(b'\t',1)
        mode, oid, stage = info.decode().split()
        name = raw_name.decode('utf-8')
        if stage != '0' or mode not in {'100644','100755'}:
            errors.append(dict(path=name, rule='unexpected-mode-or-conflict'))
            continue
        data = git('cat-file','blob',oid)
        blobs[name] = data
        if len(data) > 10*1024*1024:
            errors.append(dict(path=name, rule='file-over-10MiB'))
        if name.split('/')[0] in {'build','archive','.work','.venv'}:
            errors.append(dict(path=name, rule='local-generated-tree'))
        if name.startswith(('doc/history/','doc/reference/','tools/maintenance/')):
            errors.append(dict(path=name, rule='local-only-context'))
        if name.startswith('sim/data/') and name != 'sim/data/README.md':
            errors.append(dict(path=name, rule='generated-fixture'))
        path = Path(name)
        if name not in {'.gitignore','.gitattributes','LICENSE','NOTICE'} and path.suffix not in ALLOWED_EXTENSIONS:
            errors.append(dict(path=name, rule='unexpected-file-type'))
        if path.name.lower().startswith('.env') or re.search(r'(?i)(credential|id_rsa|id_ed25519)',path.name):
            errors.append(dict(path=name, rule='sensitive-filename'))
        if path.suffix == '.png':
            chunks = png_chunks(data)
            if any(kind not in {'IHDR','IDAT','IEND'} for kind in chunks):
                errors.append(dict(path=name, rule='unreviewed-png-metadata'))
        else:
            errors.extend(findings(name, data))
    if not blobs:
        raise RuntimeError('No staged files')
    for name, data in blobs.items():
        if name.endswith('.md'):
            for link in re.findall(r'\]\(([^)]+)\)',data.decode('utf-8')):
                link = link.split('#')[0]
                if not link or '://' in link or link.startswith('mailto:'):
                    continue
                target = posixpath.normpath(posixpath.join(posixpath.dirname(name),link))
                if target not in blobs and not any(k.startswith(target.rstrip('/')+'/') for k in blobs):
                    errors.append(dict(path=name,rule='link-not-in-commit',target=target))
    manifest_name = 'doc/validation/source-sha256.json'
    if manifest_name not in blobs:
        errors.append(dict(path=manifest_name,rule='missing-source-manifest'))
    else:
        for name, expected in json.loads(blobs[manifest_name]).items():
            if name not in blobs or hashlib.sha256(blobs[name]).hexdigest() != expected:
                errors.append(dict(path=name,rule='verified-source-blob-mismatch'))
    total = sum(map(len,blobs.values()))
    result = dict(status='PASS' if not errors else 'FAIL',files=len(blobs),bytes=total,
                  rules=list(PATTERNS),findings=errors,
                  staged_sha256={k:hashlib.sha256(v).hexdigest() for k,v in sorted(blobs.items())})
    if args.report:
        args.report.parent.mkdir(parents=True,exist_ok=True)
        args.report.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(f'{result["status"]}: {len(blobs)} staged files, {total/1024**2:.2f} MiB, {len(errors)} findings')
    for error in errors:
        print(json.dumps(error,ensure_ascii=True))
    if errors:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
