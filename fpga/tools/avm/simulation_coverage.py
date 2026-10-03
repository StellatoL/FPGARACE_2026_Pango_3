"""Prepare reversible module-name aliases for ModelSim 10.7 coverage.

This installation suppresses coverage for avm_* design-unit names, including
a two-line counter probe. The identical probe named probe_* is instrumented.
Only the coverage copy uses covdut_* names; production RTL is never edited.
The token substitution is checked to be byte-for-byte reversible and recorded.
"""
from pathlib import Path
import hashlib
import json
import re


def prepare(out, rtl, testbenches):
    names = sorted({name for path in rtl
                    for name in re.findall(rb'\bmodule\s+(\w+)', path.read_bytes())})
    aliases = {name: b'covdut_'+name.removeprefix(b'avm_') for name in names}
    pattern = re.compile(rb'\b('+b'|'.join(names)+rb')\b')
    reverse = {value: key for key, value in aliases.items()}
    undo = re.compile(rb'\b('+b'|'.join(reverse)+rb')\b')
    directory = out/'coverage-src'
    directory.mkdir()
    manifest = {'module_aliases': {k.decode(): v.decode() for k, v in aliases.items()},
                'reason': 'ModelSim 10.7 avm_* coverage suppression reproduced with minimal probe',
                'byte_reversible': True, 'files': []}
    for source in rtl+testbenches:
        original = source.read_bytes()
        translated = pattern.sub(lambda match: aliases[match[0]], original)
        if undo.sub(lambda match: reverse[match[0]], translated) != original:
            raise RuntimeError('Coverage alias is not reversible: '+str(source))
        dest = directory/source.name.replace('avm_', 'covdut_')
        dest.write_bytes(translated)
        manifest['files'].append({'original': str(source), 'copy': str(dest),
                                 'original_sha256': hashlib.sha256(original).hexdigest(),
                                 'copy_sha256': hashlib.sha256(translated).hexdigest()})
    (out/'coverage-source-map.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    return ([directory/p.name.replace('avm_', 'covdut_') for p in rtl],
            [directory/p.name.replace('avm_', 'covdut_') for p in testbenches])
