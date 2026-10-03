"""Audit actual ADS builds and a passing full-frame netlist simulation."""
from __future__ import annotations
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import shutil

from run_pds import parse_report

ROOT = Path(__file__).resolve().parents[2]
DOC = ROOT/'doc/avm'
ARCHIVE = DOC/'pds-validation'


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def dump(path, data):
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')


def copy_file(src, dst):
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)


def audit_build(source, current=False, netlist=False):
    result = read(source/'summary.json')
    hashes = read(source/'input-sha256.json')
    for relative, expected in hashes.items():
        if digest(source/relative) != expected:
            raise RuntimeError('Changed synthesis snapshot: '+str(source/relative))
        if current and relative.startswith('rtl/'):
            if digest(ROOT/'src'/relative) != expected:
                raise RuntimeError('Live RTL differs from selected synthesis: '+relative)
    log = (source/'console.log').read_text(encoding='utf-8', errors='replace')
    snr = source/'prj/synthesize/avm_core.snr'
    if result['success']:
        if result['exit_code'] or not snr.is_file() or re.search(r'^E:', log, re.M):
            raise RuntimeError('Missing successful synthesis evidence: '+str(source))
        report_text = snr.read_text(encoding='utf-8', errors='replace')
        result.update(parse_report(report_text))
        result['module_resources'] = {}
        for line in report_text.splitlines():
            columns = [cell.strip() for cell in line.split('|')]
            if len(columns) >= 7 and (columns[1] == 'avm_core' or columns[1].startswith('+ ')):
                if all(value.isdigit() for value in columns[2:7]):
                    result['module_resources'][columns[1].removeprefix('+ ')] = dict(
                        zip(['lut', 'ff', 'distributed_ram', 'apm', 'drm36k'], map(int, columns[2:7])))
        if len(result['resources']) != 4 or 'timing' not in result:
            raise RuntimeError('Incomplete resource/timing report: '+str(snr))
    result['warning_codes'] = dict(Counter(re.findall(r'^W: ([\w-]+):', log, re.M)))
    result['constant_probe_loop_warnings'] = log.count('Loop was found during constant probe.')
    result['input_sha256'] = hashes
    dest = ARCHIVE/source.name
    for file in source.rglob('*'):
        if not file.is_file():
            continue
        if file.suffix in {'.json', '.log', '.snr', '.cmr', '.ccr', '.tcl', '.fdc', '.pds', '.v', '.txt'} or (netlist and file.suffix == '.vm'):
            copy_file(file, dest/file.relative_to(source))
    result['evidence'] = dest.relative_to(DOC).as_posix()
    dump(dest/'audited-summary.json', result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--selected', type=Path, required=True, help='Successful 1280x720 PDS build')
    parser.add_argument('--gate', type=Path, required=True, help='Passing gate run of that build')
    args = parser.parse_args()
    selected_path, gate_path = args.selected.resolve(), args.gate.resolve()
    selected = audit_build(selected_path, current=True, netlist=True)
    if not selected['success'] or (selected['width'], selected['height']) != (1280, 720):
        raise RuntimeError('Successful 1280x720 synthesis required')
    if digest(selected_path/'prj/avm_core.fdc') != digest(ROOT/'constraints/avm_core.fdc'):
        raise RuntimeError('Current constraint differs from selected build')
    gate = read(gate_path/'summary.json')
    log = (gate_path/'simulation.log').read_text(encoding='utf-8', errors='replace')
    if (not gate.get('success') or gate.get('compile_only') or not gate.get('pass_bit_exact')
            or 'PASS: tb_avm_gate' not in log or re.search(r'\*\* (?:Error|Fatal):', log)):
        raise RuntimeError('Passing functional netlist simulation required')
    if Path(gate['build']).resolve() != selected_path or (gate['width'], gate['height']) != (1280,720):
        raise RuntimeError('Gate run must use the selected full-resolution build')
    for path, expected in read(gate_path/'input-sha256.json').items():
        if digest(Path(path)) != expected:
            raise RuntimeError('Changed netlist simulation input: '+path)
    fixture = ROOT/'sim/data'/gate['fixture']
    actual = [int(s,16) for s in (gate_path/'gate_output.hex').read_text().splitlines()]
    expected = [int(s,16) for s in (fixture/'expected.hex').read_text().splitlines()]
    if actual != expected or len(actual) != gate['width']*gate['height']:
        raise RuntimeError('Gate output pixels differ from golden frame')
    gate['output_sha256'] = digest(gate_path/'gate_output.hex')
    for path in gate_path.iterdir():
        if path.is_file() and path.suffix in {'.log','.json','.vh','.hex','.png'}:
            copy_file(path, ARCHIVE/'gate'/path.name)
    gate['evidence'] = 'pds-validation/gate'
    report = dict(generated_at=datetime.now(timezone.utc).isoformat(), current_rtl_verified=True,
                  selected=selected, gate=gate, place_and_route=False, board_validation=False)
    dump(DOC/'pds-results.json', report)
    lines = ['# PDS 综合与完整帧网表验证', '',
             '本报告审计当前源码、构建输入、PDS 日志、官方原语模型及全部输出像素的 SHA256；由 collect_pds_report.py 生成。', '',
             f"选中构建：`{selected_path.name}`，器件 {selected['target']}，1280×720，核心 100MHz。", '',
             '| 资源 | 使用 | 容量 | 占用率 |', '|---|---:|---:|---:|']
    for name, resource in selected['resources'].items():
        lines.append(f"| {name} | {resource['used']:g} | {resource['capacity']:g} | {resource['percent']:.3f}% |")
    timing = selected['timing']
    lines += ['', f"ADS 综合估计 Fmax {timing['estimated_mhz']}MHz，setup slack {timing['setup_slack_ns']}ns。此数据尚未经过布局布线。", '',
              f"ModelSim 完整网表输出 {len(actual):,} 个像素全部与参考一致，SOF/EOL、统计与配置由测试平台核对。", '',
              '[网表仿真日志](pds-validation/gate/simulation.log) · [机器可读报告](pds-results.json)', '',
              '使用本机 Pango 官方原语模型，+delay_mode_zero / +notimingchecks，无 SDF；属于综合后功能仿真。没有完成板级管脚、物理 DDR/PCIe、摄像头或 RK3568 显示验证。', '',
              '历史优化实验原文保存在 [旧 PDS 报告](../../archive/reports-pre-layout/avm/pds-validation.md)。']
    (DOC/'pds-validation.md').write_text('\n'.join(lines)+'\n', encoding='utf-8')
    print(f'PASS: current RTL, synthesis inputs and {len(actual)} netlist pixels audited')
    print(DOC/'pds-validation.md')


if __name__ == '__main__':
    main()
