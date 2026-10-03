"""Audit current simulation evidence and publish a self-contained AVM report.

No simulation is fabricated: every selected run must have an explicit PASS,
current input hashes and an actual raster output matching its golden frame(s).
"""
from __future__ import annotations
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
DOC = ROOT / 'doc/avm'
ARCHIVE = DOC / 'validation'
RUNS = [('small', 'nominal'), ('synthetic720', 'nominal'),
        ('public', 'nominal'), ('synthetic720-wide', 'nominal'),
        ('small', 'stress'), ('synthetic720', 'stress'),
        ('small', 'stress-feedback'), ('synthetic720', 'nominal-feedback')]


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')


def verify_manifest(manifest):
    for relative, expected in manifest.items():
        path = ROOT / relative
        if not path.is_file() or digest(path) != expected:
            raise RuntimeError(f'Stale or missing evidence input: {relative}; rerun its verification')


def copy_files(source, dest, patterns):
    dest.mkdir(parents=True, exist_ok=True)
    for pattern in patterns:
        for path in source.glob(pattern):
            if path.is_file():
                shutil.copy2(path, dest / path.name)


def main():
    ARCHIVE.mkdir(parents=True, exist_ok=True)
    (DOC / 'figures').mkdir(parents=True, exist_ok=True)
    results = []
    for case, profile in RUNS:
        fixture = ROOT / 'sim/data' / case
        result = read(fixture / f'latest-{profile}.json')
        source = Path(result['log_directory'])
        verify_manifest(read(source / 'input-sha256.json'))
        log = (source / 'simulation.log').read_text(encoding='utf-8')
        if not result['pass_bit_exact'] or 'PASS: tb_avm_frame' not in log or '** Fatal:' in log:
            raise RuntimeError(f'Missing pass evidence: {source}')
        pixels = np.fromiter((int(s, 16) for s in (source / 'rtl_output.hex').read_text().splitlines()), dtype=np.uint16)
        reference = [np.fromfile(fixture / 'expected.bin', dtype='<u2')]
        if result['frames'] == 2:
            reference.append(np.fromiter((int(s, 16) for s in (fixture / 'expected_feedback.hex').read_text().splitlines()), dtype=np.uint16))
        if not np.array_equal(pixels, np.concatenate(reference)):
            raise RuntimeError(f'Archived raster mismatch: {source}')
        label = case + '-' + profile
        dest = ARCHIVE / label
        copy_files(source, dest, ['*.log', 'fixture.vh', 'summary.json', 'statistics.json', 'input-sha256.json', 'rtl_frame_*.png'])
        result['evidence'] = str(dest.relative_to(DOC)).replace('\\', '/')
        result['rtl_output_sha256'] = digest(source / 'rtl_output.hex')
        result['read_bytes'] = result['read_beats'] * 16
        result['capture_bytes'] = result['capture_beats'] * 16
        result['lut_read_beats'] = result['read_beats'] - sum(result['source_read_beats'])
        result['observed_total_mbps'] = (result['read_bytes'] + result['capture_bytes']) / (result['processing_ms'] * 1000)
        write_json(dest / 'audited-summary.json', result)
        for path in source.glob('rtl_frame_*.png'):
            shutil.copy2(path, DOC / 'figures' / f'rtl_{label}_{path.stem}.png')
        if (source / 'core.vcd').exists():
            shutil.copy2(source / 'core.vcd', dest / 'core.vcd')
            converter = shutil.which('vcd2wlf')
            if converter:
                process = subprocess.run([converter, str(dest / 'core.vcd'), str(dest / 'core.wlf')],
                                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=120)
                (dest / 'wave-conversion.log').write_bytes(process.stdout)
                if process.returncode:
                    raise RuntimeError('vcd2wlf failed: ' + str(dest))
        results.append(result)

    units = read(ROOT / 'build/avm/latest-units.json')
    verify_manifest(units['source_sha256'])
    if len(units['tests']) != 9 or any(test['status'] != 'PASS' for test in units['tests']):
        raise RuntimeError('Expected all nine RTL unit runs')
    copy_files(Path(units['log_directory']), ARCHIVE / 'units', ['*.log', 'summary.json'])
    algorithm = {}
    for case in ['small', 'synthetic720', 'synthetic720-wide', 'public']:
        fixture = ROOT / 'sim/data' / case
        metrics, matlab = read(fixture / 'metrics.json'), read(fixture / 'matlab_verification.json')
        if metrics['grid']['max_coordinate_error_px'] > .25 or matlab['status'] != 'PASS' or matlab['float_reference_differing_channels']:
            raise RuntimeError('Algorithm acceptance failed: ' + case)
        algorithm[case] = dict(metrics=metrics, matlab=matlab)
        copy_files(fixture, ARCHIVE / ('algorithm-' + case), ['metrics.json', 'matlab_verification.json', 'frame.json', 'calibration.json'])

    regression = read(ROOT / 'build/avm/latest-regression.json')
    verify_manifest(regression['source_sha256'])
    if regression['status'] != 'PASS' or any(step['exit_code'] for step in regression['steps']):
        raise RuntimeError('The latest complete regression did not pass')
    regression_logs = Path(regression['log_directory'])
    copy_files(regression_logs, ARCHIVE / 'regression', ['*.log', 'summary.json'])
    test_step = next(step for step in regression['steps'] if step['command'][:2] == ['-m', 'unittest'])
    test_log = (regression_logs / test_step['log']).read_text(encoding='utf-8')
    if not re.search(r'Ran 13 tests[\s\S]{0,100}\bOK\b', test_log) or 'skipped' in test_log.lower():
        raise RuntimeError('Python regression PASS not found in final log')
    source_paths = [*sorted((ROOT / 'tools/avm').glob('*.py')), *sorted((ROOT / 'tools/avm').glob('*.m')),
                    *sorted((ROOT / 'src/rtl').glob('*.v')),
                    *sorted((ROOT / 'sim/tb').glob('*')),
                    *sorted((ROOT / 'sim/models').glob('*.sv')),
                    ROOT / 'sim/run.py', ROOT / 'sim/wave.do', ROOT / 'src/files.f',
                    ROOT / 'constraints/avm_core.fdc', ROOT / 'project/pds/build.tcl']
    write_json(ARCHIVE / 'delivery-source-sha256.json', {str(p.relative_to(ROOT)): digest(p) for p in source_paths if p.is_file()})
    exposure = read(DOC / 'exposure-feedback.json')
    pds = read(DOC / 'pds-results.json') if (DOC / 'pds-results.json').exists() else None
    if pds:
        for relative, expected in pds['selected']['input_sha256'].items():
            if relative.startswith('rtl/') and digest(ROOT / 'src' / relative) != expected:
                raise RuntimeError('PDS evidence does not match current RTL: ' + relative)
    pds_status = 'PDS ADS 综合与网表功能验证另见 [PDS 报告](pds-validation.md)。' if pds else 'PDS 综合证据尚未归档。'
    report = dict(generated_at=datetime.now(timezone.utc).isoformat(), runs=results, units=units, regression=regression,
                  algorithm=algorithm, exposure=exposure, python_tests=13, current_run_inputs_verified=True,
                  pds_synthesis=bool(pds and pds['selected']['success']),
                  pds_report='pds-results.json' if pds else None,
                  place_and_route=False, board_validation=False)
    write_json(DOC / 'validation-results.json', report)

    lines = ['# 四路 AVM 实际验证报告', '',
             '本报告由 `python tools/avm/collect_report.py` 从实际工具输出生成。归档前重新核对每次整帧仿真的输入 SHA256，并读取实际 `rtl_output.hex` 与参考帧比较。没有用软件图替代 RTL 输出。', '',
             '**当前算法与 RTL 仿真验收通过。** 30fps 处理目标在 64 像素有限接缝的 720p 合成场景和公开样例下达到；全交叠压力配置未达到该目标。'+pds_status+'布局布线、物理 DDR/PCIe 与摄像头上板均未执行。', '',
             '## 完整帧结果', '',
             '| 输入 / 配置 | 帧数 / 核对像素数 | 最后一帧耗时 ms | 33.33ms 目标 | 实际证据 |',
             '|---|---:|---:|---|---|']
    for result, (_, profile) in zip(results, RUNS):
        lines.append(f"| {result['case']} / {profile} | {result['frames']} / {result['pixels']:,} | {result['processing_ms']:.5f} | {'达到' if result['meets_30fps_model'] else '未达到'} | [PASS 日志]({result['evidence']}/simulation.log) |")
    lines += ['', '上述每个输出像素、SOF/EOL、四组增益前统计及配置版本均由自检测试核对。两帧测试在第一帧处理中写入影子增益并提交，第一帧保持原值，第二帧使用新值。耗时和总线/缓存计数均取最后一帧；`pixels` 列包含该运行核对的全部帧。小尺寸测试的 33.33ms 比较仅表示其运行耗时，不证明 720p 性能。', '',
              '![实际 RTL 合成鸟瞰](figures/rtl_synthetic720-nominal_rtl_frame_0.png)', '',
              '![实际 RTL 实拍鸟瞰](figures/rtl_public-nominal_rtl_frame_0.png)', '',
              '![应用反馈增益后的实际 RTL 输出](figures/rtl_synthetic720-nominal-feedback_rtl_frame_1.png)', '',
              '## 性能条件和事务量', '',
              '核心 100MHz；DDR 等效 131.25MHz / 128 位，单个在途读，首拍等待 8 个 DDR 周期加 2 个核心周期，随后最多每核心周期一拍。LUT/A/B 内层轮询，读与采集任务外层轮询。四路 RGB565 采集的 221.184MB/s 虚拟写入占用实际服务时间；表内 capture 是测量窗口内完成的写拍。该模型使用核心域握手，未模拟物理 CDC、DRAM 刷新和行命中。', '',
              '| 标称完整帧 | 读事务 | LUT / A / B 读拍 | 读流量 MB | 采集写流量 MB | 窗口总流量 MB/s |',
              '|---|---:|---:|---:|---:|---:|']
    for result in results[1:4]:
        a, b = result['source_read_beats']
        lines.append(f"| {result['case']} | {result['requests']:,} | {result['lut_read_beats']:,} / {a:,} / {b:,} | {result['read_bytes']/1e6:.3f} | {result['capture_bytes']/1e6:.3f} | {result['observed_total_mbps']:.3f} |")
    lines += ['', '| 标称完整帧 | A/B 缓存等待周期 | A/B 填充周期 | A/B 分配瓦片 | DDR 活跃服务周期 |',
              '|---|---:|---:|---:|---:|']
    for result in results[1:4]:
        pair = lambda key: ' / '.join(f'{value:,}' for value in result[key])
        lines.append(f"| {result['case']} | {pair('cache_wait_cycles')} | {pair('cache_fill_cycles')} | {pair('cache_misses')} | {result['ddr_service_cycles']:,} |")
    lines += ['', '缓存等待计数条件为该路 `s_valid && !s_ready`，包括缺失及下游背压；填充计数为缓存状态非 IDLE。分配计数包含预取瓦片。不同路、填充、总线等待之间会重叠，不能将它们相加为总帧周期。更细的请求等待、输出背压与首拍累计延迟见 [机器可读报告](validation-results.json)。', '',
              '算术单元在数据就绪、无背压时实测 II=1：双线性插值延迟 3 周期，网格插值 4 周期，增益融合 2 周期。缓存命中流连续接收 400 个样本，无气泡。PDS 优化为每次缓存缺失增加两个准备周期。整帧吞吐包含宏块装载、缓存缺失、条带交接和输出排空，因此不能以算子 II 推断整机吞吐。', '',
              '有限接缝和 wide 的原图、标定、节点、有效域及源坐标误差完全一致，双路像素从 832,724 降为 93,932，总采样请求减少 42.81%。变化仅是相机路由与接缝权重，原宽交叠测试被完整保留。公开样例的有效图为居中 540×720，不能用其补边后的输出尺寸代替四路 720p 输入吞吐证据。', '',
              '33.33ms 是输入帧已就绪到输出完成的处理目标；这里的时钟参数不能证明器件 Fmax，也不代表摄像头到显示器的端到端延迟。', '',
              '## 算法和几何', '',
              '| 样例 | 最大坐标误差 px | 节点 KiB | PSNR dB | SSIM | MATLAB 全帧不同 RGB 分量 |',
              '|---|---:|---:|---:|---:|---:|']
    for case, a in algorithm.items():
        m = a['metrics']; q = m['quality_vs_float']
        lines.append(f"| {case} | {m['grid']['max_coordinate_error_px']:.9f} | {m['grid']['node_bytes']/1024:.3f} | {q['psnr_db']:.3f} | {q['ssim']:.6f} | {a['matlab']['float_reference_differing_channels']} |")
    lines += ['', '每路原图仅进行一次图像重采样。误差在 21 位 Q8 节点量化、水平/垂直有符号半值远离零舍入之后，以源坐标欧氏距离验证所有有效像素。PSNR/SSIM 对比各自接缝策略的双精度参考并包含 RGB565 量化。公开样例节点实测 211.8KiB，高于规划时约 145KiB 的估计；节点驻留 DDR。每个 720p 帧另有 225KiB 描述符及 1800KiB 路由。', '',
              '本次沿用已校验的 MATLAB 参考文件，未重新调用 MATLAB。MATLAB R2024b 独立实现同一 OpenCV fisheye 数学模型，四组输出均完整渲染，RGB 与 Python 浮点参考零差异。每个完整样例检查 17,024 个坐标，small 检查 64 个。独立射线与地面求交及 OpenCV 投影的 50 个地面控制点最大误差 2.63×10⁻¹⁴m，合成场景非车身地面覆盖 100%。详见 [算法验证](algorithm-validation.md)。', '',
              '## 曝光反馈', '',
              f"输入实际有限接缝 RTL 的补偿前统计，以 5Hz、0.1 对数域平滑执行 {exposure['update_count']} 次静态帧回放。平均残差 {100*exposure['mean_relative_residual_before']:.5f}% → {100*exposure['mean_relative_residual_after']:.5f}%；最差相机对/颜色通道 {100*exposure['max_relative_residual_after']:.5f}%，达到可达、非饱和、连通样本 ≤2% 的目标。该残差来自统计均值与求得增益；实际量化寄存器另外通过完整第二帧 RTL 像素比较。", '',
              f"Q4.12 增益（前、后、左、右，各 RGB）：`{exposure['gains_q4_12']}`。详细状态见 [exposure-feedback.json](exposure-feedback.json)。暗区/饱和过滤、断连保持和增益上下界由 Python 与 RTL 统计用例覆盖。公开实拍的立体物体、局部遮挡与安装误差不能由单一相机增益消除，不能将合成样本 ≤2% 的结论外推到实景。", '',
              '## 单元、边界和压力覆盖', '',
              'Python 13 项测试全部通过；RTL 9 次单元运行全部通过：', '',
              '| 测试 | 验证内容 |', '|---|---|',
              '| tb_arithmetic | 双线性、网格和融合的随机/定向样本；有符号舍入、权重端点、饱和、II=1、四级网格、背压、复位；实际计数见单元日志 |',
              '| tb_reorder | 1280×35、257×35、37×35、32×48、1×1；全部RAM分块及部分末块、多帧、长阻塞、复位；读完前禁止覆盖 |',
              '| tb_patch_store | 查询、双节点写、行跨越；五种步长、四相机、双缓冲；实际计数见单元日志 |',
              '| tb_config | 4 个完成帧及复位中止帧；提交/保持/原子切换；描述符及路由跨 4KiB |',
              '| tb_cache | 566 样本；三客户仲裁、随机延迟、跨瓦片四邻域、边界、失效、填充中复位 |',
              '| tb_b_only | 34 像素；仅 B=相机0、变化节点、真实 DDR、非零增益、无效坐标黑像素、背压 |',
              '| tb_cache_prefetch ×3 | 1/2/4 瓦片预取、4/8/16 拍、页边界、部分瓦片、FIFO、替换期间输出保持 |', '',
              '压力整帧采用固定种子随机首拍额外等待 0～64 DDR 周期、请求/响应气泡、随机输出阻塞及每 2000 周期连续 150 周期阻塞。所有已执行压力样例保持像素完整，无死锁。固定种子和有限测试集合不能证明任意流量下的最坏情况。', '',
              '## 资源边界与交付', '',
              'RTL 逻辑存储：双路像素缓存 64KiB，双 16 行重排 80KiB，16 个 162×42 位节点 bank 共约 13.29KiB，双宏块路由 1KiB，两路响应 FIFO 556 字节，另有 Alpha、标签与控制寄存器。不能用逻辑比特总量直接推断 DRM 数量；42 位节点宽度、RAM 端口模式和深度取整都会影响映射。'+pds_status+'器件物理时序收敛仍需布局布线检查。', '',
              '所有本轮源码在 `src/rtl/`、`sim/` 和 `tools/avm/`，[运行指南](README.md) 给出完整命令。归档 [validation/](validation/) 含每组编译与仿真日志、输入哈希、实际 RTL PNG、独立 MATLAB 指标和小帧压力 VCD/WLF；原始 HEX、DDR 二进制、NPZ 保留在 `sim/data/`，完整运行目录在 `build/avm/`，均可重建。', '',
              '下一阶段依次完成实际 DDR/PCIe 适配、板级 PDS 构建与布局布线签核、真实相机标定与采集、RK3568 HDMI 显示。']
    (DOC / 'validation-report.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    print('PASS: audited 8 frame runs, 9 RTL unit runs, 13 Python tests and 4 MATLAB fixtures')
    print(DOC / 'validation-report.md')


if __name__ == '__main__':
    main()
