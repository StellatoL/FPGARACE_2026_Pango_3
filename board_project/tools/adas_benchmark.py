#!/usr/bin/env python3
"""Collect an ARM report from live FPGA input. PC playback supplies HDMI; this tool opens no video files."""
import argparse
import json
import os
import subprocess
from pathlib import Path


def main():
    root = Path(__file__).resolve().parents[1]
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--binary', type=Path, default=root/'build_adas/run_img_recv_demo.sh')
    p.add_argument('--duration', type=float, default=60)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--window-size', default='1280x720')
    p.add_argument('--single-view', action='store_true', help='initial layout; confirmed FPGA source remains authoritative')
    args = p.parse_args()
    if not 0 < args.duration <= 86400:
        p.error('duration must be in (0,86400]')
    args.output.mkdir(parents=True, exist_ok=True)
    report = args.output.resolve()/'report.json'
    if report.exists():
        p.error('output already contains report.json; use a fresh directory')
    command = [str(args.binary.resolve()), '--mode', 'adas', '--quit-after', str(args.duration),
               '--report', str(report), '--screenshot', str(args.output.resolve()/'display.png'),
               '--window-size', args.window_size]
    if args.single_view:
        command.append('--single-view')
    print('电脑端播放U盘素材并通过HDMI输入FPGA；请在打开的窗口中点击原有“采集视频”。', flush=True)
    with (args.output/'runtime.log').open('w', encoding='utf-8') as log:
        done = subprocess.run(command, cwd=args.output, env=os.environ.copy(), stdout=log,
                              stderr=subprocess.STDOUT, timeout=args.duration+30)
    if done.returncode or not report.exists():
        raise SystemExit('Application failed or report missing; inspect runtime.log')
    data = json.loads(report.read_text(encoding='utf-8'))
    if data.get('captured_frames', 0) == 0:
        raise SystemExit('No FPGA frames captured. This run is not a performance or perception pass.')
    print('Report captured. Separate model accuracy, confirmation delay and external input-to-display measurement are required.')


if __name__ == '__main__':
    main()
