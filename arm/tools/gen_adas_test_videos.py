#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ADAS 合成测试视频生成器（选题三 ARM 侧开发验证用）

生成 4 段对应赛题现场场景的合成视频（1280×720@30fps，RGB→ffmpeg→mp4）：
  scenario1_intersection.mp4  十字路口：斑马线 + 红绿灯（红/绿交替）+ 地面车道箭头
  scenario2_lane_lines.mp4    虚实线变道：左实右虚 5s → 左虚右实 5s
  scenario3_navigation.mp4    导航道路：左上角导航面板蓝色箭头（↑/→/← 轮换）
  scenario4_pedestrian.mp4     行人闯红灯：红灯 + 斑马线 + 行人横穿

用途说明：
- 验证经典视觉算法（车道线虚实分析、导航箭头识别）与决策链路/UI 叠加。
- RKNN 模型训练于实拍图像，对合成图形的检出率不保证；NPU 相关逻辑
  （红绿灯/行人检测）由单元测试覆盖，真实素材到位后再标定。

用法：
  python3 tools/gen_adas_test_videos.py [--outdir /tmp/adas_videos] [--codec libx264]
  （若 ffmpeg 无 libx264，用 --codec mpeg4）

依赖：numpy、ffmpeg（本板均已安装）
"""

import argparse
import os
import subprocess
import sys

import numpy as np

W, H, FPS = 1280, 720, 30


# ---------------- 基础绘图（numpy 直接操作，无 cv2 依赖） ----------------

def fill_rect(img, x0, y0, x1, y1, color):
    """实心矩形（含边界；自动裁剪到画面内）"""
    x0, y0 = max(0, x0), max(0, y0)
    x1, y1 = min(W, x1), min(H, y1)
    if x1 <= x0 or y1 <= y0:
        return
    img[y0:y1, x0:x1] = color


def fill_circle(img, cx, cy, r, color):
    """实心圆"""
    x0, x1 = max(0, cx - r), min(W, cx + r + 1)
    y0, y1 = max(0, cy - r), min(H, cy + r + 1)
    if x1 <= x0 or y1 <= y0:
        return
    yy, xx = np.mgrid[y0:y1, x0:x1]
    mask = (xx - cx) ** 2 + (yy - cy) ** 2 <= r * r
    img[y0:y1, x0:x1][mask] = color


def fill_triangle(img, p0, p1, p2, color):
    """实心三角形（重心坐标法）"""
    x_min = max(0, int(min(p0[0], p1[0], p2[0])))
    x_max = min(W, int(max(p0[0], p1[0], p2[0])) + 1)
    y_min = max(0, int(min(p0[1], p1[1], p2[1])))
    y_max = min(H, int(max(p0[1], p1[1], p2[1])) + 1)
    if x_max <= x_min or y_max <= y_min:
        return
    yy, xx = np.mgrid[y_min:y_max, x_min:x_max]
    yy = yy.astype(np.float64)
    xx = xx.astype(np.float64)
    # 重心坐标
    x0, y0 = p0
    x1, y1 = p1
    x2, y2 = p2
    den = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2)
    if abs(den) < 1e-9:
        return
    l0 = ((y1 - y2) * (xx - x2) + (x2 - x1) * (yy - y2)) / den
    l1 = ((y2 - y0) * (xx - x2) + (x0 - x2) * (yy - y2)) / den
    l2 = 1.0 - l0 - l1
    mask = (l0 >= 0) & (l1 >= 0) & (l2 >= 0)
    img[y_min:y_max, x_min:x_max][mask] = color


def draw_vline(img, x, y0, y1, width, color):
    """竖实线"""
    fill_rect(img, x - width // 2, y0, x + width // 2 + 1, y1, color)


def draw_vline_dashed(img, x, y0, y1, width, color, on=40, off=30):
    """竖虚线（规律段长/间隙——虚线判别的关键特征）"""
    y = y0
    while y < y1:
        fill_rect(img, x - width // 2, y, x + width // 2 + 1, min(y + on, y1), color)
        y += on + off


def draw_arrow_shape(img, direction, cx, cy, size, color):
    """箭头形状（三角头 + 矩形杆）"""
    half = size // 2
    head = half * 2 // 3
    shaft = size // 10
    if direction == 'up':
        fill_triangle(img, (cx, cy - half), (cx + head, cy - half + head),
                      (cx - head, cy - half + head), color)
        fill_rect(img, cx - shaft, cy - half + head, cx + shaft + 1, cy + half, color)
    elif direction == 'left':
        fill_triangle(img, (cx - half, cy), (cx - half + head, cy - head),
                      (cx - half + head, cy + head), color)
        fill_rect(img, cx - half + head, cy - shaft, cx + half, cy + shaft + 1, color)
    elif direction == 'right':
        fill_triangle(img, (cx + half, cy), (cx + half - head, cy - head),
                      (cx + half - head, cy + head), color)
        fill_rect(img, cx - half, cy - shaft, cx + half - head, cy + shaft + 1, color)


# ---------------- 场景公共元素 ----------------

ROAD_TOP = 300          # 路面起始 y
SKY_COLOR = np.array([95, 110, 130], dtype=np.uint8)
ROAD_COLOR = np.array([52, 52, 58], dtype=np.uint8)
LINE_COLOR = np.array([235, 235, 235], dtype=np.uint8)
ZEBRA_COLOR = np.array([225, 225, 225], dtype=np.uint8)
NAV_PANEL = np.array([38, 38, 46], dtype=np.uint8)
NAV_BLUE = np.array([66, 133, 244], dtype=np.uint8)   # 导航蓝（HSV 饱和亮色）
PERSON_COLOR = np.array([50, 80, 220], dtype=np.uint8)  # BGR 红色行人


def draw_road_base(img):
    """道路背景：天空 + 路面 + 三车道分隔区域"""
    img[:] = SKY_COLOR
    fill_rect(img, 0, ROAD_TOP, W, H, ROAD_COLOR)


def draw_zebra(img, y0=470, y1=560):
    """斑马线（白色条纹）"""
    x = 320
    while x < 980:
        fill_rect(img, x, y0, x + 36, y1, ZEBRA_COLOR)
        x += 72


def draw_traffic_light(img, state, cx=640, top=30):
    """红绿灯：黑箱 + 三灯圆（state: 'red'/'green'/'yellow'）"""
    fill_rect(img, cx - 55, top, cx + 55, top + 200, (20, 20, 20))
    lights = [('red', 255, 0, 0), ('yellow', 255, 210, 0), ('green', 0, 255, 0)]
    for i, (name, r, g, b) in enumerate(lights):
        cy = top + 40 + i * 60
        if state == name:
            fill_circle(img, cx, cy, 26, (b, g, r))  # numpy 是 RGB
        else:
            fill_circle(img, cx, cy, 26, (35, 35, 35))


def draw_lane_arrows(img):
    """地面车道箭头（左/直/右）"""
    draw_arrow_shape(img, 'left', 350, 620, 90, LINE_COLOR)
    draw_arrow_shape(img, 'up', 640, 620, 90, LINE_COLOR)
    draw_arrow_shape(img, 'right', 930, 620, 90, LINE_COLOR)


def draw_person(img, cx, cy=515):
    """行人（头 + 身体，简单形状）"""
    fill_circle(img, cx, cy - 40, 18, PERSON_COLOR)
    fill_rect(img, cx - 14, cy - 22, cx + 15, cy + 45, PERSON_COLOR)


# ---------------- 视频输出 ----------------

class VideoWriter:
    """ffmpeg rawvideo 管道封装"""

    def __init__(self, path, fps=FPS, codec='libx264'):
        self.proc = subprocess.Popen(
            ['ffmpeg', '-y', '-loglevel', 'error',
             '-f', 'rawvideo', '-pix_fmt', 'rgb24',
             '-s', f'{W}x{H}', '-r', str(fps), '-i', '-',
             '-c:v', codec, '-pix_fmt', 'yuv420p', path],
            stdin=subprocess.PIPE)

    def write(self, img):
        self.proc.stdin.write(np.ascontiguousarray(img, dtype=np.uint8).tobytes())

    def close(self):
        self.proc.stdin.close()
        ret = self.proc.wait()
        if ret != 0:
            print(f'[warn] ffmpeg exited with {ret}', file=sys.stderr)


# ---------------- 四个场景 ----------------

def gen_scenario1(outdir, codec):
    """场景1 十字路口：红灯 4s → 绿灯 4s → 黄灯 2s → 红灯 2s（12s）"""
    path = os.path.join(outdir, 'scenario1_intersection.mp4')
    writer = VideoWriter(path, codec=codec)
    total = 12 * FPS
    for f in range(total):
        t = f / FPS
        img = np.zeros((H, W, 3), dtype=np.uint8)
        draw_road_base(img)
        draw_zebra(img)
        draw_lane_arrows(img)
        if t < 4.0:
            state = 'red'
        elif t < 8.0:
            state = 'green'
        elif t < 10.0:
            state = 'yellow'
        else:
            state = 'red'
        draw_traffic_light(img, state)
        writer.write(img)
    writer.close()
    print(f'[ok] {path}（红→绿→黄→红，验证：红灯停/绿灯行/红灯强制行驶报警）')


def gen_scenario2(outdir, codec):
    """场景2 虚实线：左实右虚 5s → 左虚右实 5s（10s）"""
    path = os.path.join(outdir, 'scenario2_lane_lines.mp4')
    writer = VideoWriter(path, codec=codec)
    total = 10 * FPS
    for f in range(total):
        t = f / FPS
        img = np.zeros((H, W, 3), dtype=np.uint8)
        draw_road_base(img)
        # 本车车道：左线 x=400，右线 x=880（ROI y∈[360,662] 内可见）
        left_solid = t < 5.0
        if left_solid:
            draw_vline(img, 400, 340, 700, 8, LINE_COLOR)
            draw_vline_dashed(img, 880, 340, 700, 8, LINE_COLOR)
        else:
            draw_vline_dashed(img, 400, 340, 700, 8, LINE_COLOR)
            draw_vline(img, 880, 340, 700, 8, LINE_COLOR)
        writer.write(img)
    writer.close()
    print(f'[ok] {path}（左实右虚→左虚右实，验证：虚线变道/实线保持/强制变道报警）')


def gen_scenario3(outdir, codec):
    """场景3 导航：左上角导航面板，蓝色箭头 ↑4s → →4s → ←4s（12s）"""
    path = os.path.join(outdir, 'scenario3_navigation.mp4')
    writer = VideoWriter(path, codec=codec)
    total = 12 * FPS
    for f in range(total):
        t = f / FPS
        img = np.zeros((H, W, 3), dtype=np.uint8)
        draw_road_base(img)
        # 道路两侧线（保持真实感）
        draw_vline(img, 200, 340, 700, 6, LINE_COLOR)
        draw_vline(img, 1080, 340, 700, 6, LINE_COLOR)
        # 导航面板（左上角，默认 ROI x∈[0,32%] y∈[0,30%] 内）
        fill_rect(img, 30, 30, 250, 220, NAV_PANEL)
        if t < 4.0:
            direction = 'up'
        elif t < 8.0:
            direction = 'right'
        else:
            direction = 'left'
        draw_arrow_shape(img, direction, 140, 125, 130, NAV_BLUE)
        writer.write(img)
    writer.close()
    print(f'[ok] {path}（导航箭头 ↑/→/←，验证：导航跟随与实线约束）')


def gen_scenario4(outdir, codec):
    """场景4 行人闯红灯：红灯 + 行人 4s 横穿斑马线（8s）"""
    path = os.path.join(outdir, 'scenario4_pedestrian.mp4')
    writer = VideoWriter(path, codec=codec)
    total = 8 * FPS
    for f in range(total):
        t = f / FPS
        img = np.zeros((H, W, 3), dtype=np.uint8)
        draw_road_base(img)
        draw_zebra(img)
        draw_traffic_light(img, 'red')
        # 行人 0-4s 从左向右横穿斑马线，4-8s 停在路中
        if t < 4.0:
            px = int(300 + (t / 4.0) * 680)
        else:
            px = 980
        draw_person(img, px)
        writer.write(img)
    writer.close()
    print(f'[ok] {path}（红灯+行人横穿，验证：行人闯红灯刹车）')


def main():
    parser = argparse.ArgumentParser(description='ADAS 合成测试视频生成器')
    parser.add_argument('--outdir', default='/tmp/adas_videos',
                        help='输出目录（默认 /tmp/adas_videos）')
    parser.add_argument('--codec', default='libx264',
                        help='ffmpeg 编码器（默认 libx264；不可用时用 mpeg4）')
    args = parser.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    gen_scenario1(args.outdir, args.codec)
    gen_scenario2(args.outdir, args.codec)
    gen_scenario3(args.outdir, args.codec)
    gen_scenario4(args.outdir, args.codec)
    print(f'\n全部生成完毕 → {args.outdir}')
    print('板端验证：bash run.sh → ADAS 模式 → 文件回放逐个播放')


if __name__ == '__main__':
    main()
