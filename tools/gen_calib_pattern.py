#!/usr/bin/env python3
"""
生成棋盘格标定板 SVG/PNG。

用法:
  python3 gen_calib_pattern.py                    # 生成默认 A4 打印版
  python3 gen_calib_pattern.py --cols 9 --rows 6  # 自定义格子数
  python3 gen_calib_pattern.py --size 30          # 自定义格子大小(mm)
  python3 gen_calib_pattern.py --screen           # 生成屏幕显示版(大格子)
"""

import argparse
import math

def gen_svg(cols: int, rows: int, square_mm: float, page_w_mm: int = 210,
            page_h_mm: int = 297) -> str:
    """生成可打印的 SVG 棋盘格"""
    board_w = cols * square_mm
    board_h = rows * square_mm
    margin_x = (page_w_mm - board_w) / 2
    margin_y = (page_h_mm - board_h) / 2

    lines = []
    lines.append(f'<svg xmlns="http://www.w3.org/2000/svg" '
                 f'width="{page_w_mm}mm" height="{page_h_mm}mm" '
                 f'viewBox="0 0 {page_w_mm} {page_h_mm}">')
    lines.append(f'<rect width="100%" height="100%" fill="white"/>')

    # 棋盘格
    for r in range(rows):
        for c in range(cols):
            if (r + c) % 2 == 0:
                x = margin_x + c * square_mm
                y = margin_y + r * square_mm
                lines.append(
                    f'<rect x="{x:.2f}" y="{y:.2f}" '
                    f'width="{square_mm:.2f}" height="{square_mm:.2f}" '
                    f'fill="black"/>')

    # 标注信息
    lines.append(
        f'<text x="105" y="10" font-size="4" text-anchor="middle" '
        f'fill="gray">{cols}x{rows}  {square_mm}mm/格</text>')
    lines.append('</svg>')
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description='生成棋盘格标定板')
    parser.add_argument('--cols', type=int, default=9,
                        help='棋盘格列数 (内角点=cols-1)')
    parser.add_argument('--rows', type=int, default=6,
                        help='棋盘格行数 (内角点=rows-1)')
    parser.add_argument('--size', type=float, default=25,
                        help='每个格子边长 (mm)')
    parser.add_argument('--screen', action='store_true',
                        help='屏幕显示模式 (大格子, 1280x720 自适应)')
    parser.add_argument('--output', '-o', default='calib_pattern.svg',
                        help='输出文件名')
    args = parser.parse_args()

    if args.screen:
        # 屏幕模式：1280x720 内自适应，用大格子
        svg = gen_svg_for_screen(args.cols, args.rows)
    else:
        svg = gen_svg(args.cols, args.rows, args.size)

    with open(args.output, 'w') as f:
        f.write(svg)
    print(f'✅ 已生成: {args.output}')
    print(f'   棋盘格: {args.cols}x{args.rows} 内角点 ({(args.cols-1)}x{(args.rows-1)})')
    if args.screen:
        print(f'   模式: 屏幕显示 (全屏后相机拍摄)')
    else:
        print(f'   格子大小: {args.size}mm')
        print(f'   建议: 打出来贴在硬纸板上')


def gen_svg_for_screen(cols: int, rows: int) -> str:
    """生成适合屏幕显示的棋盘格 (1280x720 内最大化)"""
    sw, sh = 1280, 720
    margin = 40
    avail_w = sw - 2 * margin
    avail_h = sh - 2 * margin
    sq = min(avail_w / cols, avail_h / rows)
    board_w = cols * sq
    board_h = rows * sq
    ox = (sw - board_w) / 2
    oy = (sh - board_h) / 2

    lines = []
    lines.append(f'<svg xmlns="http://www.w3.org/2000/svg" '
                 f'width="{sw}" height="{sh}" viewBox="0 0 {sw} {sh}">')
    lines.append(f'<rect width="100%" height="100%" fill="white"/>')
    for r in range(rows):
        for c in range(cols):
            if (r + c) % 2 == 0:
                x = ox + c * sq
                y = oy + r * sq
                lines.append(
                    f'<rect x="{x:.1f}" y="{y:.1f}" '
                    f'width="{sq:.1f}" height="{sq:.1f}" fill="black"/>')
    lines.append(f'<text x="{sw//2}" y="{sh-10}" font-size="16" '
                 f'text-anchor="middle" fill="gray">'
                 f'{cols}x{rows} — 全屏显示后拍照</text>')
    lines.append('</svg>')
    return '\n'.join(lines)


if __name__ == '__main__':
    main()
