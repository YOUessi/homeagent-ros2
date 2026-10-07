#!/usr/bin/env python3
import json
import re
import struct
import sys
import zlib
from pathlib import Path


def read_pgm(path: Path):
    data = path.read_bytes()
    i = 0

    def token():
        nonlocal i
        while i < len(data):
            if data[i:i+1] == b"#":
                while i < len(data) and data[i:i+1] not in (b"\n", b"\r"):
                    i += 1
            elif data[i:i+1].isspace():
                i += 1
            else:
                break
        start = i
        while i < len(data) and not data[i:i+1].isspace():
            i += 1
        return data[start:i].decode("ascii")

    magic = token()
    if magic != "P5":
        raise ValueError(f"unsupported PGM format: {magic}")
    width = int(token())
    height = int(token())
    maxval = int(token())
    if maxval != 255:
        raise ValueError(f"unsupported PGM maxval: {maxval}")
    while i < len(data) and data[i:i+1].isspace():
        i += 1
    pixels = data[i:i + width * height]
    if len(pixels) != width * height:
        raise ValueError("truncated PGM")
    return width, height, pixels


def parse_map_yaml(path: Path):
    text = path.read_text(encoding="utf-8")
    resolution = float(re.search(r"^resolution:\s*([^\s]+)", text, re.M).group(1))
    origin_match = re.search(r"^origin:\s*\[([^\]]+)\]", text, re.M)
    origin = [float(v.strip()) for v in origin_match.group(1).split(",")]
    return resolution, origin


def set_px(img, width, height, x, y, rgb):
    if 0 <= x < width and 0 <= y < height:
        idx = (y * width + x) * 3
        img[idx:idx+3] = bytes(rgb)


def dot(img, width, height, x, y, radius, rgb):
    for yy in range(y - radius, y + radius + 1):
        for xx in range(x - radius, x + radius + 1):
            if (xx - x) ** 2 + (yy - y) ** 2 <= radius ** 2:
                set_px(img, width, height, xx, yy, rgb)


def line(img, width, height, x0, y0, x1, y1, rgb, thickness=2):
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    while True:
        dot(img, width, height, x0, y0, thickness, rgb)
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy


def write_png(path: Path, width, height, rgb):
    raw = bytearray()
    stride = width * 3
    for y in range(height):
        raw.append(0)
        raw.extend(rgb[y * stride:(y + 1) * stride])

    def chunk(kind, payload):
        return (
            struct.pack(">I", len(payload))
            + kind
            + payload
            + struct.pack(">I", zlib.crc32(kind + payload) & 0xFFFFFFFF)
        )

    png = bytearray(b"\x89PNG\r\n\x1a\n")
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(bytes(raw), 9))
    png += chunk(b"IEND", b"")
    path.write_bytes(png)


def main():
    if len(sys.argv) != 5:
        raise SystemExit(
            "usage: render_nav2_demo.py MAP.pgm MAP.yaml REPORT.json OUTPUT.png"
        )

    pgm_path, yaml_path, report_path, out_path = map(Path, sys.argv[1:])
    width, height, gray = read_pgm(pgm_path)
    resolution, origin = parse_map_yaml(yaml_path)
    report = json.loads(report_path.read_text(encoding="utf-8"))

    scale = 5
    out_w, out_h = width * scale, height * scale
    img = bytearray(out_w * out_h * 3)

    for y in range(height):
        for x in range(width):
            v = gray[y * width + x]
            # Occupancy PGM convention: white=free, black=occupied, gray=unknown.
            for sy in range(scale):
                row = (y * scale + sy) * out_w
                for sx in range(scale):
                    idx = (row + x * scale + sx) * 3
                    img[idx:idx+3] = bytes((v, v, v))

    def world_to_px(x, y):
        gx = int(round((x - origin[0]) / resolution))
        gy = int(round((y - origin[1]) / resolution))
        py = height - 1 - gy
        return gx * scale + scale // 2, py * scale + scale // 2

    traj = report.get("trajectory") or []
    if len(traj) >= 2:
        pts = [world_to_px(float(p[0]), float(p[1])) for p in traj]
        for a, b in zip(pts, pts[1:]):
            line(img, out_w, out_h, a[0], a[1], b[0], b[1], (0, 90, 255), 2)
        dot(img, out_w, out_h, pts[0][0], pts[0][1], 7, (0, 180, 0))
        dot(img, out_w, out_h, pts[-1][0], pts[-1][1], 7, (255, 140, 0))

    goal = report.get("skill_result", {}).get("goal_xyyaw", [0.8, 0.0, 0.0])
    gx, gy = world_to_px(float(goal[0]), float(goal[1]))
    dot(img, out_w, out_h, gx, gy, 9, (220, 0, 0))
    dot(img, out_w, out_h, gx, gy, 4, (255, 255, 255))

    write_png(out_path, out_w, out_h, img)
    print(
        json.dumps(
            {
                "output": str(out_path),
                "map_size": [width, height],
                "render_size": [out_w, out_h],
                "trajectory_samples": len(traj),
                "displacement_m": report.get("displacement_m"),
                "passed": report.get("passed"),
            },
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
