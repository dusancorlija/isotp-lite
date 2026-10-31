#!/usr/bin/env python3
"""Draw a candump-style ISO-TP trace as an SVG timeline. Standard library only.

    ./build/trace 100 4 0x01 > trace.log
    python3 tools/timeline.py trace.log docs/timeline.svg
"""
import re
import sys

LINE = re.compile(r"\(\s*(\d+)\.(\d+)\)\s+\S+\s+([0-9A-F]+)(?:##[0-9A-F]|#)([0-9A-F]*)")
KIND = {0: ("SF", "#5b8def"), 1: ("FF", "#15171a"), 2: ("CF", "#e2531a"), 3: ("FC", "#2f9e6e")}
FRAME_US = 280  # 8-byte frame incl. intermission at 500 kbit/s, for box width


def main(src, dst):
    frames = []
    for line in open(src):
        m = LINE.search(line)
        if m:
            t = int(m.group(1)) * 1_000_000 + int(m.group(2))
            frames.append((t, int(m.group(3), 16), bytes.fromhex(m.group(4))))
    if not frames:
        sys.exit("no frames")

    ids = sorted({f[1] for f in frames})
    t_end = frames[-1][0]
    W, L, R = 980, 120, 30
    lane_h, top = 70, 64
    H = top + lane_h * len(ids) + 60
    sx = (W - L - R) / t_end

    out = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" '
        'font-family="Inter, -apple-system, Segoe UI, sans-serif" font-size="12">',
        f'<rect width="{W}" height="{H}" fill="#fff"/>',
        f'<text x="{L}" y="28" font-size="14" font-weight="600" fill="#15171a">'
        f"ISO-TP transfer on a 500 kbit/s bus: {len(frames)} frames in {t_end / 1000:.1f} ms</text>",
    ]
    fc = next((d for _, _, d in frames if d[0] >> 4 == 3), None)
    if fc:
        st = fc[2]
        st_txt = f"{st} ms" if st <= 0x7F else f"{(st - 0xF0) * 100} µs"
        out.append(f'<text x="{L}" y="46" fill="#7a7e85">Receiver asks for block size {fc[1]} '
                   f'and STmin {st_txt}</text>')
    ax = top + lane_h * len(ids) + 6
    step = 1000 if t_end < 20000 else 5000
    for t in range(0, t_end + 1, step):
        x = L + t * sx
        out.append(f'<line x1="{x:.1f}" x2="{x:.1f}" y1="{top + 10}" y2="{ax}" stroke="#f3f1ed"/>')
        out.append(f'<text x="{x:.1f}" y="{ax + 16}" text-anchor="middle" fill="#7a7e85">{t // 1000} ms</text>')

    for i, can_id in enumerate(ids):
        y = top + i * lane_h
        name = "tester" if can_id == min(ids) else "ECU"
        out.append(f'<text x="{L - 14}" y="{y + 30}" text-anchor="end" fill="#3d4148" font-weight="600">{name}</text>')
        out.append(f'<text x="{L - 14}" y="{y + 46}" text-anchor="end" fill="#7a7e85" font-family="monospace" font-size="11">0x{can_id:03X}</text>')
        out.append(f'<line x1="{L}" x2="{W - R}" y1="{y + 36}" y2="{y + 36}" stroke="#eceae6"/>')

    for t, can_id, data in frames:
        lane = ids.index(can_id)
        kind, color = KIND.get(data[0] >> 4, ("?", "#999"))
        x = L + (t - FRAME_US) * sx
        w = max(FRAME_US * sx, 3)
        y = top + lane * lane_h + 22
        out.append(f'<rect x="{x:.1f}" y="{y}" width="{w:.1f}" height="28" rx="3" fill="{color}"/>')
        if w > 16:
            label = kind if kind != "CF" else f"{data[0] & 0xF:X}"
            out.append(f'<text x="{x + w / 2:.1f}" y="{y + 18}" text-anchor="middle" fill="#fff" font-size="10.5" font-weight="600">{label}</text>')

    lx = L
    for kind, color in [KIND[1], KIND[3], KIND[2]]:
        out.append(f'<rect x="{lx}" y="{H - 22}" width="12" height="12" rx="2" fill="{color}"/>')
        text = {"FF": "First frame", "FC": "Flow control", "CF": "Consecutive frame (sequence number)"}[kind]
        out.append(f'<text x="{lx + 18}" y="{H - 12}" fill="#3d4148">{text}</text>')
        lx += 40 + len(text) * 6.4

    out.append("</svg>")
    open(dst, "w").write("\n".join(out) + "\n")


if __name__ == "__main__":
    main(*sys.argv[1:])
