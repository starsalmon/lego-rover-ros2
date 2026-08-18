#!/usr/bin/env python3
"""Fallback: extract escape segments from stream.jsonl when escape_events.jsonl is empty."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def load_rows(path: Path) -> list[dict]:
    rows: list[dict] = []
    if not path.is_file():
        return rows
    for line in path.read_bytes().split(b'\n'):
        line = line.strip()
        if not line or b'\x00' in line:
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return rows


def analyze_stream(rows: list[dict], center: float = 90.0) -> list[dict]:
    segments: list[dict] = []
    in_esc = False
    seg: dict | None = None
    for r in rows:
        if r.get('kind') == 'avoid':
            if r.get('active') and not in_esc:
                in_esc = True
                seg = {'bins': [], 'angs': []}
            elif not r.get('active') and in_esc:
                in_esc = False
                if seg:
                    segments.append(seg)
                    seg = None
        elif in_esc and seg is not None:
            if r.get('kind') == 'range' and float(r.get('range_m', -1)) > 0:
                seg['bins'].append((float(r['pan_deg']), float(r['range_m'])))
            if r.get('kind') == 'cmd':
                seg['angs'].append(float(r.get('ang', 0)))
    out: list[dict] = []
    for i, s in enumerate(segments, 1):
        if not s['bins']:
            continue
        best = max(s['bins'], key=lambda x: x[1])
        worst = min(s['bins'], key=lambda x: x[1])
        angs = s['angs'][-20:]
        mean_ang = sum(angs) / len(angs) if angs else 0.0
        turn = 'right' if mean_ang > 0.03 else 'left' if mean_ang < -0.03 else 'straight'
        want = (
            'left'
            if best[0] > center + 5
            else 'right'
            if best[0] < center - 5
            else 'straight'
        )
        if want == 'straight' or turn == 'straight':
            verdict = 'unknown'
        elif want == turn:
            verdict = 'ok'
        else:
            verdict = 'WRONG'
        out.append(
            {
                'segment': i,
                'best_pan_deg': best[0],
                'best_m': best[1],
                'worst_pan_deg': worst[0],
                'worst_m': worst[1],
                'cmd_turn': turn,
                'want_turn': want,
                'verdict': verdict,
                'n_bins': len(s['bins']),
            }
        )
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('dir', type=Path)
    args = ap.parse_args()
    stream = args.dir / 'stream.jsonl'
    rows = load_rows(stream)
    if not rows:
        print(f'No stream data in {stream}')
        return 1
    segs = analyze_stream(rows)
    print(f'=== Stream analysis: {args.dir} ===')
    print(f'samples={len(rows)} escape_segments={len(segs)}')
    wrong = sum(1 for s in segs if s['verdict'] == 'WRONG')
    ok = sum(1 for s in segs if s['verdict'] == 'ok')
    print(f'verdict: ok={ok} WRONG={wrong} unknown={len(segs)-ok-wrong}')
    print()
    for s in segs:
        print(
            f"#{s['segment']} bins={s['n_bins']} "
            f"best={s['best_pan_deg']:.0f}° {s['best_m']:.2f}m "
            f"worst={s['worst_pan_deg']:.0f}° {s['worst_m']:.2f}m "
            f"cmd={s['cmd_turn']} want={s['want_turn']} → {s['verdict']}"
        )
    return 2 if wrong else 0


if __name__ == '__main__':
    raise SystemExit(main())
