# -*- coding: utf-8 -*-
"""逐字节节拍分析：区分主机请求 vs 舵机应答内部间隔。"""
from __future__ import annotations

import csv
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from parse_uart_timing import (  # noqa: E402
    parse_decoder_csv,
    assemble_sts_packets,
    classify_packet,
)


def _configure_stdio() -> None:
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")
        except Exception:
            pass


def pct(sorted_vals, p):
    if not sorted_vals:
        return float("nan")
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    idx = int(round((p / 100.0) * (len(sorted_vals) - 1)))
    return sorted_vals[idx]


def summarize_us(name, samples_us):
    if not samples_us:
        print(f"[{name}] 无样本")
        return
    s = sorted(samples_us)
    print(f"\n===== {name} (us) n={len(s)} =====")
    print(
        f"  min={min(s):.2f}  p50={statistics.median(s):.2f}  "
        f"mean={statistics.mean(s):.2f}  p95={pct(s,95):.2f}  "
        f"p99={pct(s,99):.2f}  max={max(s):.2f}"
    )
    # 桶分布
    buckets = [
        (0, 12, "≤12 (1Mbps正常~10-11)"),
        (12, 20, "12~20"),
        (20, 50, "20~50"),
        (50, 100, "50~100"),
        (100, 500, "100~500"),
        (500, 1e12, ">500"),
    ]
    for lo, hi, label in buckets:
        c = sum(1 for x in s if lo < x <= hi) if lo > 0 else sum(1 for x in s if x <= hi)
        if lo == 0:
            c = sum(1 for x in s if x <= hi)
        else:
            c = sum(1 for x in s if lo < x <= hi)
        if c:
            print(f"  {label:28s}: {c:4d}  ({100*c/len(s):5.1f}%)")


def frame_gaps(bytes_all, pkt):
    bs = [b for b in bytes_all if pkt.start_ns <= b.time_ns <= pkt.end_ns]
    gaps = [(bs[i].time_ns - bs[i - 1].time_ns) / 1e3 for i in range(1, len(bs))]
    return bs, gaps


def analyze(path: Path):
    bytes_ = parse_decoder_csv(path)
    packets, orphans = assemble_sts_packets(bytes_)
    reqs = [p for p in packets if classify_packet(p) == "req"]
    resps = [p for p in packets if classify_packet(p) == "resp"]

    print(f"文件: {path.name}")
    print(f"包: req={len(reqs)} resp={len(resps)} orphan={len(orphans)}")

    req_intra = []
    resp_intra = []
    turnaround = []  # req end -> resp start
    req_cs_gap = []  # 倒数第二字节 -> CS（主机常卡这里）
    resp_byte_jitter = []

    worst_resp = []

    for r in reqs:
        _, gaps = frame_gaps(bytes_, r)
        req_intra.extend(gaps)
        if gaps:
            req_cs_gap.append(gaps[-1])  # 最后间隔通常是 CS 前

    for i, p in enumerate(resps):
        bs, gaps = frame_gaps(bytes_, p)
        resp_intra.extend(gaps)
        if gaps:
            med = statistics.median(gaps)
            mx = max(gaps)
            resp_byte_jitter.append(mx - min(gaps))
            worst_resp.append((mx, med, p.start_ns / 1e6, p.hex_str()[:40], gaps))

    # 配对 turnaround
    for r in reqs:
        after = [p for p in resps if p.start_ns > r.end_ns]
        if not after:
            continue
        resp = min(after, key=lambda p: p.start_ns)
        # 若中间还有别的 req 则不算
        next_req = [x for x in reqs if x.start_ns > r.start_ns]
        if next_req and resp.start_ns > next_req[0].start_ns:
            continue
        turnaround.append((resp.start_ns - r.end_ns) / 1e3)

    summarize_us("主机请求帧内字节间隔", req_intra)
    summarize_us("主机请求末字节前间隔(常为CS前)", req_cs_gap)
    summarize_us("舵机应答帧内字节间隔", resp_intra)
    summarize_us("请求结束→应答开始(turnaround)", turnaround)
    summarize_us("应答帧内(max-min)抖动", resp_byte_jitter)

    worst_resp.sort(key=lambda x: -x[0])
    print("\n===== 应答帧内最大字节间隔 Top8 =====")
    for mx, med, t, hx, gaps in worst_resp[:8]:
        # 标出哪个位置间隔最大
        j = gaps.index(mx)
        print(
            f"  t={t:8.3f}ms  max_gap={mx:7.2f}us @byte[{j}->{j+1}]  "
            f"med={med:.2f}us  {hx}..."
        )
        # 打印该帧全部间隔
        print(f"    gaps: {' '.join(f'{g:.1f}' for g in gaps)}")

    print("\n===== 主机请求末间隔 Top8（CS前卡顿）=====")
    scored = []
    for r in reqs:
        _, gaps = frame_gaps(bytes_, r)
        if gaps:
            scored.append((gaps[-1], r.start_ns / 1e6, r.hex_str(), gaps))
    scored.sort(key=lambda x: -x[0])
    for g, t, hx, gaps in scored[:8]:
        print(f"  t={t:8.3f}ms  CS_gap={g:7.2f}us  gaps=[{' '.join(f'{x:.1f}' for x in gaps)}]  {hx}")

    # 理论 1Mbps 8N1 ≈ 10 bit * 1us = 10us/byte；容差
    print("\n===== 相对 1Mbps 理想节拍(10us) =====")
    for name, samples in (
        ("请求", req_intra),
        ("应答", resp_intra),
    ):
        if not samples:
            continue
        over15 = sum(1 for x in samples if x > 15)
        over30 = sum(1 for x in samples if x > 30)
        print(
            f"  {name}: >15us {over15}/{len(samples)} ({100*over15/len(samples):.1f}%), "
            f">30us {over30}/{len(samples)} ({100*over30/len(samples):.1f}%)"
        )


def main():
    _configure_stdio()
    paths = sys.argv[1:] or [
        r"e:\wtzn\wt_project\servo\测试数据\decoder--260715-213159.txt",
    ]
    for i, p in enumerate(paths):
        if i:
            print("\n" + "=" * 72)
        analyze(Path(p))


if __name__ == "__main__":
    main()
