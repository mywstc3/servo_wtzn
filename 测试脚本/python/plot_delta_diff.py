# -*- coding: utf-8 -*-
"""把 single_vs_multi_delta_diff.csv 画成对比图，便于直观查看。"""
from __future__ import annotations

import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

CLEAN = Path(r"e:\wtzn\wt_project\servo\测试数据\cleaned")
OUT = CLEAN / "figures"
DIFF_CSV = CLEAN / "single_vs_multi_delta_diff.csv"
SINGLE_CSV = CLEAN / "第二次单机测试数据--260720-115643_delta.csv"
MULTI_CSV = CLEAN / "第二次多机测试数据--260720-114529_delta.csv"


def setup_style() -> None:
    plt.rcParams.update(
        {
            "font.size": 11,
            "axes.grid": True,
            "grid.alpha": 0.35,
            "figure.facecolor": "white",
            "axes.facecolor": "#fafafa",
            "axes.edgecolor": "#333333",
            "axes.labelcolor": "#222222",
            "xtick.color": "#222222",
            "ytick.color": "#222222",
        }
    )
    # 尽量用系统字体显示中文；失败则英文标题仍可用
    for font in ("Microsoft YaHei", "SimHei", "Arial Unicode MS", "DejaVu Sans"):
        try:
            plt.rcParams["font.sans-serif"] = [font]
            plt.rcParams["axes.unicode_minus"] = False
            break
        except Exception:
            pass


def load_diff(path: Path):
    ids, sd, md, dd, same = [], [], [], [], []
    with path.open("r", encoding="utf-8-sig", newline="") as f:
        for r in csv.DictReader(f):
            ids.append(int(r["Id"]))
            sd.append(float(r["SingleDelta[ns]"]) / 1e3)
            md.append(float(r["MultiDelta[ns]"]) / 1e3)
            dd.append(float(r["DiffDelta[ns]"]) / 1e3)
            same.append(int(r["SameByte"]))
    return (
        np.array(ids),
        np.array(sd),
        np.array(md),
        np.array(dd),
        np.array(same),
    )


def load_delta(path: Path):
    rows = []
    with path.open("r", encoding="utf-8-sig", newline="") as f:
        for r in csv.DictReader(f):
            rows.append(
                {
                    "id": int(float(r["Id"])),
                    "dt_us": float(r["DeltaTime[ns]"]) / 1e3,
                    "hex": r["0:UART: RX/TX"].strip().upper(),
                }
            )
    return rows


def assemble_packets(rows):
    pkts = []
    i = 0
    n = len(rows)
    while i < n:
        if i + 3 >= n:
            break
        if rows[i]["hex"] != "FF" or rows[i + 1]["hex"] != "FF":
            i += 1
            continue
        length = int(rows[i + 3]["hex"], 16)
        last = i + 3 + length
        if last >= n:
            break
        kind = "resp" if length == 0x13 else "req"
        if length == 0x04:
            kind = "req"
        pkts.append({"start": i, "end": last, "kind": kind, "lead_us": rows[i]["dt_us"]})
        i = last + 1
    return pkts


def turnarounds(pkts):
    return [p["lead_us"] for p in pkts if p["kind"] == "resp"]


def save(fig, name: str) -> Path:
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / name
    fig.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {path}")
    return path


def plot_overview(ids, sd, md, dd):
    # 裁掉 |diff|>1ms 的假尖峰，另开一图看全量
    mask = np.abs(dd) <= 1000
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)

    axes[0].plot(ids[1:], sd[1:], color="#1f77b4", lw=0.8, label="Single")
    axes[0].plot(ids[1:], md[1:], color="#d62728", lw=0.8, alpha=0.85, label="Multi")
    axes[0].set_ylabel("Delta (us)")
    axes[0].set_title("Byte interval: Single vs Multi (full, log y)")
    axes[0].set_yscale("log")
    axes[0].legend(loc="upper right")

    axes[1].plot(ids[mask], dd[mask], color="#2ca02c", lw=0.7)
    axes[1].axhline(0, color="#444", lw=0.8)
    axes[1].set_ylabel("Diff (us)")
    axes[1].set_title("Diff = Multi - Single (|Diff|<=1000 us)")

    axes[2].plot(ids[1:], dd[1:], color="#9467bd", lw=0.7)
    axes[2].axhline(0, color="#444", lw=0.8)
    axes[2].set_ylabel("Diff (us)")
    axes[2].set_xlabel("Byte index (Id)")
    axes[2].set_title("Diff full range (includes ~50ms phase spikes)")

    return save(fig, "01_overview_delta_diff.png")


def plot_zoom_intra(ids, sd, md, dd):
    # 帧内字节：两边都 <20us
    mask = (sd < 20) & (md < 20) & (ids > 1)
    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    axes[0].plot(ids[mask], sd[mask], "o-", ms=2, lw=0.6, color="#1f77b4", label="Single")
    axes[0].plot(ids[mask], md[mask], "o-", ms=2, lw=0.6, color="#d62728", label="Multi")
    axes[0].set_ylabel("Delta (us)")
    axes[0].set_title("Intra-frame byte intervals (<20 us)")
    axes[0].legend()

    axes[1].plot(ids[mask], dd[mask], color="#2ca02c", lw=0.7)
    axes[1].axhline(0, color="#444", lw=0.8)
    axes[1].set_ylabel("Diff (us)")
    axes[1].set_xlabel("Byte index (Id)")
    axes[1].set_title("Diff on intra-frame bytes")
    return save(fig, "02_intra_byte_compare.png")


def plot_hist(dd):
    body = dd[1:]
    near = body[np.abs(body) <= 100]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))

    axes[0].hist(near, bins=41, color="#4c78a8", edgecolor="white")
    axes[0].set_title("Diff histogram (|Diff|<=100 us)")
    axes[0].set_xlabel("Diff = Multi-Single (us)")
    axes[0].set_ylabel("Count")

    axes[1].hist(body, bins=60, color="#f58518", edgecolor="white")
    axes[1].set_title("Diff histogram (full)")
    axes[1].set_xlabel("Diff (us)")
    axes[1].set_ylabel("Count")
    return save(fig, "03_diff_histogram.png")


def plot_scatter(sd, md):
    body_s, body_m = sd[1:], md[1:]
    fig, ax = plt.subplots(figsize=(6.5, 6))
    ax.scatter(body_s, body_m, s=10, alpha=0.45, c="#1f77b4", edgecolors="none")
    lim = max(body_s.max(), body_m.max()) * 1.05
    ax.plot([0, lim], [0, lim], "--", color="#d62728", lw=1, label="y=x")
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("Single Delta (us)")
    ax.set_ylabel("Multi Delta (us)")
    ax.set_title("Single vs Multi interval (log-log)")
    ax.legend()
    return save(fig, "04_scatter_single_vs_multi.png")


def plot_turnaround():
    sp = assemble_packets(load_delta(SINGLE_CSV))
    mp = assemble_packets(load_delta(MULTI_CSV))
    st = np.array(turnarounds(sp))
    mt = np.array(turnarounds(mp))
    n = min(len(st), len(mt))
    x = np.arange(1, n + 1)
    st, mt = st[:n], mt[:n]
    diff = mt - st

    fig, axes = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)
    axes[0].plot(x, st, "o-", color="#1f77b4", label="Single")
    axes[0].plot(x, mt, "s-", color="#d62728", label="Multi")
    axes[0].set_ylabel("Turnaround (us)")
    axes[0].set_title("TX->RX turnaround (gap before response FF FF)")
    axes[0].legend()

    axes[1].bar(x, diff, color=np.where(diff >= 0, "#d62728", "#2ca02c"), width=0.7)
    axes[1].axhline(0, color="#444", lw=0.8)
    axes[1].set_xlabel("Response index")
    axes[1].set_ylabel("Diff (us)")
    axes[1].set_title("Turnaround Diff = Multi - Single")
    return save(fig, "05_turnaround_compare.png")


def plot_paired_bars():
    """Turnaround + summary stats as bar chart."""
    sp = assemble_packets(load_delta(SINGLE_CSV))
    mp = assemble_packets(load_delta(MULTI_CSV))
    st = np.array(turnarounds(sp))
    mt = np.array(turnarounds(mp))
    n = min(len(st), len(mt))
    st, mt = st[:n], mt[:n]

    labels = ["min", "p50", "mean", "p95", "max"]

    def stats(v):
        s = np.sort(v)
        return [
            s.min(),
            np.median(s),
            s.mean(),
            s[int(0.95 * (len(s) - 1))],
            s.max(),
        ]

    ss, ms = stats(st), stats(mt)
    x = np.arange(len(labels))
    w = 0.35
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.bar(x - w / 2, ss, w, label="Single", color="#1f77b4")
    ax.bar(x + w / 2, ms, w, label="Multi", color="#d62728")
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("us")
    ax.set_title("Turnaround summary: Single vs Multi")
    ax.legend()
    for i, (a, b) in enumerate(zip(ss, ms)):
        ax.text(i - w / 2, a + 0.5, f"{a:.1f}", ha="center", va="bottom", fontsize=8)
        ax.text(i + w / 2, b + 0.5, f"{b:.1f}", ha="center", va="bottom", fontsize=8)
    return save(fig, "06_turnaround_summary_bars.png")


def main():
    setup_style()
    ids, sd, md, dd, _same = load_diff(DIFF_CSV)
    paths = [
        plot_overview(ids, sd, md, dd),
        plot_zoom_intra(ids, sd, md, dd),
        plot_hist(dd),
        plot_scatter(sd, md),
        plot_turnaround(),
        plot_paired_bars(),
    ]
    print(f"\nDone. {len(paths)} figures in {OUT}")


if __name__ == "__main__":
    main()
