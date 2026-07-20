# -*- coding: utf-8 -*-
"""对比：自研单机 vs 原厂单机（已清洗的 delta CSV）"""
from __future__ import annotations

import csv
import statistics
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

CLEAN = Path(r"e:\wtzn\wt_project\servo\测试数据\cleaned")
OURS = CLEAN / "第二次单机测试数据--260720-115643_delta.csv"
OEM = CLEAN / "单机原厂测试数据--260720-195540_delta.csv"
OUT = CLEAN / "figures_vs_oem"
DIFF_OUT = CLEAN / "ours_vs_oem_delta_diff.csv"


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


def assemble(rows):
    pkts = []
    i, n = 0, len(rows)
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
        pkts.append(
            {
                "start": i,
                "end": last,
                "kind": kind,
                "len": length,
                "id_byte": int(rows[i + 2]["hex"], 16),
                "lead_us": rows[i]["dt_us"],
                "hex": [rows[j]["hex"] for j in range(i, last + 1)],
                "intra": [rows[j]["dt_us"] for j in range(i + 1, last + 1)],
            }
        )
        i = last + 1
    return pkts


def pct(s, p):
    s = sorted(s)
    return s[int(round(p / 100 * (len(s) - 1)))]


def summ(name, v):
    s = sorted(v)
    print(
        f"  {name}: n={len(s)} min={min(s):.2f} p50={statistics.median(s):.2f} "
        f"mean={statistics.mean(s):.2f} p95={pct(s,95):.2f} max={max(s):.2f}"
    )


def main():
    ours = load_delta(OURS)
    oem = load_delta(OEM)
    print(f"ours bytes={len(ours)}  oem bytes={len(oem)}")

    n = min(len(ours), len(oem))
    # write aligned diff: Diff = OEM - OURS  (正=原厂该位间隔更长)
    with DIFF_OUT.open("w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "Id",
                "OursHex",
                "OemHex",
                "OursDelta[ns]",
                "OemDelta[ns]",
                "DiffDelta[ns]",
                "SameByte",
            ]
        )
        for i in range(n):
            od = ours[i]["dt_us"] * 1e3
            ed = oem[i]["dt_us"] * 1e3
            w.writerow(
                [
                    i + 1,
                    ours[i]["hex"],
                    oem[i]["hex"],
                    f"{od:.2f}",
                    f"{ed:.2f}",
                    f"{ed - od:.2f}",
                    int(ours[i]["hex"] == oem[i]["hex"]),
                ]
            )
    print(f"wrote {DIFF_OUT}")

    same = sum(1 for i in range(n) if ours[i]["hex"] == oem[i]["hex"])
    print(f"aligned={n} same_byte={same}/{n} ({100*same/n:.1f}%)")

    dd = np.array([oem[i]["dt_us"] - ours[i]["dt_us"] for i in range(1, n)])
    ou = np.array([ours[i]["dt_us"] for i in range(1, n)])
    oe = np.array([oem[i]["dt_us"] for i in range(1, n)])

    print("\n===== Diff = OEM - Ours (us) =====")
    summ("Diff", dd.tolist())
    print(f"  |Diff|<=5us: {np.mean(np.abs(dd)<=5)*100:.1f}%")
    print(f"  |Diff|>100us: {np.mean(np.abs(dd)>100)*100:.1f}%")
    print(f"  total span ours: {sum(r['dt_us'] for r in ours)/1e3:.3f} ms")
    print(f"  total span oem:  {sum(r['dt_us'] for r in oem)/1e3:.3f} ms")

    # class by ours interval
    for lab, lo, hi in [
        ("intra<=20", 0, 20),
        ("short 20-100", 20, 100),
        ("mid 100-1000", 100, 1000),
        ("long>1000", 1000, 1e12),
    ]:
        m = (ou > lo) & (ou <= hi) if lo > 0 else (ou <= hi)
        if m.any():
            summ(f"Diff when ours {lab}", dd[m].tolist())

    op = assemble(ours)
    ep = assemble(oem)
    print(f"\npackets ours={len(op)} oem={len(ep)}")
    print(
        f"  ours req/resp: {sum(1 for p in op if p['kind']=='req')}/"
        f"{sum(1 for p in op if p['kind']=='resp')}"
    )
    print(
        f"  oem  req/resp: {sum(1 for p in ep if p['kind']=='req')}/"
        f"{sum(1 for p in ep if p['kind']=='resp')}"
    )

    # show first frame hex both
    print("\n===== First REQ frame =====")
    print("ours:", " ".join(op[0]["hex"]))
    print("oem: ", " ".join(ep[0]["hex"]))
    print("===== First RESP frame (head 12) =====")
    print("ours:", " ".join(op[1]["hex"][:12]), "...")
    print("oem: ", " ".join(ep[1]["hex"][:12]), "...")

    # servo id distribution
    from collections import Counter

    print("ours IDs:", Counter(p["id_byte"] for p in op))
    print("oem  IDs:", Counter(p["id_byte"] for p in ep))

    # turnaround
    ot = [p["lead_us"] for p in op if p["kind"] == "resp"]
    et = [p["lead_us"] for p in ep if p["kind"] == "resp"]
    print("\n===== TX->RX turnaround (us) =====")
    summ("ours", ot)
    summ("oem ", et)
    k = min(len(ot), len(et))
    td = [et[i] - ot[i] for i in range(k)]
    summ("oem-ours", td)

    # frame duration
    def frame_dur(p):
        return sum(p["intra"])  # from 2nd byte; lead is inter-frame

    print("\n===== Frame duration (intra only, us) =====")
    for kind in ("req", "resp"):
        od = [frame_dur(p) for p in op if p["kind"] == kind]
        ed = [frame_dur(p) for p in ep if p["kind"] == kind]
        summ(f"ours {kind}", od)
        summ(f"oem  {kind}", ed)

    # host poll gap before req (exclude first)
    oh = [p["lead_us"] for p in op if p["kind"] == "req" and p["start"] > 0]
    eh = [p["lead_us"] for p in ep if p["kind"] == "req" and p["start"] > 0]
    print("\n===== Gap before next REQ =====")
    summ("ours all", oh)
    summ("oem  all", eh)
    summ("ours long(>=1ms)", [x for x in oh if x >= 1000] or [0])
    summ("oem  long(>=1ms)", [x for x in eh if x >= 1000] or [0])
    summ("ours short(<1ms)", [x for x in oh if x < 1000] or [0])
    summ("oem  short(<1ms)", [x for x in eh if x < 1000] or [0])

    # plots
    OUT.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({"font.size": 11, "axes.grid": True, "grid.alpha": 0.35})
    for font in ("Microsoft YaHei", "SimHei", "DejaVu Sans"):
        try:
            plt.rcParams["font.sans-serif"] = [font]
            plt.rcParams["axes.unicode_minus"] = False
            break
        except Exception:
            pass

    ids = np.arange(1, n + 1)
    ours_d = np.array([r["dt_us"] for r in ours[:n]])
    oem_d = np.array([r["dt_us"] for r in oem[:n]])
    diff = oem_d - ours_d

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    axes[0].semilogy(ids[1:], ours_d[1:], color="#1f77b4", lw=0.8, label="Ours")
    axes[0].semilogy(ids[1:], oem_d[1:], color="#d62728", lw=0.8, alpha=0.85, label="OEM")
    axes[0].set_ylabel("Delta (us)")
    axes[0].set_title("Byte interval: Ours vs OEM (log y)")
    axes[0].legend()

    m = np.abs(diff) <= 1000
    axes[1].plot(ids[m], diff[m], color="#2ca02c", lw=0.7)
    axes[1].axhline(0, color="#444", lw=0.8)
    axes[1].set_ylabel("Diff (us)")
    axes[1].set_title("Diff = OEM - Ours (|Diff|<=1000 us)")

    axes[2].plot(ids[1:], diff[1:], color="#9467bd", lw=0.7)
    axes[2].axhline(0, color="#444", lw=0.8)
    axes[2].set_ylabel("Diff (us)")
    axes[2].set_xlabel("Byte index")
    axes[2].set_title("Diff full range")
    fig.tight_layout()
    fig.savefig(OUT / "01_overview.png", dpi=150)
    plt.close(fig)

    # turnaround
    fig, axes = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)
    x = np.arange(1, k + 1)
    axes[0].plot(x, ot[:k], "o-", color="#1f77b4", label="Ours")
    axes[0].plot(x, et[:k], "s-", color="#d62728", label="OEM")
    axes[0].set_ylabel("Turnaround (us)")
    axes[0].set_title("TX->RX turnaround: Ours vs OEM")
    axes[0].legend()
    d = np.array(td)
    colors = np.where(d >= 0, "#d62728", "#2ca02c")
    axes[1].bar(x, d, color=colors, width=0.7)
    axes[1].axhline(0, color="#444", lw=0.8)
    axes[1].set_xlabel("Response index")
    axes[1].set_ylabel("Diff (us)")
    axes[1].set_title("Turnaround Diff = OEM - Ours")
    fig.tight_layout()
    fig.savefig(OUT / "02_turnaround.png", dpi=150)
    plt.close(fig)

    # summary bars turnaround
    def stats(v):
        s = np.sort(v)
        return [s.min(), np.median(s), s.mean(), s[int(0.95 * (len(s) - 1))], s.max()]

    labels = ["min", "p50", "mean", "p95", "max"]
    ss, es = stats(np.array(ot)), stats(np.array(et))
    fig, ax = plt.subplots(figsize=(8, 4.5))
    xx = np.arange(len(labels))
    w = 0.35
    ax.bar(xx - w / 2, ss, w, label="Ours", color="#1f77b4")
    ax.bar(xx + w / 2, es, w, label="OEM", color="#d62728")
    ax.set_xticks(xx)
    ax.set_xticklabels(labels)
    ax.set_ylabel("us")
    ax.set_title("Turnaround summary: Ours vs OEM")
    ax.legend()
    for i, (a, b) in enumerate(zip(ss, es)):
        ax.text(i - w / 2, a + 0.5, f"{a:.1f}", ha="center", va="bottom", fontsize=8)
        ax.text(i + w / 2, b + 0.5, f"{b:.1f}", ha="center", va="bottom", fontsize=8)
    fig.tight_layout()
    fig.savefig(OUT / "03_turnaround_bars.png", dpi=150)
    plt.close(fig)

    # hist
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    near = dd[np.abs(dd) <= 100]
    axes[0].hist(near, bins=41, color="#4c78a8", edgecolor="white")
    axes[0].set_title("Diff hist (|Diff|<=100 us)")
    axes[0].set_xlabel("OEM - Ours (us)")
    axes[1].hist(dd, bins=60, color="#f58518", edgecolor="white")
    axes[1].set_title("Diff hist (full)")
    axes[1].set_xlabel("OEM - Ours (us)")
    fig.tight.layout() if False else fig.tight_layout()
    fig.savefig(OUT / "04_diff_hist.png", dpi=150)
    plt.close(fig)

    # intra compare
    mask = (ours_d < 20) & (oem_d < 20) & (ids > 1)
    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    axes[0].plot(ids[mask], ours_d[mask], "o-", ms=2, lw=0.6, color="#1f77b4", label="Ours")
    axes[0].plot(ids[mask], oem_d[mask], "o-", ms=2, lw=0.6, color="#d62728", label="OEM")
    axes[0].set_ylabel("Delta (us)")
    axes[0].set_title("Intra-frame byte intervals (<20 us)")
    axes[0].legend()
    axes[1].plot(ids[mask], diff[mask], color="#2ca02c", lw=0.7)
    axes[1].axhline(0, color="#444", lw=0.8)
    axes[1].set_xlabel("Byte index")
    axes[1].set_ylabel("Diff (us)")
    fig.tight_layout()
    fig.savefig(OUT / "05_intra.png", dpi=150)
    plt.close(fig)

    print(f"\nfigures -> {OUT}")


if __name__ == "__main__":
    main()
