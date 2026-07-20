#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
解析逻辑分析仪 UART decoder 导出文本，统计：
  1) 相邻字节时间间隔
  2) 完整 STS 包之间的时间间隔

输入格式示例（CSV）：
  Id,Time[ns],0:UART: RX/TX
  1,26294400.00,FF
  2,26305450.00,FF
  ...

STS 帧：FF FF | ID | LEN | body(LEN 字节，含校验)
"""

from __future__ import annotations

import argparse
import csv
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple


# ---------------------------------------------------------------------------
# 数据结构
# ---------------------------------------------------------------------------

@dataclass
class UartByte:
    row_id: int
    time_ns: float
    value: int


@dataclass
class StsPacket:
    index: int
    start_ns: float
    end_ns: float
    bytes_: List[int]
    checksum_ok: bool
    note: str = ""

    @property
    def duration_ns(self) -> float:
        return self.end_ns - self.start_ns

    @property
    def servo_id(self) -> int:
        return self.bytes_[2] if len(self.bytes_) >= 3 else -1

    @property
    def length(self) -> int:
        return self.bytes_[3] if len(self.bytes_) >= 4 else -1

    @property
    def instruction_or_error(self) -> int:
        return self.bytes_[4] if len(self.bytes_) >= 5 else -1

    def hex_str(self) -> str:
        return " ".join(f"{b:02X}" for b in self.bytes_)


# ---------------------------------------------------------------------------
# 解析
# ---------------------------------------------------------------------------

def parse_decoder_csv(path: Path) -> List[UartByte]:
    """读取 decoder CSV，返回按时间排序的字节列表。"""
    rows: List[UartByte] = []
    with path.open("r", encoding="utf-8-sig", newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            return rows

        for raw in reader:
            if not raw or len(raw) < 3:
                continue
            try:
                row_id = int(float(raw[0]))
                time_ns = float(raw[1])
                value = int(raw[2].strip(), 16)
            except (ValueError, IndexError):
                continue
            rows.append(UartByte(row_id=row_id, time_ns=time_ns, value=value))

    rows.sort(key=lambda b: b.time_ns)
    return rows


def sts_checksum(frame_without_cs: List[int]) -> int:
    """Checksum = ~(ID + LEN + Inst/Err + Params...) 低 8 位。"""
    return (~(sum(frame_without_cs) & 0xFF)) & 0xFF


def assemble_sts_packets(
    bytes_: List[UartByte],
) -> Tuple[List[StsPacket], List[UartByte]]:
    """
    从字节流组 STS 帧。
    规则：找到 FF FF 后读 ID、LEN，再取 LEN 字节 body（含 CS）。
    无法成帧的字节归入 orphans。
    """
    packets: List[StsPacket] = []
    orphans: List[UartByte] = []
    i = 0
    n = len(bytes_)

    while i < n:
        # 需要至少 FF FF ID LEN
        if i + 3 >= n:
            orphans.extend(bytes_[i:])
            break

        if bytes_[i].value != 0xFF or bytes_[i + 1].value != 0xFF:
            orphans.append(bytes_[i])
            i += 1
            continue

        length = bytes_[i + 3].value
        total = 4 + length  # FF FF ID LEN + LEN 字节
        if i + total > n:
            orphans.extend(bytes_[i:])
            break

        frame_bytes = [b.value for b in bytes_[i : i + total]]
        body_no_cs = frame_bytes[2:-1]  # ID..最后一字节前
        cs_calc = sts_checksum(body_no_cs)
        cs_ok = frame_bytes[-1] == cs_calc
        note = "" if cs_ok else f"CS mismatch expect={cs_calc:02X}"

        packets.append(
            StsPacket(
                index=len(packets),
                start_ns=bytes_[i].time_ns,
                end_ns=bytes_[i + total - 1].time_ns,
                bytes_=frame_bytes,
                checksum_ok=cs_ok,
                note=note,
            )
        )
        i += total

    return packets, orphans


# ---------------------------------------------------------------------------
# 统计
# ---------------------------------------------------------------------------

def intervals(values: List[float]) -> List[float]:
    return [values[i] - values[i - 1] for i in range(1, len(values))]


def summarize(name: str, samples_ns: List[float], file=sys.stdout) -> None:
    if not samples_ns:
        print(f"[{name}] 无样本", file=file)
        return

    us = [x / 1e3 for x in samples_ns]
    ms = [x / 1e6 for x in samples_ns]

    def pct(p: float) -> float:
        if len(ms) == 1:
            return ms[0]
        k = sorted(ms)
        idx = int(round((p / 100.0) * (len(k) - 1)))
        return k[idx]

    print(f"\n===== {name} =====", file=file)
    print(f"  样本数: {len(samples_ns)}", file=file)
    print(f"  最小:   {min(ms):.6f} ms  ({min(us):.3f} us)", file=file)
    print(f"  最大:   {max(ms):.6f} ms  ({max(us):.3f} us)", file=file)
    print(f"  平均:   {statistics.mean(ms):.6f} ms  ({statistics.mean(us):.3f} us)", file=file)
    if len(ms) >= 2:
        print(f"  中位:   {statistics.median(ms):.6f} ms", file=file)
        print(f"  标准差: {statistics.stdev(ms):.6f} ms", file=file)
        print(f"  P95:    {pct(95):.6f} ms", file=file)
        print(f"  P99:    {pct(99):.6f} ms", file=file)


def classify_packet(pkt: StsPacket) -> str:
    """粗分请求/响应（LEN=4 且 Inst=02 等更像请求；状态字节常见响应）。"""
    if len(pkt.bytes_) < 5:
        return "short"
    inst = pkt.bytes_[4]
    # 主机常用指令
    if inst in (0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x0A, 0x0B):
        # 响应 PING 也是 LEN=2, Err=0，与请求 PING(Inst=1) 同形态需看上下文
        if pkt.length == 2 and inst == 0x00:
            return "resp"
        if pkt.length == 2 and inst == 0x01:
            return "req"  # PING
        if pkt.length >= 3 and inst in (0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x0A, 0x0B):
            return "req"
    # 典型响应：Err=0/1/...，后跟数据
    return "resp"


# ---------------------------------------------------------------------------
# 输出
# ---------------------------------------------------------------------------

def write_byte_intervals_csv(
    path: Path, bytes_: List[UartByte], delta_ns: List[float]
) -> None:
    with path.open("w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "from_id",
                "to_id",
                "from_time_ns",
                "to_time_ns",
                "delta_ns",
                "delta_us",
                "delta_ms",
                "from_byte",
                "to_byte",
            ]
        )
        for i, d in enumerate(delta_ns):
            a, b = bytes_[i], bytes_[i + 1]
            w.writerow(
                [
                    a.row_id,
                    b.row_id,
                    f"{a.time_ns:.2f}",
                    f"{b.time_ns:.2f}",
                    f"{d:.2f}",
                    f"{d / 1e3:.6f}",
                    f"{d / 1e6:.9f}",
                    f"{a.value:02X}",
                    f"{b.value:02X}",
                ]
            )


def write_packet_report_csv(path: Path, packets: List[StsPacket]) -> None:
    with path.open("w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "pkt_idx",
                "kind",
                "start_ns",
                "end_ns",
                "duration_us",
                "gap_from_prev_start_ns",
                "gap_from_prev_start_ms",
                "gap_from_prev_end_ns",
                "gap_from_prev_end_ms",
                "id",
                "len",
                "inst_or_err",
                "checksum_ok",
                "hex",
                "note",
            ]
        )
        prev: Optional[StsPacket] = None
        for pkt in packets:
            gap_start = (pkt.start_ns - prev.start_ns) if prev else ""
            gap_end = (pkt.start_ns - prev.end_ns) if prev else ""
            w.writerow(
                [
                    pkt.index,
                    classify_packet(pkt),
                    f"{pkt.start_ns:.2f}",
                    f"{pkt.end_ns:.2f}",
                    f"{pkt.duration_ns / 1e3:.3f}",
                    f"{gap_start:.2f}" if gap_start != "" else "",
                    f"{float(gap_start) / 1e6:.6f}" if gap_start != "" else "",
                    f"{gap_end:.2f}" if gap_end != "" else "",
                    f"{float(gap_end) / 1e6:.6f}" if gap_end != "" else "",
                    f"{pkt.servo_id:02X}",
                    f"{pkt.length:02X}",
                    f"{pkt.instruction_or_error:02X}",
                    "1" if pkt.checksum_ok else "0",
                    pkt.hex_str(),
                    pkt.note,
                ]
            )
            prev = pkt


def print_packet_list(
    packets: List[StsPacket], limit: int = 30, file=sys.stdout
) -> None:
    print(f"\n===== 完整包列表（最多显示 {limit} 条，共 {len(packets)}） =====", file=file)
    prev: Optional[StsPacket] = None
    for pkt in packets[:limit]:
        gap_s = ""
        gap_e = ""
        if prev is not None:
            gap_s = f"  Δstart={(pkt.start_ns - prev.start_ns) / 1e6:.3f} ms"
            gap_e = f"  Δfrom_end={(pkt.start_ns - prev.end_ns) / 1e6:.3f} ms"
        cs = "OK" if pkt.checksum_ok else "BAD"
        print(
            f"  [{pkt.index:03d}] {classify_packet(pkt):4s} "
            f"t={pkt.start_ns / 1e6:.3f} ms  "
            f"dur={pkt.duration_ns / 1e3:.1f} us  "
            f"CS={cs}{gap_s}{gap_e}",
            file=file,
        )
        print(f"         {pkt.hex_str()}", file=file)
        if pkt.note:
            print(f"         !! {pkt.note}", file=file)
        prev = pkt
    if len(packets) > limit:
        print(f"  ... 其余 {len(packets) - limit} 包见 CSV", file=file)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def default_data_path() -> Path:
    here = Path(__file__).resolve().parent
    return here.parent / "测试数据" / "decoder--260715-202804.txt"


def _configure_stdio() -> None:
    """尽量让 Windows 控制台正确显示中文。"""
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")
        except Exception:
            pass


def main() -> int:
    _configure_stdio()
    parser = argparse.ArgumentParser(
        description="解析 UART decoder 文本，计算字节间隔与 STS 完整包间隔"
    )
    parser.add_argument(
        "input",
        nargs="?",
        default=str(default_data_path()),
        help="decoder CSV 路径（默认：../测试数据/decoder--260715-202804.txt）",
    )
    parser.add_argument(
        "-o",
        "--outdir",
        default="",
        help="输出目录（默认：输入文件同目录下的 timing_out）",
    )
    parser.add_argument(
        "--list-limit",
        type=int,
        default=40,
        help="控制台打印完整包条数上限",
    )
    parser.add_argument(
        "--burst-gap-us",
        type=float,
        default=50.0,
        help="相邻字节间隔超过该值(us)时视为新「发送突发」边界，默认 50us",
    )
    args = parser.parse_args()

    in_path = Path(args.input)
    if not in_path.is_file():
        print(f"找不到输入文件: {in_path}", file=sys.stderr)
        return 1

    outdir = Path(args.outdir) if args.outdir else (in_path.parent / "timing_out")
    outdir.mkdir(parents=True, exist_ok=True)

    bytes_ = parse_decoder_csv(in_path)
    if len(bytes_) < 2:
        print("有效字节不足，无法分析", file=sys.stderr)
        return 1

    # --- 字节间隔 ---
    byte_deltas = intervals([b.time_ns for b in bytes_])
    summarize("相邻字节时间间隔", byte_deltas)

    # --- 发送突发（字节间隔过大则切断）---
    burst_starts = [bytes_[0].time_ns]
    gap_threshold_ns = args.burst_gap_us * 1e3
    for i, d in enumerate(byte_deltas):
        if d >= gap_threshold_ns:
            burst_starts.append(bytes_[i + 1].time_ns)
    burst_gaps = intervals(burst_starts)
    summarize(
        f"发送突发间隔（字节间隙 ≥ {args.burst_gap_us:g} us 切断）",
        burst_gaps,
    )

    # --- 完整 STS 包 ---
    packets, orphans = assemble_sts_packets(bytes_)
    print(f"\n字节总数: {len(bytes_)}")
    print(f"完整 STS 包: {len(packets)}")
    print(f"未组帧孤儿字节: {len(orphans)}")
    if orphans:
        sample = ", ".join(
            f"{b.value:02X}@{b.time_ns / 1e6:.3f}ms" for b in orphans[:8]
        )
        more = " ..." if len(orphans) > 8 else ""
        print(f"  孤儿示例: {sample}{more}")

    bad_cs = sum(1 for p in packets if not p.checksum_ok)
    if bad_cs:
        print(f"  校验失败包: {bad_cs}")

    # 包间隔：起点→起点 / 上一包结束→本包起点
    pkt_start_gaps = intervals([p.start_ns for p in packets])
    pkt_idle_gaps = [
        packets[i].start_ns - packets[i - 1].end_ns for i in range(1, len(packets))
    ]
    summarize("完整包间隔（起点 → 起点）", pkt_start_gaps)
    summarize("完整包空闲间隔（上一包结束 → 本包起点）", pkt_idle_gaps)

    # 区分：请求→响应（短间隙）vs 响应→下一请求（长轮询）
    req_to_resp: List[float] = []
    resp_to_req: List[float] = []
    for i in range(1, len(packets)):
        prev, cur = packets[i - 1], packets[i]
        gap = cur.start_ns - prev.end_ns
        pk, ck = classify_packet(prev), classify_packet(cur)
        if pk == "req" and ck == "resp":
            req_to_resp.append(gap)
        elif pk == "resp" and ck == "req":
            resp_to_req.append(gap)
    summarize("请求→响应 空闲间隔", req_to_resp)
    summarize("响应→下一请求 空闲间隔（轮询周期近似）", resp_to_req)

    print_packet_list(packets, limit=args.list_limit)

    # --- 写 CSV ---
    stem = in_path.stem
    byte_csv = outdir / f"{stem}_byte_intervals.csv"
    pkt_csv = outdir / f"{stem}_packet_intervals.csv"
    write_byte_intervals_csv(byte_csv, bytes_, byte_deltas)
    write_packet_report_csv(pkt_csv, packets)

    print(f"\n已写出:")
    print(f"  {byte_csv}")
    print(f"  {pkt_csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
