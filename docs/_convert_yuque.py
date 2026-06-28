# -*- coding: utf-8 -*-
"""Convert 功能实现任务拆解.md to Yuque-compatible markdown."""
import re
from pathlib import Path


def yuque_anchor(heading: str) -> str:
    s = heading.strip()
    s = re.sub(r"[（）()·→/]", "-", s)
    s = re.sub(r"\s+", "-", s)
    s = re.sub(r"-+", "-", s)
    return s.strip("-")


def main() -> None:
    src = Path(__file__).parent / "功能实现任务拆解.md"
    text = src.read_text(encoding="utf-8")

    id_to_heading: dict[str, str] = {}
    lines = text.splitlines()
    i = 0
    while i < len(lines):
        m = re.match(r'<a id="([^"]+)"></a>', lines[i].strip())
        if m:
            aid = m.group(1)
            j = i + 1
            while j < len(lines) and not lines[j].strip():
                j += 1
            if j < len(lines):
                hm = re.match(r"^#{2,6}\s+(.+)$", lines[j].strip())
                if hm:
                    id_to_heading[aid] = hm.group(1).strip()
        i += 1

    text = re.sub(r'^<a id="[^"]+"></a>\s*\n', "", text, flags=re.MULTILINE)

    def repl_link(m: re.Match) -> str:
        path = m.group(1)
        if path.startswith("../"):
            path = path[3:]
        return f"`{path}`"

    text = re.sub(r"\[[^\]]*\]\(\.\./([^)]+)\)", repl_link, text)

    extra = {
        "motor": "电机类",
        "comm": "通信类",
        "3-任务详情": "3. 任务详情",
        "2-任务总表": "2. 任务总表",
    }

    def repl_anchor(m: re.Match) -> str:
        prefix, aid = m.group(1), m.group(2)
        heading = id_to_heading.get(aid) or extra.get(aid)
        if heading:
            return f"{prefix}#{yuque_anchor(heading)})"
        return m.group(0)

    text = re.sub(r"(\[[^\]]*\]\()\#([^)]+)\)", repl_anchor, text)

    header = """# 功能实现任务拆解 — 16999-PS26040802 伺服板（语雀版）

> **语雀使用说明**
> - 本文档已适配语雀 Markdown：无 HTML 锚点、代码路径为行内代码、页内链接指向标题。
> - 导入方式：知识库 → **导入** → **Markdown** → 选择本文件。
> - 若某跳转链接失效：在语雀编辑器中输入 `[` 可自动选择标题补全链接。
> - 代码路径请在本地 IDE 中按路径打开；后续可替换为 Git 仓库 URL。
>
> 板卡：16999-PS26040802（GD32F130 + AS5600 + EG2104 H 桥）  
> 工程路径：`servo_project/`  
> 硬件参考：硬件说明-16999-PS26040802.md（同知识库内链接）  
> 编写日期：2026-06-23 · 文档版本：v2.0-语雀

[TOC]

---

"""

    text = re.sub(
        r"^# 功能实现任务拆解 — 16999-PS26040802 伺服板\n\n> 板卡：.*?\n\n---\n\n",
        header,
        text,
        count=1,
        flags=re.DOTALL,
    )

    text = text.replace(
        "> **快速跳转：** [电机类](#motor) · [通信类](#comm) · [详情区](#3-任务详情)",
        "> **快速跳转：** [电机类](#电机类) · [通信类](#通信类) · [详情区](#3.-任务详情)",
    )

    text = text.replace("[任务名称](#x-xx)", "[任务名称](#X-XX-任务名称)")
    text = text.replace("[子任务名称](#x-xx-1)", "[子任务名称](#X-XX-1-子任务名称)")

    # 附录修订记录
    if "v2.0-语雀" not in text:
        text = text.replace(
            "| 2026-06-24 | v2.0 | 重构：任务总表（大/小/子任务+状态+跳转）、统一详情模板、附录模板 |",
            "| 2026-06-24 | v2.0 | 重构：任务总表（大/小/子任务+状态+跳转）、统一详情模板、附录模板 |\n"
            "| 2026-06-24 | v2.0-语雀 | 语雀适配版：去 HTML 锚点、代码路径改行内代码、[TOC] 目录 |",
        )

    out = Path(__file__).parent / "功能实现任务拆解-语雀版.md"
    out.write_text(text, encoding="utf-8")
    print(f"Written {out} ({len(text.splitlines())} lines)")
    print(f"Mapped {len(id_to_heading)} anchors")


if __name__ == "__main__":
    main()
