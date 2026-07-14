# -*- coding: utf-8 -*-
"""Fix Yuque doc: remove broken hand-written anchor links.

Applied to ``功能实现任务拆解-语雀版.md`` (v3.x rich doc). After manual content
updates, run this script to strip stale ``[text](#anchor)`` links before re-import.
"""
import re
from pathlib import Path

p = Path(__file__).parent / "功能实现任务拆解-语雀版.md"
text = p.read_text(encoding="utf-8")

# [text](#anchor) -> text
text = re.sub(r"\[([^\]]+)\]\(#[^)]+\)", r"\1", text)

# Remove 详情 column from table header and rows
text = text.replace("| 状态 | 详情 |", "| 状态 |")
text = text.replace("| 状态 | 详情 |", "| 状态 |")
text = re.sub(r" \| \[→\][^\n]*", "", text)
text = re.sub(r" \| — \| — \| — \|", " | — | — |", text)  # 大类行可能多列

# Fix 大类 rows that had extra column
text = text.replace("| 大类 | M | 电机类 | — | — | — |", "| 大类 | M | 电机类 | — | — |")
text = text.replace("| 大类 | C | 通信类 | — | — | — |", "| 大类 | C | 通信类 | — | — |")

# Remove broken return-to-table links
text = re.sub(r"\n\n\[↑ 总表[^\]]*\]\([^)]+\)\n", "\n", text)
text = re.sub(r"\n\[↑ 返回总表\]\([^)]+\)\n", "\n", text)

# Update quick jump to plain text
text = text.replace(
    "> **快速跳转：** 硬件分析 · 电机类 · 通信类 · 详情区（请用右侧 **大纲** 或文首 **目录** 跳转）",
    "> **快速跳转：** 硬件分析 · 电机类 · 通信类 · 详情区（请用右侧 **大纲** 或文首 **目录** 跳转）",
)
old_quick = "> **快速跳转：** [硬件分析](#2.-硬件分析) · [电机类](#电机类) · [通信类](#通信类) · [详情区](#6.-任务详情)"
new_quick = "> **快速跳转：** 请使用语雀右侧 **大纲面板**，或文首 `[TOC]` 自动目录（见 §4.2）"
text = text.replace(old_quick, new_quick)

# Update header note
old_compat = "> **语雀兼容说明：** 子任务采用标题 + 表格直接展开（语雀无法正确渲染 HTML 折叠块内的 Markdown 表格）。"
new_compat = """> **语雀兼容说明：**
> - 子任务为标题 + 表格直接展开（勿用 HTML 折叠块）。
> - **页内跳转**：语雀标题锚点为导入后自动生成的随机 ID（如 `#kxm3a`），无法在外部 Markdown 里预写；请用 **大纲面板** 或导入后在语雀内 **插入链接 → 选择当前文档标题**。"""
text = text.replace(old_compat, new_compat)

# Add section 4.2 if not exists - insert after 如何添加新任务 block
nav_section = """
### 4.2 语雀里如何跳转到某个任务（重要）

语雀 **不能跳到某一行**，只能跳到 **某个标题块**。手写 `[链接](#M-01-xxx)` 在导入后 **无效**，点击可能新开页面。

**推荐方式（无需手改链接）：**

1. 阅读时打开文档右侧 **「大纲」** 面板，点击 `M-02-1 ADC 时钟与 GPIO 初始化` 等标题即可跳转。
2. 文首 `[TOC]` 会生成可点击目录（部分语雀版本支持）。

**若要在总表里做可点击跳转（导入后操作一次）：**

1. 在 **阅读模式** 下，鼠标悬停目标标题左侧，点击 **#** 图标，复制锚点（形如 `#abcde`）。
2. 回到编辑模式，选中总表中的任务名 → **插入链接** → 粘贴该锚点。
3. 或：选中文字 → 插入链接 → **选择当前文档的标题**（语雀自动匹配，最省事）。

**编号即定位：** 总表「编号」列（如 `M-02-1`）与详情区标题一致，可用 `Ctrl+F` 搜索编号快速定位。

"""

if "### 4.2 语雀里如何跳转" not in text:
    text = text.replace("### 文档结构\n", nav_section + "### 文档结构\n")

# Appendix template - remove link syntax
text = text.replace(
    "| 任务 | X-XX | [任务名称](#X-XX-任务名称) | P? | 未开始 | [→](#X-XX-任务名称) |",
    "| 任务 | X-XX | 任务名称 | P? | 未开始 |",
)
text = text.replace(
    "| 子任务 | X-XX-1 | [子任务名称](#X-XX-1-子任务名称) | P? | 未开始 | [→](#X-XX-1-子任务名称) |",
    "| 子任务 | X-XX-1 | 子任务名称 | P? | 未开始 |",
)

# Revision
if "v3.4-语雀" not in text:
    text = text.replace(
        "| 2026-07-02 | v3.3-语雀 | 同步固件进度：M-01 完成；M-02/03/05、C-01/03 推进中；双环 PID+轨迹规划；USART1 1Mbps；STS 协议为 C-02 下一步；更新 §5 状态表、§6 代码路径、§9 工程快照 |",
        "| 2026-07-02 | v3.3-语雀 | 同步固件进度：M-01 完成；M-02/03/05、C-01/03 推进中；双环 PID+轨迹规划；USART1 1Mbps；STS 协议为 C-02 下一步；更新 §5 状态表、§6 代码路径、§9 工程快照 |\n"
        "| 2026-07-10 | v3.4-语雀 | 同步 7/9 工程：STS 协议栈、Flash EPROM、中位校准、T 型轨迹、Stribeck |",
    )

text = text.replace("· 文档版本：v3.2-语雀", "· 文档版本：v3.4-语雀")
text = text.replace("· 文档版本：v3.3-语雀", "· 文档版本：v3.4-语雀")

p.write_text(text, encoding="utf-8")
print("fixed:", p)
