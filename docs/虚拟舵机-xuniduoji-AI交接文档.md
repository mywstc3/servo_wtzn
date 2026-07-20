# 虚拟舵机（xuniduoji）— AI 交接文档

> **用途**：说明参考工程 `wt_project/xuniduoji` 的定位、串口通信实现与协议栈，供自研舵机固件对照借鉴。  
> **最后更新**：2026-07-20  
> **工程路径**：`E:\wtzn\wt_project\xuniduoji\xuniduoji\`  
> **关联文档**：自研固件入口见 `docs/项目总览-AI交接文档.md`

---

## 0. AI 交互约定

> **强制规则**：**用中文回答用户的所有问题。**  
> 代码标识符、寄存器地址、文件路径等可保留英文。

---

## 1. 一句话说明

在 GD32F130 上实现 **STS 协议兼容的“虚拟舵机”**：半双工 UART + 0x80 内存表 + AS5600 位置反馈；**协议/Flash/校准齐全**，但 **电机控制未接**（写 GOAL 不驱动电机）。自研 `servo` 工程主要借鉴其协议层与 EPROM，不照搬电机控制。

---

## 2. 关键路径速查

| 项目 | 路径 |
|------|------|
| 用户源码 | `xuniduoji/User/` |
| 主循环 | `User/main.c` |
| UART 半双工 | `User/uart1.c` / `uart1.h` |
| STS 协议状态机 | `User/protocol.c` / `protocol.h` |
| 0x80 内存表 + Flash | `User/servo_memory.c` / `servo_memory.h` |
| AS5600 | `User/IIC_AS5600.c` |
| ADC | `User/ADC.c` |
| Bootloader / Ymodem | `Bootloader/` |
| 自研对照 UART | `servo/servo_project/User/servo/uart.c` |

---

## 3. 与自研舵机的关系

| 维度 | xuniduoji（虚拟） | servo（自研实机） |
|------|-------------------|-------------------|
| 目标 | 协议联调 / FD 上位机验证 | 实机三环控制 + 协议 |
| 电机 | ❌ 未接 | ✅ PWM + PID + 轨迹 |
| UART | 显式切 REN/TEN | HDEN 下 **不切** REN/TEN，用 `s_uart_rx_blocked` 丢回波 |
| 波特率 | 代码默认 **115200**（注释留 1Mbps） | **1 Mbps** |
| 协议指令 | 更全（REG_WRITE/ACTION/SYNC/RESTORE/REBOOT/JUMP_BOOT） | PING/READ/WRITE/RESET/CALIB 为主 |
| Flash EPROM | 同文件内 static | 拆成 `sts_eeprom.c` |
| 多机 | 非本机 ID：**直接 return，不组帧丢弃** | 有 discard 态按 LENGTH 跳过 |

**借鉴边界**：只抄协议/Flash/校准；半双工收发策略以自研 `uart.c` 注释为准，勿把虚拟舵机的 REN/TEN 切换原样搬进 HDEN 实机。

---

## 4. 串口通信架构（核心）

```text
上位机 / FD
    │  PA2 USART1，半双工 HDEN
    ▼
USART1_IRQHandler（uart1.c）
    │  ORE 先读 DATA；RBNE → 256B 环形缓冲 Uart1_rb
    ▼
main 循环 while (head != tail)
    │  弹出字节 → protocol_parse_byte()
    │  旁路检测 ASCII "UPDATE" → 写升级标志复位进 Boot
    ▼
protocol.c 状态机组帧 → process_packet()
    │  校验 + ID 匹配 → 指令分发
    ▼
servo_memory.c（memory[128]）
    │  READ/WRITE / Flash / 校准写 OFFSET
    ▼
应答：protocol_send_response() → Uart1_SendNByte()
```

### 4.1 硬件与初始化（`uart1_init`）

| 项 | 实现 |
|----|------|
| 外设 | USART1，引脚 **PA2**，`GPIO_AF_1` |
| GPIO | AF + **开漏 OD** + 内部上拉 |
| 模式 | `USART_CTL2_HDEN` 半双工 |
| 帧格式 | 8N1 |
| 波特率 | `115200U`（旁注可改 `1000000U`） |
| 中断 | RBNE，NVIC 优先级 2 |
| 上电清错 | 读标志清 ORE；排空 RBNE；环缓冲清零 |

### 4.2 收：中断 + 环缓冲

- `USART1_IRQHandler` 定义在 **`uart1.c`**（不在 `gd32f1x0_it.c`）。
- ORE：读 `usart_data_receive` 清标志。
- RBNE：读字节写入 `Uart1_rb`；满则丢弃。
- 环大小：`UART1_RB_SIZE = 256`。
- 主循环直接读 `Uart1_rb.head/tail`（也提供 `Uart1_ReceiveByte` / `_NonBlocking`）。

### 4.3 发：显式收发模式切换

虚拟舵机用两套配置函数切换方向：

```c
Usart1_Transmit_Mode();  // REN 关，TEN 开
// 等 TBE 逐字节发，整帧后再等 TC
delay_us(100);
Usart1_Receive_Mode();   // REN 开，TEN 关
```

| API | 行为 |
|-----|------|
| `Uart1_SendByte` | 切发送 → 1 字节 → 等 TC → delay 100µs → 切接收 |
| `Uart1_SendString` | 同上，整串发完再切回 |
| `Uart1_SendNByte` | **协议应答主路径**；整缓冲发完再切回 |

应答前 `protocol_send_response` 还会：

1. 排空硬件 RBNE 残留；
2. 读一次清 ORE；
3. 组帧 `FF FF | ID | LEN | error | params | CS`；
4. `Uart1_SendNByte(buf, idx)`。

广播 ID `0xFE`：**不回包**。

### 4.4 与自研半双工策略对比（易踩坑）

| | 虚拟舵机 | 自研 servo |
|--|----------|------------|
| 发时 RBNE | 发前关接收（`RECEIVE_DISABLE`） | **保持 RBNE 开** |
| 回波处理 | 靠关接收 + 发前排空 | `s_uart_rx_blocked=1` 在 IRQ 里丢弃 |
| 发后 | 等 TC + `delay_us(100)` 再开接收 | `tx_end`：delay 100µs + 清错 + 解除 blocked |
| HDEN 注意 | 工程上能跑，但切 REN/TEN 与 HDEN 语义可能冲突 | 注释明确：**HDEN 勿切 REN/TEN** |

自研踩坑（已写在总览 §5.4）：发期间关 RBNE 会导致第二次 PING 无回包；`tx_end` 再死等 TC 会卡住 blocked。对照虚拟舵机时，**理解意图（防回波）即可，实现跟自研方案**。

---

## 5. 协议解析（`protocol.c`）

### 5.1 逐字节状态机

```text
IDLE → HEAD1(0xFF) → ID → LEN → INST → PARAM* → CHECKSUM → process_packet → IDLE
```

- Checksum：`~(ID + LEN + Inst + Params)` 低 8 位；错则静默丢弃。
- `last_byte_time`：每字节更新；主循环里 **50ms 超时复位目前注释掉**。
- **非本机且非广播**：`process_packet` 开头直接 `return`（不进入 discard 跳过剩余字节）。多机总线上，后续字节可能污染下一帧状态机——自研已用 discard 态改进。

### 5.2 已实现指令

| 指令 | 码 | 说明 |
|------|-----|------|
| PING | 0x01 | 空参数应答 |
| READ | 0x02 | 读内存块 |
| WRITE | 0x03 | 写内存；GOAL 写点仅占位注释 |
| REG_WRITE | 0x04 | 缓存，等 ACTION |
| ACTION | 0x05 | 提交 REG_WRITE |
| RESTORE | 0x06 | 除 ID 外恢复默认并应答 |
| REBOOT | 0x08 | 关扭矩后系统复位（无应答） |
| JUMP_BOOT | 0x09 | 应答后写升级标志复位 |
| RESET | 0x0A | OFFSET 清零 |
| CALIBRATE | 0x0B | `offset = target - raw`，默认 target=2048 |
| SYNC_READ | 0x82 | 本机命中则单独回包 |
| SYNC_WRITE | 0x83 | 按 ID 段写入 |

自研尚未做：REG_WRITE/ACTION、SYNC_*、RESTORE、REBOOT、JUMP_BOOT。

### 5.3 主循环调度（`main.c`）

```c
while (1) {
    while (Uart1_rb.head != Uart1_rb.tail) {
        // 弹出字节 → protocol_parse_byte
        // 旁路匹配 "UPDATE" → Bootloader
    }
    // 每 50ms：servo_update_present_position()
    // LED 心跳
}
```

- App 起始：`0x08004000`，启动时 `SCB->VTOR` 重定向。
- 上电发 `"ok"` 字符串（非 STS 帧）。
- **无电机控制循环**；位置反馈靠 50ms 刷 AS5600。

---

## 6. 内存表与 Flash（摘要）

- `memory[128]`；EPROM 区 `0x05~0x27`；Flash 页 `0x0800FC00`，魔数 `0x55AA`。
- `MEM_LOCK`(0x37)=1 时拒绝写 EPROM；=0 且写触及 EPROM 则 `eeprom_save`。
- 校准：`PRESENT = (raw + offset)`；CALIB/RESET 改 `MEM_OFFSET_*`。
- `servo_restore_params()`：RESTORE 指令入口。

细节可直接读 `servo_memory.c`；自研对应 `sts_mem.c` + `sts_eeprom.c`。

---

## 7. Bootloader 与串口

| 触发 | 行为 |
|------|------|
| 主循环收到 ASCII `"UPDATE"` | 提示后写标志复位 |
| STS `INST_JUMP_BOOT` (0x09) | 先 STS 应答，再写标志复位 |
| Boot 内 | 可关 RBNE，改轮询；Ymodem 走同 USART1 |

升级路径与自研实机无关时可忽略；对照时注意 **App/Boot 共用 PA2**。

---

## 8. 给对照自研时的建议

1. **协议指令表**：缺 REG_WRITE / SYNC / RESTORE 时优先看 `protocol.c` 的 `process_packet`。
2. **Flash / 校准**：对照 `servo_memory.c` 的 `eeprom_*` 与 OFFSET 换算；自研已拆模块。
3. **半双工**：虚拟舵机 = 关接收防回波；自研 = blocked 丢回波。改 `uart.c` 时以自研注释为准。
4. **多机**：虚拟工程非本机直接 return；自研有 discard，排查多机超时勿假设两边行为一致。
5. **波特率**：虚拟默认 115200，实机 1Mbps；联调 FD 前确认两边一致。
6. **不要期待电机响应**：写 GOAL 在虚拟工程里基本是“写表”。

---

## 9. 目录结构（用户相关）

```text
xuniduoji/xuniduoji/
├── User/
│   ├── main.c              # 环缓冲消费 + 协议 + 50ms 位置
│   ├── uart1.c/h           # HDEN 半双工、环缓冲、发时切 REN/TEN
│   ├── protocol.c/h        # STS 状态机 + 全指令
│   ├── servo_memory.c/h    # 0x80 表 + Flash EPROM
│   ├── IIC_AS5600.c/h
│   ├── ADC.c/h
│   └── ...
├── Bootloader/             # IAP + Ymodem
└── SYSTEM/ / GB_Lib/       # 芯片与库
```

---

## 10. 串口关键代码锚点

| 主题 | 文件:关注点 |
|------|-------------|
| 初始化 HDEN / OD / 波特率 | `uart1.c` → `uart1_init` |
| IRQ 入环 | `uart1.c` → `USART1_IRQHandler` |
| 发时切模式 | `Usart1_Transmit_Mode` / `Receive_Mode` / `Uart1_SendNByte` |
| 应答清残留 | `protocol.c` → `protocol_send_response` |
| 组帧状态机 | `protocol.c` → `protocol_parse_byte` |
| 主循环消费环 | `main.c` → `while (head != tail)` |
| 自研对照 | `servo/.../uart.c` → `s_uart_rx_blocked` / `uart_comm_tx_*` |

---

*本文档整理自 2026-07-20 对 `xuniduoji` 源码的阅读；侧重串口与协议。虚拟工程若有更新，请同步 §4、§5 与 §3 对比表。*
