# 飞特 STS 舵机固件 — 项目总览（AI 交接文档）

> **用途**：将本文件连同工程代码发给新 AI，即可快速了解项目全貌、当前进度与待办。  
> **最后更新**：2026-07-15（多机时序排查；固件 WIP 仍止于 7/14：Stribeck oneshot + 精调 + 速度 sign-magnitude）  
> **目标产品**：飞特 STS3215c018 磁编码舵机兼容固件  
> **硬件平台**：16999-PS26040802 伺服板（GD32F130F8 + AS5600 + EG2104 H 桥）  
> **代码基准**：`feature/traj-stribeck-flash` @ `4c15bb9` + **工作区未提交**：速度观测、follow、Stribeck、精调、速度 sign-magnitude  

---

## 0. AI 交互约定

> **强制规则**：**用中文回答用户的所有问题。**  
> 代码标识符、寄存器地址、Git 提交号、文件路径等技术名词可保留英文；说明、分析、步骤、结论一律使用中文。

---

## 0.5 日报用进度

> **用途**：写日报的 AI **只读本小节**；项目收工时覆盖更新（约 15～30 行）。模板见 `汇报/日报/日报用进度-模板.md`。  
> **项目名：** STS 舵机固件  
> **进度日期：** 2026-07-15  
> **最后更新：** 2026-07-15（由 7/15 工作与纪要回溯；新日请覆盖）

### 今日做了什么

- 多机通信异常排查：逻辑分析仪对比飞特原厂与自研逐字节时序（原厂约 10μs/字节、自研约 12μs）；串口助手复现上位机同款命令，多机可正常回包。
- 参与舵机齿轮厂商对接会议（纪要见 `厂商对接/齿轮厂商对接纪要_20260715.md`）。

### 完成度 / 结论

- 排除「字节延迟差异导致失败」与「舵机本身应答不通」；多机失败更可能在上位机轮询/超时/判定逻辑；固件与总线时序侧暂未发现足以解释失败的差异。根因未闭环。
- 齿轮厂商会议已对齐加工/配合需求与厂商能力、交期与技术约束，作机械件选型输入。

### 卡点

- 上位机多机通信失败/超时；助手同命令可通。阻塞程度中；需继续对照 FD/上位机侧 RTT、超时阈与多 ID 轮询间隔。

### 需对外协同

- 对接人：上位机/协议相关同事 | 事项：多机在 FD/上位机失败、助手可通 | 需求：对齐轮询间隔、超时阈值与失败判定，必要时共抓上位机日志 | P0

### 明日建议（供日报「明日计划」参考）

- **P0：** 定位上位机多机失败根因（对照轮询时序与超时策略）；目标给出可验证根因与改法。
- **P1：** 顺延位置阶跃 + 速度三角 oneshot/精调再验；Follow 抖动观察视带宽。
- **P2：** WIP Git 提交。

### 相关产物路径（可选）

- `厂商对接/齿轮厂商对接纪要_20260715.md`
- `测试数据/`、`测试脚本/analyze_byte_timing.py` 等时序分析产物

---

## 1. 一句话说明

在自研 GD32F130 电机驱动板上，实现与飞特 STS 舵机协议兼容的固件：半双工 UART 通信 + 0x80 字节虚拟内存表 + 位置/速度/力矩三环电机控制，供上位机（FD 调试软件）直接操控。

---

## 2. 关键路径速查

| 项目 | 路径 |
|------|------|
| Keil 工程 | `servo_project/project/template.uvprojx` |
| 用户源码 | `servo_project/User/` |
| 主循环 | `servo_project/User/main.c` |
| STS 协议 | `servo_project/User/servo/sts_proto.c` |
| STS 内存表 | `servo_project/User/servo/sts_mem.c` |
| Flash EPROM | `servo_project/User/servo/sts_eeprom.c` |
| 电机控制 | `servo_project/User/servo/motor.c` |
| 高频 GOAL 跟随 | `servo_project/User/servo/motor_follow.c` |
| 速度观测 | `servo_project/User/servo/speed.c`（MA + 跨距差分 + 真实 Δt） |
| UART 半双工 | `servo_project/User/servo/uart.c` + `bsp/bsp_uart.c` |
| 参考工程（协议已实现、电机未接） | `E:\wtzn\xuniduoji\xuniduoji\User\` |
| 原始协议文档 | `docs/舵机STS自定义通信协议.docx`、`docs/磁编码STS内存表.docx` |
| PID 稳定性评估 | `docs/PID稳定性评估方案.md`（计划已定，`pid_tune` 未写） |
| 速度观测改进 | `docs/速度观测改进方案.md`（工作区已改 pipeline；VOFA 验收待做） |

---

## 3. 硬件概要

### 3.1 主要器件

| 器件 | 型号 | 功能 |
|------|------|------|
| MCU | GD32F130F8P6TR | 72 MHz，64 KB Flash，TSSOP-20 |
| 磁编码器 | AS5600-ASOM | 12-bit，I2C 地址 0x36 |
| H 桥驱动 | EG2104 × 2 + B3202 × 2 | 有刷直流电机 PWM 驱动 |
| 电流采样 | RS1 0.01Ω + GS8591 | 低端 shunt + 运放 → ADC |

### 3.2 关键引脚（以原理图为准，非例程 board_config.h）

| 信号 | MCU 引脚 | 说明 |
|------|----------|------|
| M+ PWM | PA7 | TIMER2_CH1，AF0，~20 kHz |
| M- PWM | PB1 | TIMER2_CH3，AF0 |
| H 桥 #SD | PA6 | GPIO，**低电平关断** |
| 电流 ADC | PA3 | DMA 扫描 |
| 母线电压 ADC | PA4 | DMA 扫描 |
| I2C SCL/SDA | PA9 / PA10 | I2C1，AF4 |
| 半双工 UART | PA2 | USART1，AF1，**1 Mbps** |
| SWD | PA13 / PA14 | 调试烧录 |

> 例程 `GD32F130例程共32个/gpio_output` 中 PA6/PB1 与原理图接反，**不可照搬**。详见 `docs/硬件说明-16999-PS26040802.md` §8.3。

### 3.3 角度与速度单位

- AS5600 原始值 0~4095 对应 360°
- STS 位置寄存器：1 格 = 0.087°（360° / 4096）
- STS 速度寄存器：1 格/秒 = 0.087°/s（**不是 RPM**）

---

## 4. 软件架构

```text
                    ┌─────────────────────────────────────────┐
  上位机 FD 软件     │  FF FF ID LEN Inst Params CS            │
  (1Mbps 半双工)     └──────────────────┬──────────────────────┘
                                       │ USART1 PA2
                                       ▼
                              uart.c（256B 字节环 + RBNE 中断）
                                       │ uart_rx_pop()
                                       ▼
                              sts_proto.c（组帧状态机 → 3 槽帧环）
                                       │ PING / READ / WRITE
                                       ▼
                              sts_mem.c（memory[0x80] 虚拟寄存器表）
                                       │ 写副作用 / 反馈刷新
                         ┌─────────────┴─────────────┐
                         ▼                           ▼
                  sts_eeprom.c                  motor.c + motor_follow.c
                  Flash 0x0800FC00           轨迹规划 / 跟随 / PID + PWM
                  0x05~0x27 掉电保存
                         ┌─────────────┼─────────────┐
                         ▼             ▼             ▼
                    encoder.c    electricity.c   bsp_pwm.c
                    + speed.c    (ADC DMA)       (H 桥)
                    (AS5600)
```

### 4.1 主循环调度（main.c）

```c
while (1) {
    uart_comm_poll();          // 优先：收字节 → sts_proto_poll → 回复
    sts_mem_poll();            // 50ms 刷新反馈寄存器

    if (motor_adc_flag)        { electricity_update(); uart_comm_poll(); }
    if (motor_encoder_flag)    { encoder_update(); speed_update(); ... }

    if (!sts_mem_control_active())  motor_test_poll();  // STS 接管时跳过
    motor_control(&motor_context);
    uart_comm_poll();
}
```

### 4.2 控制环时序（TIMER13，1 kHz 基准）

| 环 | 频率 | 标志位 |
|----|------|--------|
| 编码器采样 | 1000 Hz | `motor_encoder_flag` |
| 速度观测 | 1000 Hz | **unwrap → MA(4) → 跨距差分 SPAN=16 + 真实 Δt（`g_control_tick`）→ LPF(α=0.3)**；旁支另有 LS(S-01) 实现见 `feature/speed-ls-observation` |
| 位置 PID | 250 Hz | `motor_location_pid_flag` |
| 速度 PID | 500 Hz | `motor_speed_pid_flag` |
| ADC 电流/电压 | 1000 Hz | `motor_adc_flag` |

### 4.3 控制模式

| 模式 | 枚举 | 说明 |
|------|------|------|
| 空 | `motor_control_mode_null` | 不输出 |
| 力矩/开环 | `motor_control_mode_torque` | 直接 duty |
| 速度环 | `motor_control_mode_speed_torque` | T 型加减速 + 速度 PI |
| 位置环 | `motor_control_mode_position_speed_torque` | 轨迹规划 → 速度 PI |

位置环策略（2026-07-14 晚间更新）：
- **粗调**：未进精调区时 `angle_control_plan` 时间域梯形（`plan_angle` 积分）
  - 减速区 `POS_DECEL_HYSTERESIS`；规划到点 `POS_PLAN_ARRIVE_DEG=0.05°` → `s_angle_plan_finished`
  - `target` 变化 / reset 可继承当前速度
- **精调切入（任一即进，同拍可切）**：
  1. 实际误差 `|target−actual| ≤ 1°`
  2. 规划剩余 `|target−plan_angle| ≤ 1°`（更早交棒，减过冲）
  3. 规划到点 `s_angle_plan_finished`（避免规划完却因 |err|>1° 卡住）
  - 退出滞回：`|err| > 1.25°` 才回粗调
- 精调区 `location_pid`（PD，Ki=0）→ `plan_speed`；**精调期间强制关 Stribeck**
- **高频 GOAL 跟随**（`motor_follow.c`）：连续小步 GOAL 流 → 禁用精调、估速刷 `target_speed`
- **Stribeck**：仅零速启动 / 换向 **边沿 oneshot**（见 §4.4），非连续补偿

默认 PID / 运动默认（motor_init + sts_mem）：
- 位置：Kp=5, Kd=0.05, Ki=0
- 速度：Kp=3, Ki=0.1, Kd=0（SPEED_P=3、SPEED_I=10，缩放 `×0.01`）
- `GOAL_ACC` 上电默认寄存器 **100**（×8.7 → °/s²）；RUN_SPEED=0 时位置点到点限速回退 **90 °/s**
- `MIN_START_FORCE` EPROM 默认 **150**（上位机 UI 常限 256）

### 4.4 Stribeck / 位置精调：问题与对策（2026-07-14）

| 现象 | 根因 | 现行对策 |
|------|------|----------|
| 速度模式 `duty` 毛刺粗 | 旧连续 Stribeck 用 noisy `\|v\|` 调幅 / blend | 改为 **边沿 oneshot**，稳态不再常开 |
| 关 `0x18=0` 毛刺明显变好 | 证实毛刺主因是前馈调幅 | 同上；调试可用写 0 对比 |
| 寄存器写 150 ≠ 补偿 150 | 旧 `plan/45`、`exp(\|v\|)` 连乘打折 | oneshot 入助推直接满量 `±MIN_START` |
| 三角波过零贴零平台 | 进助推过晚（等 \|v\| 很低） | 零速上升沿 + 换向沿（\|v\|≤EXIT） |
| 助推太猛 / 尖峰 | 任意反向满推 + 整段清 I | 换向须 \|v\|≤EXIT；清 I 最多削 \|want\|；精调硬关 |
| 退出 `duty` 断崖 | 前馈骤撤 | 退出把 `s_stribeck_applied` **移交速度环积分** |
| 位置到位持续微振 | 精调小 `plan` + 近零反复进 Stribeck | **`s_pos_stop_hold` 时强制关助推** |
| 规划完进不了精调 | 仅 \|err\|≤1° 才进，规划到点时实际仍 >1° | `s_angle_plan_finished` 强制精调 |
| 位置过冲 | 粗调拖到很近才交棒 | **实际 1° 或规划剩余 1°** 即切精调 |

**现行触发（`motor_apply_stribeck_ff`）**：
- 禁止：位置精调 / `MIN_START_FORCE=0`
- 进入：`|plan|` 从 <2 升到 ≥2 且 `|v|≤5`（零速启动）；或 `plan` 符号翻转且 `|v|≤12`（换向）
- 退出：同向且 `|v|≥12` 且最短保持 20ms，或 `|plan|` 回落到阈下；退出移交积分
- 宏：`servo_config.h` 的 `MOTOR_STRIBECK_*`

---

## 5. STS 协议栈（当前核心工作）

### 5.1 实现状态

| 模块 | 文件 | 状态 |
|------|------|------|
| UART 字节环 + 半双工 | `uart.c` / `bsp_uart.c` | ✅ 完成 |
| 帧解析 + PING/READ/WRITE | `sts_proto.c` | ✅ 完成 |
| 0x80 内存表 + 电机桥接 | `sts_mem.c` | ✅ 完成 |
| 反馈刷新（50 ms） | `sts_mem_poll()` | ✅ 完成 |
| 非本机 ID 整帧丢弃 | `sts_proto.c` discard 态 | ✅ 已提交（02a928d） |
| Flash EPROM 持久化 | `sts_eeprom.c` / `sts_mem.c` | ✅ 已完成（`0x0800FC00`，魔数 `0x55AA`） |
| 校准指令 0x0B / RESET 0x0A | `sts_proto.c` + `sts_mem.c` | ✅ 已完成（OFFSET @0x1F，TORQUE_SWITCH=128） |
| SYNC_READ/WRITE | — | ❌ 未做 |
| REG_WRITE + ACTION | — | ❌ 未做 |

### 5.2 帧格式

```text
FF FF | ID | LEN | Instruction + Parameters | Checksum
 2B      1B    1B         LEN-1 字节               1B
```

- Checksum = `~(ID + LEN + Inst + Params...)` 低 8 位
- 广播 ID = `0xFE`
- 已实现指令：PING(0x01)、READ(0x02)、WRITE(0x03)、RESET(0x0A)、CALIB(0x0B)

### 5.3 关键寄存器映射

| 地址 | 名称 | 读写 | 桥接行为 |
|------|------|------|----------|
| 0x05 | SERVO_ID | R/W | 本机 ID |
| 0x15~0x17 | POS_P/D/I | R/W | 写后缩放更新位置 PID |
| 0x18~0x19 | MIN_START_FORCE | R/W | 最小启动力 duty 0~3599（Stribeck 基准） |
| 0x1F | POS_OFFSET | R/W | 位置偏置（signed count），参与 PRESENT/GOAL 换算，EPROM 保存 |
| 0x25, 0x27 | SPEED_P/I | R/W | 写后缩放更新速度 PID |
| 0x37 | EPROM_LOCK | R/W | 1=锁 EPROM 区；0=解锁且写入触发 Flash 保存 |
| 0x28 | TORQUE_SWITCH | R/W | 0=关断；1=使能；128=中位校准后恢复为 1 |
| 0x29 | GOAL_ACC | R/W | → `target_a_speed` |
| 0x2A | GOAL_POS | R/W | → 位置模式；连续 GOAL 按 STS 差分累计多圈（避免跨 0 折返）；可进入跟随子模式 |
| 0x2C | PWM_OPEN_SPEED | R/W | 非 0 → 力矩模式 duty |
| 0x2E | RUN_SPEED | R/W | → 速度模式 + `target_speed`；**bit15 方向 + 低15位幅值**；位置点到点限速取幅值 |
| 0x38 | PRESENT_POS | R | `(encoder_raw + offset) mod 4096` |
| 0x3A | PRESENT_SPEED | R | 同 RUN_SPEED：**bit15 方向 + 低15位幅值**（非 int16 补码） |
| 0x3C | PRESENT_LOAD | R | `encode_load(-duty)`（仅反馈取反） |
| 0x3E | PRESENT_VOLT | R | 母线电压 |
| 0x45 | PRESENT_CUR | R | 母线电流 |

**重要副作用修复（7/7）**：同包写 GOAL_POS + 0x2C=0 时，0x2C=0 不再误切力矩 0% 覆盖位置模式。

**速度单位（7/8 修复）**：幅值 `× (360/4096)` °/s；编解码为飞特 sign-magnitude（`0x8000` 方向位）。

**负载编码（7/8）**：`|duty| × 1000 / 3599`，负向加 `0x0400` 方向位；**上报时对内部 duty 取反**（仅反馈，不改驱动）。

### 5.4 UART 收发策略（踩坑总结）

1. **发期间不关 RBNE**：用 `s_uart_rx_blocked` 丢弃回波，避免第二次 PING 无回包
2. **tx 后不等 TC**：`usart_putc` 已等 TC，`tx_end` 再死等会导致 blocked 永为 1
3. **I2C 禁止关总中断**：`bsp_i2c_read_reg` 中 `__disable_irq` 会导致 1 Mbps UART 丢字节断连（6c17373 已修）
4. **ORE 先读 DATA 再清标志**：防溢出丢字节
5. **RX 帧环 16 槽**（原 10），应对上位机连发多帧
6. **非本机 ID discard**：组帧时按 LENGTH 跳过整帧，减轻多机总线干扰

### 5.5 调试开关（main.c）

```c
#define DEBUG_JUSTFLOAT  1   // 1=VOFA+ 波形，0=上位机 FD 联调
```

- `DEBUG_JUSTFLOAT=1`：5 通道 JustFloat（角度/速度/plan_speed/**target_angle**/duty）
- **与 FD/STS 同占 USART1，联调 STS 时必须改 0**
- `sts_mem_control_active()==0` 时才发 JustFloat，且跳过 `motor_test_poll`

---

## 6. 功能模块完成度

| 编号 | 模块 | 状态 | 说明 |
|------|------|------|------|
| M-01 | 电机直接驱动 | ✅ 已完成 | PWM 20 kHz，正反转，#SD 关断 |
| M-02 | 电流/电压采集 | ✅ 已完成 | TIMER2 TRGO + DMA，1000 Hz |
| M-03 | 位置传感器 | ✅ 已完成 | AS5600 I2C，多圈 unwrap，1000 Hz |
| M-04 | 电流环 | ❌ 未开始 | P4 远期 |
| M-05 | 位置/速度闭环 | 🔶 推进中 | T 型轨迹 + follow + **Stribeck oneshot** + 精调 1° 早切入；过冲/贴零仍待实机整参 |
| C-01 | 通信端口 | ✅ 已完成 | PA2 USART1 半双工 1 Mbps |
| C-02 | 协议框架 | ✅ 已完成 | sts_proto 收帧/组帧/校验 |
| C-03 | STS 内存表 | ✅ 已完成 | sts_mem 0x80 + 桥接 + 连续 GOAL 多圈跟踪 |
| C-04 | Flash EPROM | ✅ 已完成 | `sts_eeprom.c` @0x0800FC00，EPROM_LOCK 写保护 |
| C-05 | 校准/出厂 | 🔶 推进中 | 中位校准 0x0B/0x1F/128 已实现；RESTORE 待做；断电验收待做 |

---

## 7. Git 版本历史（重要节点）

| 提交 | 日期 | 内容 |
|------|------|------|
| `b4ddd17` | 6 月 | 基础框架：PWM、ADC DMA、开环转动 |
| `61a5ec2` | 6 月 | 位置/速度闭环、T 型加减速、VOFA+、motor_test |
| `e7e0b42` | 7/7 | STS 收帧框架 + UART 半双工 |
| `5e857af` | 7/7 | sts_mem + PING/READ/WRITE + UART 多帧修复 |
| `cb0c079` | 7/8 | 速度/扭矩单位对齐上位机 |
| `6c17373` | 7/8 | 去掉 I2C 关总中断，修复通信断连 |
| `02a928d` | 7/8 | RX 非本机 ID 整帧 discard |
| `4c15bb9` | 7/9 | T 型轨迹 + Stribeck + Flash EPROM + 速度观测文档同步 |
| （工作区） | 7/10~14 | 速度观测 MA+SPAN16+Δt；`motor_follow`；Stribeck→oneshot；精调关助推；规划完/1°早切精调；Ki=0.1、ACC=100、默认速 90 |

**当前分支**：`feature/traj-stribeck-flash`（HEAD @ `4c15bb9`）  
**旁支**：`feature/speed-ls-observation` @ `aaa0a61`（S-01 线性回归 N=20，未合入当前工作区）  
**工作区**：相对 HEAD 大量未提交（`motor_follow`、speed、sts_mem、motor Stribeck/精调等）

---

## 8. 已知问题与待办（截至 2026-07-15）

### 8.1 待验证 / 排查中

| 优先级 | 事项 | 说明 |
|--------|------|------|
| **P0** | **上位机多机超时根因** | 7/15：原厂≈10μs/字节、自研≈12μs；助手同命令多机可通。已排除字节延迟与舵机不应答；嫌疑在 FD/上位机轮询·超时·判定。根因未闭环，需协同抓包 |
| **P1** | **速度观测 VOFA 验收** | MA+SPAN16+真实Δt；确认 0↔88 是否消除 |
| P1 | 位置过冲 / 精调早切 | 已改 1°/规划剩余 1°/规划完进精调；三角与点到点再验收 |
| P1 | Stribeck oneshot 工况 | 零速启动、换向贴零平台 vs 冲过头折中；精调无微振 |
| P1 | 多机 discard / ~100°/s 饱和 | discard 已合入；饱和与 discard 实机验收仍待（与 P0 上位机路径分开） |
| P1 | `motor_follow` 流跟随 | 实机验收 |
| P2 | PID Phase 0 / Flash 断电 | 见既有方案 |

### 8.2 未实现（已规划）

| 事项 | 参考 |
|------|------|
| 速度观测线性回归（S-01，可选） | 旁支 `feature/speed-ls-observation` |
| PID 稳定性 MCU 模块（Phase 1） | `pid_tune.c/h` |
| RESTORE 出厂恢复（0x06） | xuniduoji |
| RETURN_DELAY / USART 优先级 | 多机根因若落固件侧再做（当前更倾向上位机） |
| REG_WRITE + ACTION、SYNC_* | 协议后期 |

### 8.3 已解决的关键 Bug

| 现象 | 根因 | 修复 |
|------|------|------|
| 第二次 PING 无回包 | tx_end 死等 TC | blocked 丢回波 |
| 通信断连 | I2C `__disable_irq` | 去掉关总中断 |
| WRITE 电机不动 | 0x2C=0 误切力矩 | 非 0 才进扭矩 |
| 速度单位不符 | RPM×6 | raw×(360/4096) |
| 连续 GOAL 跨 0 折返 | 每包最短路径 | 寄存器差分累计多圈 |
| 速度 VOFA 锯齿 0~88 | 单步差分 | 跨距差分+真实Δt（待 VOFA 终验） |
| Stribeck 稳态毛刺 | 连续调幅跟 noisy speed | oneshot + 精调硬关 |
| 退出断崖 | 前馈骤撤 | 移交速度环积分 |
| 到位精调微振 | 精调反复进 Stribeck | `pos_hold` 关助推 |
| 规划完卡粗调 | 仅 \|err\|≤1 进精调 | `s_angle_plan_finished` 强制进 |
| 过冲偏大 | 交棒过晚 | 实际/规划剩余 1° 即精调 |

---

## 9. 调试方法

### 9.1 Keil Watch 常用变量

| 变量 | 含义 |
|------|------|
| `g_uart_rx_ring` | UART 接收环（head/tail/buf） |
| `g_uart_rx_irq_cnt` | 中断收字节计数 |
| `g_uart_rx_drop_cnt` | 环满丢弃计数 |
| `g_sts_rx_frame_count` | 待处理完成帧数 |
| `motor_context.control.mode` | 当前控制模式 |
| `motor_context.control.plan_speed` | 规划速度 |
| `motor_context.control.target_angle` | 目标角度 |
| `motor_get_duty()` | 当前 PWM 占空比 |
| `g_i2c_last_fail_stage` | I2C 失败阶段（1~8） |

### 9.2 上位机联调

1. `DEBUG_JUSTFLOAT` 改 0，Rebuild 烧录
2. FD 软件连接 1 Mbps 半双工
3. 先 PING → READ 0x38 → WRITE GOAL_POS / RUN_SPEED
4. 抓包对照 `docs/STS完整实现指南.md` 附录 D

### 9.3 VOFA+ 波形联调

1. `DEBUG_JUSTFLOAT` 改 1
2. 通道：multi_degree / speed / plan_speed / target_angle / duty
3. `motor_test_poll()` 可通过 Keil Watch 改 `motor_cmd` 做本地测试

### 9.4 PID 稳定性整参（计划）

> 完整规格：[PID稳定性评估方案.md](./PID稳定性评估方案.md)

**Phase 0（当前，零代码）**：

1. 速度环：`motor_cmd_speed` + `direct`，setpoint 阶跃 0→60 °/s，VOFA 录 ≥ 8 s
2. 位置环：`motor_cmd_position` + `direct`，setpoint 阶跃 60°，录 ≥ 10 s
3. 离线算 `e_v = plan_speed - speed`、`e_θ = target_angle - angle`
4. 指标：`settle_ms`、`steady_pp`、`steady_rms`、`stable`（详见方案 §2）

**Phase 1（待实现）**：`pid_tune.c/h` → Watch `g_pid_tune_speed` / `g_pid_tune_pos`

**整参顺序**：先 **速度观测 VOFA 验收** 稳定 ch2 → 速度环单独通过 → 位置精调区 → STS 写参验证

### 9.5 速度观测（当前实现）

> 完整规格：[速度观测改进方案.md](./速度观测改进方案.md)

- 原现象：ch2 `speed` 在 plan≈50 °/s 时振荡 0↔88 °/s
- 工作区实现（非旁支 LS）：
  - unwrap 多圈角（raw 差分累积）
  - `SPEED_MA_WIN=4` 滑动平均
  - `SPEED_DIFF_SPAN=16` 滤波角首尾差分 / 真实 `Δt`（`g_control_tick`）
  - `SPEED_LPF_ALPHA=0.3`
- 旁支 `feature/speed-ls-observation`：S-01 线性回归 N=20 + 弱 LPF
- **待做**：按方案 §9 做 VOFA 匀速验收；未过则再合入/对比 LS 方案

### 9.6 高频 GOAL 跟随（`motor_follow`）

- 入口：`sts_on_write(GOAL_POS)` → `motor_follow_on_goal_pos()`
- 进入条件（概要）：连续 `ENTER_COUNT` 次小步 GOAL 流
- 激活后：禁用精调，`target_speed` 由 GOAL 差分 LPF 估计
- 退出：流中断 / 大跳变 / 空闲超时（`MOTOR_FOLLOW_*`）

### 9.7 Stribeck / 精调联调要点

1. 对比：`0x18=0` 关助推 vs `150` 开助推（Flash 旧值需解锁改或擦 EPROM）
2. 速度三角：看过零贴零平台是否可接受、是否再冲过头
3. 位置阶跃：进 1° 应切精调（VOFA：`plan_speed` 改由位置 PD 输出）；到位不应微振
4. 参数：`servo_config.h` → `MOTOR_STRIBECK_*`、`POS_PLAN_DEADZONE_DEG`

---

## 10. 文档索引

| 文档 | 内容 | 新鲜度 |
|------|------|--------|
| **本文档** | 项目总览，AI 交接入口 | ✅ 2026-07-15（§0.5/§8；固件 WIP 描述仍 7/14） |
| `docs/STS完整实现指南.md` | STS 六阶段路线图 + 附录地址表 | ✅ 2026-07-15（§0 多机结论） |
| `docs/STS协议自实现指南.md` | 协议细节、帧格式、校准测试用例 | ✅ 2026-07-10 |
| `docs/硬件说明-16999-PS26040802.md` | 原理图引脚、BOM、电源 | ✅ 引脚以 §8.3 为准 |
| `docs/功能实现任务拆解.md` | 全量任务表 + §0/§8 进度快照 | ✅ 2026-07-15 |
| `docs/功能实现任务拆解-语雀版.md` | 语雀发布版（含硬件分析） | ✅ 2026-07-15 |
| `docs/M-01-电机驱动实现笔记.md` | PWM / H 桥实现 | ✅ |
| `docs/M-02-M03-感知采集实现笔记.md` | ADC / AS5600 | ✅ |
| `docs/GD32F1x0-外设笔记.md` | 外设 API 笔记（2057 行） | ✅ 参考手册 |
| `docs/PID稳定性评估方案.md` | PID 整参：指标定义、测试流程、pid_tune 设计 | ✅ 2026-07-14 同步速度观测描述 |
| `docs/速度观测改进方案.md` | 1000Hz 速度量化噪声：S-01~S-06；工作区跨距差分已实施 | ✅ 2026-07-14 |
| `docs/研发日报-2026-07-0*.txt` | 每日工作记录 | ✅ 7/1~7/9 |

---

## 11. 给新 AI 的建议起手式

0. **全程用中文回复用户**（见 §0）；技术名词可保留英文
1. **先读本文档 §5~§8**，了解 STS 协议栈现状与 open issues
2. **读 `main.c`** 理解主循环调度与 DEBUG 开关
3. **读 `sts_mem.c` 的 `sts_on_write()`**，理解寄存器→电机副作用
4. **改 UART/协议相关时**，同时看 `uart.c` 和 `bsp_i2c.c`（I2C 与 UART 并发）
5. **对照参考工程** `E:\wtzn\xuniduoji` 时，注意 xuniduoji **电机控制未接**，只借鉴协议/Flash 层
6. **引脚以 `docs/硬件说明` §8.3 为准**，不要抄例程 board_config.h
7. **整参 PID 时**，先读 `docs/PID稳定性评估方案.md`；**速度 ch2** 先读 `docs/速度观测改进方案.md`，验收过后再抬 Kp
8. **改 GOAL_POS 路径** 时同时看 `sts_mem.c`（多圈跟踪）与 `motor_follow.c`
9. **对比 LS 速度观测** 时切到旁支 `feature/speed-ls-observation`，勿与当前跨距差分混拷

### 当前最可能的下一个任务

```
P0：上位机多机超时根因（对照 FD 轮询/超时；助手可通 vs 上位机失败）
P1：位置过冲/精调切入 + Stribeck oneshot 实机再验（三角速 + 位置阶跃）
P1：速度观测 VOFA 验收；motor_follow / discard·饱和实机验收
P2：PID Phase 0；Flash 断电验收；提交工作区 WIP
```

---

## 12. 工程目录结构

```text
servo/
├── docs/                          # 文档（本文件为入口）
│   ├── 项目总览-AI交接文档.md      # ← 你正在读的文件
│   ├── PID稳定性评估方案.md
│   ├── 速度观测改进方案.md        # 1000Hz 速度量化噪声 S-01~S-06
│   ├── STS完整实现指南.md
│   ├── STS协议自实现指南.md
│   ├── 硬件说明-16999-PS26040802.md
│   ├── 功能实现任务拆解.md
│   ├── 功能实现任务拆解-语雀版.md
│   ├── GD32F1x0-外设笔记.md
│   └── 研发日报-2026-07-*.txt
├── servo_project/
│   ├── project/template.uvprojx   # Keil 工程
│   └── User/
│       ├── main.c
│       ├── servo/                 # 应用层
│       │   ├── motor.c/h          # 电机控制 + 轨迹规划 + Stribeck
│       │   ├── motor_follow.c/h   # 高频 GOAL 跟随子模式
│       │   ├── pid.c/h
│       │   ├── encoder.c/h        # AS5600
│       │   ├── electricity.c/h    # ADC 电流电压
│       │   ├── speed.c/h          # 速度观测（MA + 跨距差分 + Δt）
│       │   ├── uart.c/h           # UART 环 + poll
│       │   ├── sts_proto.c/h      # STS 帧解析
│       │   ├── sts_mem.c/h        # STS 内存表 + EPROM 桥接
│       │   ├── sts_eeprom.c/h     # Flash 掉电保存（0x05~0x27）
│       │   ├── sts_mem_map.h      # 地址常量
│       │   ├── motor_test.c/h     # 本地调试
│       │   ├── data_send.c/h      # VOFA+ JustFloat
│       │   └── servo_config.h     # 全局类型 / FOLLOW / STRIBECK 宏
│       └── bsp/                   # 板级驱动
│           ├── bsp_pwm.c/h
│           ├── bsp_adc.c/h
│           ├── bap_i2c.c/h
│           ├── bsp_uart.c/h
│           └── bsp_gpio.c/h
└── README.md
```

---

### 5.6 Flash EPROM 持久化（2026-07-09 合入）

参考 xuniduoji `servo_memory.c`，拆分为独立模块 `sts_eeprom.c`（xuniduoji 为同文件 static 函数）：

| 项目 | 说明 |
|------|------|
| Flash 页 | `0x0800FC00`（64 KB Flash 最后一页） |
| 布局 | `[0x55][0xAA][0x05~0x27 共 35 字节]` |
| 启动 | `sts_eeprom_load` → 全零则 `sts_mem_set_eprom_defaults` + save |
| 保存触发 | `EPROM_LOCK==0` 且写入触及 `0x05~0x27` |
| 默认锁 | 每次启动 `EPROM_LOCK=1`，不影响正常运行 |
| Keil | `template.uvprojx` 已加入 `sts_eeprom.c`（`gd32f1x0_fmc.c` 已有） |

### 5.7 中位校准（2026-07-09 合入）

参考 xuniduoji `protocol.c` + `servo_memory.c`，`POS_OFFSET` @ `0x1F` 位于 EPROM 区，随 Flash 保存。

| 项目 | 说明 |
|------|------|
| 坐标换算 | `PRESENT_POS = (encoder_raw + offset) mod 4096` |
| GOAL_POS | 首包：相对 PRESENT 最短路径开多圈；**后续连续包**：按 STS 寄存器差分累计，避免跨 0 反向 |
| CALIB `0x0B` | `offset = target - raw`，默认 target=2048；可带 2B 参数 |
| RESET `0x0A` | offset 清零 |
| TORQUE_SWITCH=128 | 等效 CALIB(2048)，完成后写回 1 并使能 |
| 持久化 | `EPROM_LOCK==0` 时写入 offset 触发 `sts_eeprom_save` |
| 位置单位 | `STS_POS_UNIT_DEG = 360/4096`（与 encoder 一致，非近似 0.087） |

**常用帧：**

```text
CALIB 中点：  FF FF 01 02 0B F1
RESET 偏移：  FF FF 01 02 0A F2
WRITE 128：   FF FF 01 03 03 28 80 D9   （校验随 ID 变化）
```

### 5.8 高频 GOAL 跟随（2026-07-14 工作区）

`motor_follow.c` 在协议仍为位置模式的前提下，识别上位机高频小步 `GOAL_POS` 流：

| 项目 | 说明 |
|------|------|
| 进入 | 连续命中 `MOTOR_FOLLOW_ENTER_COUNT`（默认 4）且步长/间隔在阈值内 |
| 行为 | 跳过 `angle_control_plan_reset`；禁用精调；`target_speed` 由 GOAL 差分 + LPF 估计 |
| 退出 | miss 累计 / 大跳变 `JUMP_DEG` / 空闲 `IDLE_DT_MS` |
| Keil | `template.uvprojx` 已加入 `motor_follow.c/h` |

### 5.9 Stribeck oneshot（2026-07-14 晚工作区）

见本文 **§4.4**。实现文件：`motor.c` `motor_apply_stribeck_ff()`；参数：`servo_config.h` `MOTOR_STRIBECK_*`；启动力：`0x18~0x19` / `sts_mem_get_min_start_force()`。

---

*本文档固件 WIP 整理自 2026-07-14 晚；§0.5/§8 已同步 2026-07-15 多机排查。重大进展请同步 §0.5、§4.3、§4.4、§7、§8、§9.7，并更新 `STS完整实现指南` / `功能实现任务拆解`（及语雀版）进度快照。*
