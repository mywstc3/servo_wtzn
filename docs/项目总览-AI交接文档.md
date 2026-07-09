# 飞特 STS 舵机固件 — 项目总览（AI 交接文档）

> **用途**：将本文件连同工程代码发给新 AI，即可快速了解项目全貌、当前进度与待办。  
> **最后更新**：2026-07-09  
> **目标产品**：飞特 STS3215c018 磁编码舵机兼容固件  
> **硬件平台**：16999-PS26040802 伺服板（GD32F130F8 + AS5600 + EG2104 H 桥）

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
| 电机控制 | `servo_project/User/servo/motor.c` |
| UART 半双工 | `servo_project/User/servo/uart.c` + `bsp/bsp_uart.c` |
| 参考工程（协议已实现、电机未接） | `E:\wtzn\xuniduoji\xuniduoji\User\` |
| 原始协议文档 | `docs/舵机STS自定义通信协议.docx`、`docs/磁编码STS内存表.docx` |
| PID 稳定性评估 | `docs/PID稳定性评估方案.md`（计划已定，代码未改） |
| 速度观测改进 | `docs/速度观测改进方案.md`（1000Hz 量化噪声，待 S-01 实施） |

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
                                       ▼
                              motor.c（轨迹规划 + PID + PWM）
                         ┌─────────────┼─────────────┐
                         ▼             ▼             ▼
                    encoder.c    electricity.c   bsp_pwm.c
                    (AS5600)     (ADC DMA)       (H 桥)
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
| 速度观测 | 1000 Hz | 差分→中值(5)→MA(8)→LPF；**待改**见 `速度观测改进方案.md` |
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

位置环策略（7/2 定型）：
- |误差| > 1°：`angle_control_plan` 按加速度/最大速度规划 `plan_speed`
- |误差| ≤ 1°（带 0.25° 滞回）：`location_pid`（PD，Ki=0）精调
- 速度环 PI（Kp=10, Ki=0.5）跟踪 `plan_speed` 输出 PWM

默认 PID（motor_init）：
- 位置：Kp=5, Kd=0.05, Ki=0
- 速度：Kp=10, Ki=0.5, Kd=0

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
| Flash EPROM 持久化 | — | ❌ 未做（曾试后回退） |
| 校准指令 0x0B | — | ❌ 未做 |
| SYNC_READ/WRITE | — | ❌ 未做 |
| REG_WRITE + ACTION | — | ❌ 未做 |

### 5.2 帧格式

```text
FF FF | ID | LEN | Instruction + Parameters | Checksum
 2B      1B    1B         LEN-1 字节               1B
```

- Checksum = `~(ID + LEN + Inst + Params...)` 低 8 位
- 广播 ID = `0xFE`
- 已实现指令：PING(0x01)、READ(0x02)、WRITE(0x03)

### 5.3 关键寄存器映射

| 地址 | 名称 | 读写 | 桥接行为 |
|------|------|------|----------|
| 0x05 | SERVO_ID | R/W | 本机 ID |
| 0x15~0x17 | POS_P/D/I | R/W | 写后缩放更新位置 PID |
| 0x25, 0x27 | SPEED_P/I | R/W | 写后缩放更新速度 PID |
| 0x28 | TORQUE_SWITCH | R/W | 0=关断电机 |
| 0x29 | GOAL_ACC | R/W | → `target_a_speed` |
| 0x2A | GOAL_POS | R/W | → 位置模式 + `target_angle` |
| 0x2C | PWM_OPEN_SPEED | R/W | 非 0 → 力矩模式 duty |
| 0x2E | RUN_SPEED | R/W | → 速度模式 + `target_speed` |
| 0x38 | PRESENT_POS | R | 编码器多圈角编码 |
| 0x3A | PRESENT_SPEED | R | 速度绝对值 |
| 0x3C | PRESENT_LOAD | R | `encode_load(duty)` |
| 0x3E | PRESENT_VOLT | R | 母线电压 |
| 0x45 | PRESENT_CUR | R | 母线电流 |

**重要副作用修复（7/7）**：同包写 GOAL_POS + 0x2C=0 时，0x2C=0 不再误切力矩 0% 覆盖位置模式。

**速度单位（7/8 修复）**：`speed_raw_to_degs = raw × 0.087`（°/s），去掉误用 RPM×6。

**负载编码（7/8）**：`|duty| × 1000 / 3599`，负向加 `0x0400` 方向位。

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

- `DEBUG_JUSTFLOAT=1`：5 通道 JustFloat（角度/目标角/速度/plan_speed/target_speed/duty）
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
| M-05 | 位置/速度闭环 | 🔶 推进中 | 架构完成；整参见 PID 方案；**速度观测 1000Hz 量化抖动待 S-01** |
| C-01 | 通信端口 | ✅ 已完成 | PA2 USART1 半双工 1 Mbps |
| C-02 | 协议框架 | ✅ 已完成 | sts_proto 收帧/组帧/校验 |
| C-03 | STS 内存表 | ✅ 已完成 | sts_mem 0x80 + 桥接 |
| C-04 | Flash EPROM | ❌ 未开始 | 计划 @0x0800FC00 |
| C-05 | 校准/出厂 | ❌ 未开始 | OFFSET、0x0B 等 |

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

**工作区未提交**：`main.c` 中 `DEBUG_JUSTFLOAT=1`（本地 VOFA 调试）。

---

## 8. 已知问题与待办（截至 2026-07-09）

### 8.1 待验证

| 优先级 | 事项 | 说明 |
|--------|------|------|
| **P1** | **速度观测 0↔88 °/s 抖动** | 1000Hz 差分 + 12-bit 量化；方案见 `速度观测改进方案.md`，推荐 S-01+S-03 |
| P1 | 多机同总线超时 | 单机正常；多机并联易超时，discard 已合入待烧录验收 |
| P1 | 最高速度饱和 | raw=50 规划 ~220°/s，实测 ~96~98°/s；需 Watch `output`/`duty` 确认物理饱和还是控制链路 |
| P2 | 位置环实机整参 | 7/2 遗留；评估方法见 `PID稳定性评估方案.md` §3~§7 |
| P2 | PID 稳定性基线（Phase 0） | VOFA 阶跃录波 + Python 离线分析，零代码改动 |

### 8.2 未实现（已规划）

| 事项 | 参考 |
|------|------|
| 速度观测线性回归（S-01）+ 真实 Δt（S-03） | `速度观测改进方案.md` §2、§4 |
| PID 稳定性 MCU 模块（Phase 1） | `PID稳定性评估方案.md` §4，`pid_tune.c/h` |
| Flash EPROM @0x0800FC00 | xuniduoji `servo_memory.c` |
| OFFSET 校准 0x1F、0x0B 指令 | xuniduoji + 协议文档 |
| RETURN_DELAY 提权 / USART 中断优先级 | 若多机仍超时 |
| REG_WRITE + ACTION、SYNC_READ/WRITE | 协议文档后期指令 |

### 8.3 已解决的关键 Bug

| 现象 | 根因 | 修复 |
|------|------|------|
| 第二次 PING 无回包 | tx_end 死等 TC + drain RBNE | 去掉 tx_end 死等，blocked 丢回波 |
| 通信运行中断连 | I2C 读期间 `__disable_irq` | 去掉关总中断 |
| WRITE 有回包电机不动 | 0x2C=0 误切力矩 0% | 0x2C 非 0 才进扭矩模式 |
| 速度单位与上位机不符 | 误用 RPM×6 | 改为 raw×0.087 °/s |
| VOFA 角度恒为 0 | encoder flag 未按 100Hz 置位 | time.c 分频修复 |
| 速度 VOFA 锯齿 0~88 | 1000Hz 单步差分 + AS5600 1LSB | 待 S-01；见速度观测改进方案 |

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
2. 通道：multi_degree / target_angle / speed / plan_speed / target_speed / duty
3. `motor_test_poll()` 可通过 Keil Watch 改 `motor_cmd` 做本地测试

### 9.4 PID 稳定性整参（计划）

> 完整规格：[PID稳定性评估方案.md](./PID稳定性评估方案.md)

**Phase 0（当前，零代码）**：

1. 速度环：`motor_cmd_speed` + `direct`，setpoint 阶跃 0→60 °/s，VOFA 录 ≥ 8 s
2. 位置环：`motor_cmd_position` + `direct`，setpoint 阶跃 60°，录 ≥ 10 s
3. 离线算 `e_v = plan_speed - speed`、`e_θ = target_angle - angle`
4. 指标：`settle_ms`、`steady_pp`、`steady_rms`、`stable`（详见方案 §2）

**Phase 1（待实现）**：`pid_tune.c/h` → Watch `g_pid_tune_speed` / `g_pid_tune_pos`

**整参顺序**：先 **速度观测 S-01** 稳定 ch2 → 速度环单独通过 → 位置精调区 → STS 写参验证

### 9.5 速度观测（已知问题）

> 完整规格：[速度观测改进方案.md](./速度观测改进方案.md)

- 现象：ch2 `speed` 在 plan≈50 °/s 时振荡 0↔88 °/s
- 根因：12-bit 量化 + `Δθ×1000` 单步差分
- 推荐：**S-01 线性回归（N=16）+ S-03 真实 Δt**
- 临时：**S-02** 加大中值/MA 窗口（滞后大，仅验证）

---

## 10. 文档索引

| 文档 | 内容 | 新鲜度 |
|------|------|--------|
| **本文档** | 项目总览，AI 交接入口 | ✅ 2026-07-09 |
| `docs/STS完整实现指南.md` | STS 六阶段路线图 + 附录地址表 | 🔶 §0 已同步 |
| `docs/STS协议自实现指南.md` | 协议细节、帧格式、伪代码 | ✅ 参考用 |
| `docs/硬件说明-16999-PS26040802.md` | 原理图引脚、BOM、电源 | ✅ 引脚以 §8.3 为准 |
| `docs/功能实现任务拆解.md` | 全量任务表 + 详情 | 🔶 §0 进度快照已同步 |
| `docs/M-01-电机驱动实现笔记.md` | PWM / H 桥实现 | ✅ |
| `docs/M-02-M03-感知采集实现笔记.md` | ADC / AS5600 | ✅ |
| `docs/GD32F1x0-外设笔记.md` | 外设 API 笔记（2057 行） | ✅ 参考手册 |
| `docs/PID稳定性评估方案.md` | PID 整参：指标定义、测试流程、pid_tune 设计 | ✅ 2026-07-09 计划 |
| `docs/速度观测改进方案.md` | 1000Hz 速度量化噪声：S-01~S-06 路线与验收 | ✅ 2026-07-09 计划 |
| `docs/研发日报-2026-07-0*.txt` | 每日工作记录 | ✅ 7/1~7/8 |

---

## 11. 给新 AI 的建议起手式

1. **先读本文档 §5~§8**，了解 STS 协议栈现状与 open issues
2. **读 `main.c`** 理解主循环调度与 DEBUG 开关
3. **读 `sts_mem.c` 的 `sts_on_write()`**，理解寄存器→电机副作用
4. **改 UART/协议相关时**，同时看 `uart.c` 和 `bsp_i2c.c`（I2C 与 UART 并发）
5. **对照参考工程** `E:\wtzn\xuniduoji` 时，注意 xuniduoji **电机控制未接**，只借鉴协议/Flash 层
6. **引脚以 `docs/硬件说明` §8.3 为准**，不要抄例程 board_config.h
7. **整参 PID 时**，先读 `docs/PID稳定性评估方案.md`；**速度 ch2 抖动** 先读 `docs/速度观测改进方案.md`
8. **速度观测不稳定** 时优先 S-01+S-03，勿先加大 Kp

### 当前最可能的下一个任务

```
P1：速度观测 S-01 + S-03（线性回归 N=16 + g_control_tick Δt）
P1：多机 discard 验收 / ~100°/s 饱和排查
P2：PID Phase 0 基线（speed 稳定后再整参）
P2：pid_tune.c/h、Flash EPROM
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
│   ├── GD32F1x0-外设笔记.md
│   └── 研发日报-2026-07-*.txt
├── servo_project/
│   ├── project/template.uvprojx   # Keil 工程
│   └── User/
│       ├── main.c
│       ├── servo/                 # 应用层
│       │   ├── motor.c/h          # 电机控制 + 轨迹规划
│       │   ├── pid.c/h
│       │   ├── encoder.c/h        # AS5600
│       │   ├── electricity.c/h    # ADC 电流电压
│       │   ├── speed.c/h          # 速度观测器
│       │   ├── uart.c/h           # UART 环 + poll
│       │   ├── sts_proto.c/h      # STS 帧解析
│       │   ├── sts_mem.c/h        # STS 内存表
│       │   ├── sts_mem_map.h      # 地址常量
│       │   ├── motor_test.c/h     # 本地调试
│       │   ├── data_send.c/h      # VOFA+ JustFloat
│       │   └── servo_config.h     # 全局类型定义
│       └── bsp/                   # 板级驱动
│           ├── bsp_pwm.c/h
│           ├── bsp_adc.c/h
│           ├── bap_i2c.c/h
│           ├── bsp_uart.c/h
│           └── bsp_gpio.c/h
└── README.md
```

---

*本文档由 2026-07-09 工程状态自动整理，后续有重大进展时请同步更新 §5、§7、§8、§9.4、§9.5。*
