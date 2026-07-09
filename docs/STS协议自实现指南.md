# 磁编码 STS 协议自实现指南

> **完整版已合并至：[STS完整实现指南.md](./STS完整实现指南.md)**  
> **项目总览（AI 交接）：[项目总览-AI交接文档.md](./项目总览-AI交接文档.md)**  
> 本文档保留协议细节与附录；实现路线、xuniduoji 对照、分阶段步骤请以完整指南为准。

本文档供你在本工程中**从零手写** STS 串口舵机协议栈。UART 接收环、电机控制等基础设施仍保留。

**原始参考文档（建议对照阅读）：**

- `docs/磁编码STS内存表.docx` — 寄存器地址与含义
- `docs/舵机STS自定义通信协议.docx` — 帧格式与指令集

---

## 1. 目标

让 MCU 通过半双工 UART 对外表现得像一颗标准 **磁编码 STS 舵机**：

1. 解析主机发来的串口帧（PING / READ / WRITE 等）
2. 维护 0x80 字节的虚拟「内存表」
3. 把寄存器读写映射到 `motor_context` 电机控制
4. 把传感器反馈写回内存表供主机 READ

```text
上位机 ──UART──► sts_proto（帧解析）──► sts_mem（寄存器）──► motor_context
                     ▲                           │
                     └──────── 组包回复 ──────────┘
```

---

## 2. 工程里已有的基础设施

实现 STS 时**直接复用**，不必重写：

| 模块 | 路径 | 用途 |
|------|------|------|
| UART 接收环 | `User/servo/uart.c` | 中断收字节 → 256B 环形缓冲；`uart_rx_pop()` 取字节 |
| UART 发送 | `User/bsp/bsp_uart.c` | `bap_uart_send()` 阻塞发送；半双工 USART1 |
| 半双工收发切换 | `uart_comm_tx_begin/end()` | 发前关 RX 中断并清 FIFO，发完再开 |
| 电机控制 | `User/servo/motor.c` | `motor_enable/disable`、`motor_control()` |
| 控制模式 | `servo_config.h` | `motor_control_mode_*` 枚举 |
| 全局上下文 | `motor_context` | 传感器 + 控制目标 + PID |
| 轨迹规划 | `angle_control_plan_reset()` 等 | 位置/速度模式切换时调用 |
| 编码器 | `encoder.c` | AS5600 读角度；I2C 失败或磁铁丢失时返回 0 |
| 电流电压 | `electricity.c` | `motor_adc_v_bus`、`motor_adc_i_bus` |

**集成挂点（你需要改回去的地方）：**

```c
// uart.c — uart_comm_init()
bap_uart_init();
sts_proto_init();      // ← 你实现后加回来
uart_comm_rx_enable();

// uart.c — uart_comm_poll()
sts_proto_poll();      // ← 主循环每圈调用

// main.c — 主循环
uart_comm_poll();
if (!sts_mem_control_active()) {   // ← STS 接管控制时跳过本地测试
    motor_test_poll();
}
motor_control(&motor_context);

// encoder.c — 磁编异常时
sts_mem_set_magnet_ok(0);          // ← 触发 STS_BIT_MAGNET 报警
```

---

## 3. 建议文件结构

在 `User/servo/` 下新建 7 个文件：

```text
sts_mem_map.h    纯头文件：地址常量、位定义（无 .c）
sts_mem.h/c      虚拟内存表 + 与 motor 桥接
sts_proto.h/c    串口帧解析、指令分发、组包发送
sts_debug.h/c    可选：Keil Watch 调试快照
```

Keil 工程 `template.uvprojx` 里把 `.c` 文件加入 **User/servo** 组。

---

## 4. 串口帧格式（sts_proto）

### 4.1 帧结构

与 Dynamixel / 飞特 STS 兼容：

```text
┌──────┬────┬─────┬─────────────────────────┬──────────┐
│ FF FF│ ID │ LEN │ Instruction + Parameters│ Checksum │
└──────┴────┴─────┴─────────────────────────┴──────────┘
  2B    1B   1B            LEN-1 字节              1B
```

- **LEN**：从 ID 之后到 Checksum ** inclusive** 的字节数，即 `Instruction + Parameters + Checksum` 共 LEN 字节
- 等价理解：LEN = 1(Inst) + 参数长度 + 1(CS)
- **Checksum** = `~(ID + LEN + Inst + Params...)` 取低 8 位

**示例 — PING ID=1：**

```text
TX: FF FF 01 02 01 FB
         │  │  │  └─ CS = ~(0x01+0x02+0x01) = 0xFB
         │  │  └─ Inst = PING (0x01)
         │  └─ LEN = 2
         └─ ID = 1
```

**示例 — READ 地址 0x38 长度 2：**

```text
TX: FF FF 01 04 02 38 02 XX
RX: FF FF 01 04 00 [data0] [data1] CS
              │  └─ 错误字节，0=无报警
              └─ LEN=4 = Err(1)+Data(2)+CS(1)
```

### 4.2 指令码（sts_proto.h）

| 宏 | 值 | 说明 | 建议实现优先级 |
|----|-----|------|----------------|
| `STS_INST_PING` | 0x01 | 心跳，回状态 | ★ 必做 |
| `STS_INST_READ` | 0x02 | 读内存 | ★ 必做 |
| `STS_INST_WRITE` | 0x03 | 写内存 | ★ 必做 |
| `STS_INST_REG_WRITE` | 0x04 | 预写（不执行） | 可选 |
| `STS_INST_ACTION` | 0x05 | 触发 REG_WRITE | 可选 |
| `STS_INST_RECOVERY` | 0x06 | 恢复出厂 | 后期 |
| `STS_INST_REBOOT` | 0x08 | 重启 | 后期 |
| `STS_INST_BACKUP` | 0x09 | 备份 EPROM | 后期 |
| `STS_INST_RESET` | 0x0A | 复位 | 后期 |
| `STS_INST_CALIB` | 0x0B | 校准 | 后期 |
| `STS_INST_SYNC_READ` | 0x82 | 同步读 | 后期 |
| `STS_INST_SYNC_WRITE` | 0x83 | 同步写 | 后期 |

广播 ID：`0xFE`（`STS_ID_BROADCAST`）

### 4.3 接收状态机

```text
WAIT_H0 ──0xFF──► WAIT_H1 ──0xFF──► RX_ID ──► RX_LEN ──► RX_BODY ──满 LEN──► handle_frame
   ▲                  │非FF非FF              │LEN非法
   └──────────────────┴──────────────────────┘ 丢弃，回 WAIT_H0
```

伪代码：

```c
typedef enum { WAIT_H0, WAIT_H1, RX_ID, RX_LEN, RX_BODY } sts_rx_state_t;

void sts_proto_feed(uint8_t byte) {
    switch (state) {
    case WAIT_H0:  if (byte==0xFF) state=WAIT_H1; break;
    case WAIT_H1:  if (byte==0xFF) state=RX_ID;
                   else if (byte==0xFF) state=WAIT_H1;
                   else state=WAIT_H0; break;
    case RX_ID:    frame_id=byte; state=RX_LEN; break;
    case RX_LEN:   frame_len=byte; idx=0;
                   if (frame_len<2 || frame_len>=BUF_SIZE) state=WAIT_H0;
                   else state=RX_BODY; break;
    case RX_BODY:  body[idx++]=byte;
                   if (idx>=frame_len) sts_handle_frame(); break;
    }
}
```

`sts_proto_poll()`：循环 `while (uart_rx_pop(&byte)) sts_proto_feed(byte);`

### 4.4 帧处理流程（sts_handle_frame）

1. 从 `body[0..len-2]` 取 payload，末字节为 `rx_cs`
2. 计算 `calc_cs = checksum(ID, LEN, body[0..len-2])`
3. 校验失败 → 丢弃，回 WAIT_H0
4. ID 不匹配本机 ID 且非广播 → 丢弃
5. `inst = body[0]`，按指令分支
6. 处理完毕 → 回 WAIT_H0

### 4.5 ID 匹配与回复策略

```c
uint8_t id_match(uint8_t id) {
    return (id == sts_mem_get_servo_id()) || (id == 0xFE);
}

uint8_t need_response(uint8_t id, uint8_t inst) {
    if (id != 0xFE) return 1;           // 单播必回
    return (inst == STS_INST_PING);     // 广播仅 PING 回
}
```

### 4.6 发送路径

```c
static void sts_proto_send(const uint8_t *data, uint16_t len) {
    while (bap_uart_tx_dma_busy()) { }  // 若用 DMA 则等待
    uart_comm_tx_begin();               // 关 RX，清残留
    bap_uart_send((uint8_t*)data, len);
    uart_comm_tx_end();                 // 等 TC，再开 RX
}
```

**状态回复包（PING / WRITE 后）：**

```text
FF FF ID 02 Err CS
           └─ LEN=2: Err(1)+CS(1)
```

**READ 回复包：**

```text
FF FF ID (2+N) Err [N字节数据] CS
           └─ LEN = 1 + 1 + N + 1 ？ 注意：LEN 含 Err + Data + CS，不含 Instruction
           实际：plen = 2 + data_len;  // Err + Data + CS ？ 
```

对照原协议：响应帧 **没有 Instruction 字段**，结构为：

```text
FF FF ID LEN Err [Data...] CS
```

其中 `LEN = 1(Err) + data_len + 1(CS)`。

Checksum 范围：`ID + LEN + Err + Data...`

### 4.7 各指令参数格式

| 指令 | 请求 body（Inst 之后） | 响应 |
|------|------------------------|------|
| PING | 无 | 状态包，Err = 当前报警 |
| READ | `addr(1), len(1)` | 状态包 + `len` 字节数据 |
| WRITE | `addr(1), data(len-1)` | 状态包 |

READ/WRITE 的 `payload_len = frame_len - 1`（减去 Checksum）。

---

## 5. 虚拟内存表（sts_mem）

### 5.1 存储

```c
#define STS_MEM_SIZE  0x80
static uint8_t s_mem[STS_MEM_SIZE];
```

所有双字节值为 **小端**（低字节在前）。

辅助函数：

```c
void put_u16_le(uint8_t addr, int16_t val);
int16_t get_u16_le(uint8_t addr);
```

### 5.2 地址分区（sts_mem_map.h）

完整地址见 **附录 A**。分区概要：

| 区域 | 地址 | 访问 |
|------|------|------|
| 版本信息 | 0x00~0x04 | 只读 |
| EPROM 配置 | 0x05~0x27, 0x37 | 可写（掉电不保存，RAM 模拟） |
| SRAM 控制 | 0x28~0x36 | 可写，写后触发电机动作 |
| SRAM 反馈 | 0x38~0x47 | 只读（由 refresh 填充） |
| 出厂参数 | 0x50~0x56 | 只读 |

`sts_addr_writable(addr)` 逻辑：

- `addr < 0x05` → 不可写
- `0x38 <= addr < 0x50` → 不可写（反馈区）
- `0x50 <= addr < 0x57` → 不可写（出厂区）
- 其余可写

### 5.3 单位换算常量（sts_mem.h）

| 宏 | 值 | 含义 |
|----|-----|------|
| `STS_POS_UNIT_DEG` | 0.087 | 1 count = 0.087° |
| `STS_ACC_UNIT_DEGS2` | 8.7 | 加速度 count → °/s² |
| `STS_SPEED_UNIT_RPM` | 0.732 | 粗速度单位 RPM/count |
| `STS_SPEED_UNIT_RPM_FINE` | 0.0146 | 细速度（PHASE bit2=1） |
| `STS_VOLT_UNIT_V` | 0.1 | 电压 V |
| `STS_CURR_UNIT_MA` | 6.5 | 电流 mA |
| `STS_PWM_DUTY_MAX` | 3599 | 内部 PWM 满量程 |

**位置换算：**

```c
int16_t deg_to_pos(float deg) {
    float raw = (deg - pos_offset_deg) / STS_POS_UNIT_DEG;
    return clamp_i16(raw);
}
float pos_to_deg(int16_t raw) {
    return raw * STS_POS_UNIT_DEG + pos_offset_deg;
}
```

**速度换算（deg/s ↔ STS count）：**

```c
float speed_unit_rpm(void) {
    return (phase & STS_PHASE_SPEED_UNIT_FINE) ? FINE : NORMAL;
}
int16_t degs_to_speed(float deg_s) {
    float rpm = deg_s / 6.0f;
    return clamp_i16(rpm / speed_unit_rpm());
}
float speed_to_degs(int16_t raw) {
    return raw * speed_unit_rpm() * 6.0f;
}
```

**位置偏置（0x1F，2B）解码：**

```c
// raw 0~2047   → +raw * 0.087°
// raw 2048~4095 → -(raw-2048) * 0.087°
// raw 4096~6143 → +(raw-4096) * 0.087°
// raw 6144~8191 → -(raw-6144) * 0.087°
```

**负载编码（PRESENT_LOAD）：**

```c
// duty 范围 ±3599 → 0~1000，方向 bit10 (0x0400)
int16_t encode_load(int16_t duty) {
    int32_t mag = abs(duty) * 100 / 3599;
    return (duty < 0) ? (mag | 0x0400) : mag;  // 限 1023
}
```

### 5.4 初始化默认值（sts_mem_init）

建议初值（与原厂行为接近）：

```c
// 版本
FW 1.0, Model 3.10, Endian 0

// EPROM
SERVO_ID=1, BAUD=0, RESPONSE_LEVEL=1
ANGLE_MIN=0, ANGLE_MAX=4095
TEMP_MAX=70, VOLT_MAX=120(12.0V), VOLT_MIN=60(6.0V)
TORQUE_MAX=1000, PHASE=BRUSHED|HBRIDGE_INT
UNLOAD_COND=VOLTAGE|MAGNET
POS_P/D=128, POS_I=0, SPEED_P/I=128
PROTECT_CURRENT=500, EPROM_LOCK=1

// SRAM 控制
TORQUE_SWITCH=1, GOAL_POS=2048, RUN_SPEED=0, TORQUE_LIMIT=1000
```

初始化末尾调用 `sts_apply_pid_from_mem()` 同步 PID。

### 5.5 读 / 写 API

```c
uint8_t sts_mem_read(uint8_t addr, uint8_t *out, uint8_t len);
uint8_t sts_mem_write(uint8_t addr, const uint8_t *data, uint8_t len);
```

- **read**：先 `sts_mem_refresh_feedback()`，再 memcpy
- **write**：跳过不可写地址；写完后 `sts_on_write(addr, len)` 触发副作用
- 越界 `(addr+len) > 0x80` → 返回 0

### 5.6 写寄存器副作用（sts_on_write）

按写入地址范围触发（注意跨寄存器批量写）：

| 寄存器 | 地址 | 副作用 |
|--------|------|--------|
| POS_OFFSET | 0x1F | 更新 `pos_offset_deg` |
| POS_P/D/I, SPEED_P/I | 0x15~0x17, 0x25, 0x27 | 更新 `motor_pid` 增益 |
| TORQUE_SWITCH | 0x28 | 见下表 |
| RUN_MODE | 0x21 | 切换控制模式 |
| GOAL_ACC | 0x29 | 位置模式下更新加速度 |
| PWM_OPEN_SPEED | 0x2C | RUN_MODE=2 时开环 PWM |
| GOAL_POS + RUN_SPEED | 0x2A, 0x2E | 启动位置运动 |

**TORQUE_SWITCH 值：**

| 值 | 行为 |
|----|------|
| 0 | `motor_disable()`，`mode=null` |
| 1 | `motor_enable()` |
| 2 | 使能 + 速度模式，目标速度 0 |
| 128 | 使能，当前位置设为中点 2048 对应角度 |

**RUN_MODE 值：**

| 值 | 行为 |
|----|------|
| 0 | `motor_control_mode_position_speed_torque` |
| 1 | `motor_control_mode_speed_torque` |
| 2 | 开环扭矩：`PWM_OPEN_SPEED` → `target_duty` |

**GOAL_POS 运动（apply_goal_motion）：**

```c
ctrl->mode = position_speed_torque;
ctrl->target_angle = pos_to_deg(goal_pos_raw);
ctrl->target_speed = abs(speed_to_degs(run_speed_raw));
if (phase & SPEED0_IS_MAX && run_speed_raw==0) max_spd = 720°/s;
else if (run_speed_raw==0) max_spd = 0;
ctrl->target_a_speed = goal_acc * STS_ACC_UNIT_DEGS2;
if (target_a_speed <= 0) target_a_speed = 2000;
angle_control_plan_reset(ctrl, current_angle);
s_control_active = 1;
```

**PID 从寄存器映射：**

```c
pos_kp = mem[POS_P] * (5.0/128);
pos_kd = mem[POS_D] * (0.05/128);
pos_ki = mem[POS_I] * (0.5/128);
spd_kp = mem[SPEED_P] * (10.0/128);
spd_ki = mem[SPEED_I] * (0.5/128);
```

### 5.7 反馈刷新（sts_mem_refresh_feedback）

每次 READ 前、取 error 前调用。从 `motor_context` 采样：

| 寄存器 | 来源 |
|--------|------|
| PRESENT_POS 0x38 | `deg_to_pos(motor_angle_multi_degree)` |
| PRESENT_SPEED 0x3A | `degs_to_speed(motor_speed_degree)` |
| PRESENT_LOAD 0x3C | `encode_load(motor_get_duty())` |
| PRESENT_VOLT 0x3E | `v_bus / 0.1`，限 254 |
| PRESENT_TEMP 0x3F | 暂可固定 25（无温度传感器） |
| PRESENT_CUR 0x45 | `i_bus_mA / 6.5` |
| GOAL_POS_FB 0x43 | `deg_to_pos(target_angle)` |
| MOVING 0x42 | 位置误差 > `POS_PLAN_DEADZONE_DEG` 且 mode≠null |
| SERVO_STATUS 0x41 | 保护检测位 |

### 5.8 保护逻辑（sts_check_protection）

读 `UNLOAD_COND (0x13)` 掩码，置位 `SERVO_STATUS` 并可能 `motor_disable()`：

| 位 | 条件 |
|----|------|
| STS_BIT_VOLTAGE 0x01 | 电压 > VOLT_MAX 或 < VOLT_MIN |
| STS_BIT_MAGNET 0x02 | `s_magnet_ok==0`（encoder 上报） |
| STS_BIT_CURRENT 0x08 | 电流 > PROTECT_CURRENT |

### 5.9 对外辅助 API

```c
void sts_mem_init(void);
uint8_t sts_mem_get_servo_id(void);
uint8_t sts_mem_get_error(void);          // refresh 后读 SERVO_STATUS
uint8_t sts_mem_control_active(void);     // STS 是否已接管控制
void sts_mem_set_magnet_ok(uint8_t ok);   // encoder.c 调用
void sts_mem_set_reg_write_flag(uint8_t); // REG_WRITE 用，可选
```

`s_control_active`：任意一次有效运动/模式切换后置 1，供 `main.c` 决定是否跳过 `motor_test_poll()`。

---

## 6. 调试模块（sts_debug，可选）

Keil Watch 窗口观察 `g_sts_debug`，无需串口抓包。

```c
typedef struct {
    uint8_t data[64], len, id, inst;
    uint8_t checksum_ok, id_match, handled;
    uint32_t seq;
} sts_debug_frame_t;

typedef struct {
    sts_debug_frame_t rx_last, tx_last;
    uint32_t rx_total, rx_cs_fail, rx_id_skip, rx_unknown_inst;
} sts_debug_t;
```

在 proto 层校验前后调用 `sts_debug_capture_rx/tx()`。用 `#define STS_DEBUG 1` 开关。

---

## 7. 推荐实现顺序

按里程碑递进，每步都可独立验证：

### 阶段 1 — 骨架（1~2 天）

1. 建 `sts_mem_map.h`，抄附录地址常量
2. 建 `sts_mem.c`，实现 `s_mem[]`、`read/write`、越界检查
3. `sts_mem_init()` 填默认值
4. `sts_proto` 状态机 + checksum
5. 实现 PING → 回 `FF FF ID 02 Err CS`
6. `uart.c` 挂接 init/poll

**验证：** 串口助手发 `FF FF 01 02 01 FB`，应收到 6 字节回复。

### 阶段 2 — READ / WRITE（2~3 天）

1. READ：读版本区 `0x00` 长度 5
2. WRITE：写 `SERVO_ID`
3. `refresh_feedback`：至少填 PRESENT_POS
4. READ `0x38` 长度 2，角度应变化

### 阶段 3 — 电机桥接（3~5 天）

1. `TORQUE_SWITCH`、`RUN_MODE`
2. `GOAL_POS` + `RUN_SPEED` + `GOAL_ACC` → 位置控制
3. PID 映射、`POS_OFFSET`
4. 保护逻辑 + encoder 磁编报警
5. `main.c` 加 `control_active` 判断

**验证：** 上位机写 GOAL_POS，电机应转到目标。

### 阶段 4 — 完善（按需）

- REG_WRITE + ACTION
- SYNC_READ / SYNC_WRITE
- EPROM 掉电保存（Flash）
- 真实温度传感器

---

## 8. 测试用例清单

| # | 操作 | 期望 |
|---|------|------|
| 1 | PING | 回 6 字节，Err=0 |
| 2 | 错误 CS | 无回复，`rx_cs_fail++` |
| 3 | 错误 ID | 无回复 |
| 4 | READ 0x00 len=5 | 版本 1,0,0,3,10 |
| 5 | WRITE ID=2 再 READ 0x05 | ID 变为 2 |
| 6 | WRITE TORQUE_SWITCH=0 | 电机失能 |
| 7 | WRITE GOAL_POS=2048+offset | 电机转约 90° |
| 8 | 拔掉磁编 | PING Err 含 MAGNET 位，电机失能 |
| 9 | 广播 PING 0xFE | 有回复 |
| 10 | 广播 WRITE 0xFE | 无回复 |

**常用 READ 当前位置：**

```text
FF FF 01 04 02 38 02 C1
```

**常用 WRITE 目标位置 2048：**

```text
FF FF 01 05 03 2A 00 08 F7
```

---

## 9. 常见坑

1. **LEN 含义**：含 Checksum，不是「参数长度」
2. **响应帧无 Instruction**：只有 Err + Data + CS
3. **Checksum 范围**：从 ID 开始，不是从 FF FF 开始
4. **半双工**：发送时必须 `tx_begin/end`，否则自己的 TX 会进 RX 环
5. **小端**：GOAL_POS=2048 → 写入 `00 08`
6. **READ 前 refresh**：否则反馈是旧值
7. **批量 WRITE**：`sts_on_write` 要按 `[addr, addr+len)` 区间判断触发了哪些寄存器
8. **motor_test 冲突**：STS 控制期间必须跳过 `motor_test_poll()`

---

## 附录 A — 完整地址表

```c
#define STS_MEM_SIZE                0x80U

/* 2.1 版本 0x00~0x04 */
#define STS_ADDR_FW_MAJOR           0x00U   /* RO */
#define STS_ADDR_FW_MINOR           0x01U
#define STS_ADDR_ENDIAN             0x02U
#define STS_ADDR_MODEL_MAJOR        0x03U
#define STS_ADDR_MODEL_MINOR        0x04U

/* 2.2 EPROM 0x05~0x27 */
#define STS_ADDR_SERVO_ID           0x05U
#define STS_ADDR_BAUD_RATE          0x06U
#define STS_ADDR_RESPONSE_DELAY     0x07U
#define STS_ADDR_RESPONSE_LEVEL     0x08U
#define STS_ADDR_ANGLE_MIN          0x09U   /* 2B */
#define STS_ADDR_ANGLE_MAX          0x0BU   /* 2B */
#define STS_ADDR_TEMP_MAX           0x0DU
#define STS_ADDR_VOLT_MAX           0x0EU
#define STS_ADDR_VOLT_MIN           0x0FU
#define STS_ADDR_TORQUE_MAX         0x10U   /* 2B */
#define STS_ADDR_PHASE              0x12U
#define STS_ADDR_UNLOAD_COND        0x13U
#define STS_ADDR_LED_ALARM          0x14U
#define STS_ADDR_POS_P              0x15U
#define STS_ADDR_POS_D              0x16U
#define STS_ADDR_POS_I              0x17U
#define STS_ADDR_MIN_START_FORCE    0x18U
#define STS_ADDR_INTEGRAL_LIMIT     0x19U
#define STS_ADDR_DEADZONE_FWD       0x1AU
#define STS_ADDR_DEADZONE_REV       0x1BU
#define STS_ADDR_PROTECT_CURRENT    0x1CU   /* 2B */
#define STS_ADDR_ANGLE_RESOLUTION   0x1EU
#define STS_ADDR_POS_OFFSET         0x1FU   /* 2B */
#define STS_ADDR_RUN_MODE           0x21U
#define STS_ADDR_HOLD_TORQUE        0x22U
#define STS_ADDR_PROTECT_TIME       0x23U
#define STS_ADDR_OVERLOAD_TORQUE    0x24U
#define STS_ADDR_SPEED_P            0x25U
#define STS_ADDR_OC_PROTECT_TIME    0x26U
#define STS_ADDR_SPEED_I            0x27U

/* 2.3 SRAM 控制 0x28~0x37 */
#define STS_ADDR_TORQUE_SWITCH      0x28U
#define STS_ADDR_GOAL_ACC           0x29U
#define STS_ADDR_GOAL_POS           0x2AU   /* 2B */
#define STS_ADDR_PWM_OPEN_SPEED     0x2CU   /* 2B */
#define STS_ADDR_RUN_SPEED          0x2EU   /* 2B */
#define STS_ADDR_TORQUE_LIMIT       0x30U   /* 2B */
#define STS_ADDR_EPROM_LOCK         0x37U

/* 2.4 SRAM 反馈 0x38~0x47 */
#define STS_ADDR_PRESENT_POS        0x38U   /* 2B */
#define STS_ADDR_PRESENT_SPEED      0x3AU   /* 2B */
#define STS_ADDR_PRESENT_LOAD       0x3CU   /* 2B */
#define STS_ADDR_PRESENT_VOLT       0x3EU
#define STS_ADDR_PRESENT_TEMP       0x3FU
#define STS_ADDR_REG_WRITE_FLAG     0x40U
#define STS_ADDR_SERVO_STATUS       0x41U
#define STS_ADDR_MOVING             0x42U
#define STS_ADDR_GOAL_POS_FB        0x43U   /* 2B */
#define STS_ADDR_PRESENT_CUR        0x45U   /* 2B */

/* 2.5 出厂 0x50~0x56 */
#define STS_ADDR_FACTORY_BASE       0x50U
#define STS_ADDR_FACTORY_END        0x57U

/* PHASE 0x12 位 */
#define STS_PHASE_DRV_DIR           0x01U
#define STS_PHASE_BRUSHED           0x02U
#define STS_PHASE_SPEED_UNIT_FINE   0x04U
#define STS_PHASE_SPEED0_IS_MAX     0x08U
#define STS_PHASE_ANGLE_MULTI       0x10U
#define STS_PHASE_HBRIDGE_INT       0x20U
#define STS_PHASE_PWM_16K           0x40U
#define STS_PHASE_FB_DIR            0x80U

/* 报警位 */
#define STS_BIT_VOLTAGE             0x01U
#define STS_BIT_MAGNET              0x02U
#define STS_BIT_TEMP                0x04U
#define STS_BIT_CURRENT             0x08U
#define STS_BIT_LOAD                0x20U
```

---

## 附录 B — 头文件 API 汇总

### sts_proto.h

```c
#define STS_HEADER_0        0xFFU
#define STS_HEADER_1        0xFFU
#define STS_FRAME_BUF_SIZE  128U
/* INST 宏 ... */
void sts_proto_init(void);
void sts_proto_poll(void);
```

### sts_mem.h

```c
#define STS_ID_BROADCAST    0xFEU
/* 单位宏 ... */
void sts_mem_init(void);
uint8_t sts_mem_get_servo_id(void);
uint8_t sts_mem_read(uint8_t addr, uint8_t *out, uint8_t len);
uint8_t sts_mem_write(uint8_t addr, const uint8_t *data, uint8_t len);
void sts_mem_refresh_feedback(void);
uint8_t sts_mem_get_error(void);
uint8_t sts_mem_control_active(void);
void sts_mem_set_magnet_ok(uint8_t ok);
void sts_mem_set_reg_write_flag(uint8_t flag);
```

---

*文档版本：2026-07-07，对应移除 STS 参考实现后的工程状态。*
