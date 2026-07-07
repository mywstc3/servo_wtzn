# STS 协议完整实现指南

本文档是 **servo 工程** 手写磁编码 STS 舵机协议栈的完整路线图，供你按阶段自行实现。

**配套资料：**

| 资料 | 路径 |
|------|------|
| 寄存器定义 | `docs/磁编码STS内存表.docx` |
| 帧格式与指令 | `docs/舵机STS自定义通信协议.docx` |
| 参考工程（已实现协议+内存+Flash） | `E:/wtzn/xuniduoji/xuniduoji/User/` |
| 本工程外设笔记 | `docs/GD32F1x0-外设笔记.md` |

---

## 0. 当前工程进度（截至 2026-07-07）

| 模块 | 文件 | 状态 |
|------|------|------|
| UART 字节环 | `uart.c` | ✅ 已完成 |
| BSP 半双工发送 | `bsp_uart.c` | ✅ 已完成 |
| 协议组帧 + 帧环 | `sts_proto.c` | 🔶 收帧完成，PING 回复完成 |
| 内存表 + 电机桥接 | `sts_mem.c` | ❌ 待实现 |
| Flash EPROM 持久化 | — | ❌ 待实现（可参考 xuniduoji） |
| main 集成 | `main.c` | 🔶 缺 `sts_mem_control_active` 判断 |

**你下一步：** 实现 `sts_mem`，并在 `sts_proto` 的 READ/WRITE 分支里调用它。

---

## 1. 总体目标

让 GD32 通过半双工 UART 对外表现得像一颗标准 STS 舵机：

1. 解析上位机帧（PING / READ / WRITE …）
2. 维护 0x80 字节虚拟内存表
3. 写控制寄存器 → 驱动 `motor_context` 电机
4. 读反馈寄存器 → 返回编码器/ADC 实时数据
5. EPROM 配置掉电保存（可选后期）

```text
上位机
  │  FF FF ID LEN ...
  ▼
uart.c          字节环 g_uart_rx_ring
  │ uart_rx_pop
  ▼
sts_proto.c     组帧 → 帧环 g_sts_rx_frame[3] → 指令分发
  │ sts_mem_read / write
  ▼
sts_mem.c       memory[0x80] 统一地址空间
  │                    ┌── Flash（EPROM 区）
  │                    ├── RAM（控制区 + 副作用）
  │                    └── 传感器刷新（反馈区）
  ▼
motor.c         PID + 轨迹规划 + PWM
```

---

## 2. 参考工程 xuniduoji 对照

xuniduoji 已实现协议和内存层，**电机控制未接**，但分层思路可直接借鉴。

| xuniduoji | servo 工程 | 职责 |
|-----------|-----------|------|
| `uart1.c` | `uart.c` + `bsp_uart.c` | 字节环、半双工 |
| `protocol.c` | `sts_proto.c` | 帧解析、指令、回复 |
| `servo_memory.c` | `sts_mem.c`（待写） | 内存表、Flash、反馈刷新 |
| `Flash_IAP.c` | 可选 | Bootloader 升级（与 EPROM 不同用途） |
| — | `motor.c` | 电机控制（servo 独有） |

### xuniduoji 核心思路（务必理解）

1. **协议层不认 Flash/RAM**，只调 `servo_memory_read_block` / `write_block`
2. **一张 `memory[128]` RAM 镜像** 对应整张 STS 表
3. **EPROM 区 0x05~0x27** 启动时从 Flash 加载，解锁时写入 Flash
4. **反馈区 0x38~** 由 `servo_update_present_position()` 周期刷新，READ 前也可刷新
5. **WRITE 到 GOAL_POS** 应触发电机（xuniduoji 留空，你在 servo 里要补）

### Flash 地址规划（xuniduoji 实例）

| Flash 地址 | 用途 |
|------------|------|
| `0x08004000` | APP 程序（IAP） |
| `0x0800F800` | 升级标志 `0x55AA55AA` |
| `0x0800FC00` | EPROM 参数页（魔数 `0x55AA` + 0x05~0x27 数据） |

**EPROM 保存与 IAP 升级是两块独立 Flash 区域，不要混用。**

---

## 3. 分层架构详解

### 3.1 第一层：UART（硬件 + 字节环）

**文件：** `bsp_uart.c`（驱动）、`uart.c`（通信策略）

```text
中断 USART1_IRQHandler
  → ring_put → g_uart_rx_ring.buf[head]

主循环 uart_comm_poll → sts_proto_poll
  → uart_rx_pop → 取 g_uart_rx_ring.buf[tail]
```

**发送固定三步：**

```c
uart_comm_tx_begin();       // 关 RX 中断，清硬件 FIFO
bap_uart_send(data, len);   // 阻塞发字节
uart_comm_tx_end();         // 等 TC，再开 RX
```

或封装好的 `uart_comm_tx(data, len)`（内部已包含三步）。

**Keil Watch：** `g_uart_rx_ring.head / tail / buf`

---

### 3.2 第二层：sts_proto（协议 + 双环）

**文件：** `sts_proto.c` / `sts_proto.h`

#### 双环结构

```text
字节流 ──► g_sts_rx_assemble（组帧状态机）
              │ 收满 + 校验通过
              ▼
         g_sts_rx_frame[3]（帧环，head/tail/count）
              │ sts_proto_rx_frame_pop
              ▼
         sts_proto_process_one_frame（PING/READ/WRITE）
```

- **字节环**（uart）：存原始字节，与帧格式无关
- **组帧缓冲**（assemble）：状态机正在拼的一帧
- **帧环**（3 槽）：已完成的帧排队等待处理；满则丢弃新帧

#### 请求帧 vs 响应帧

| | 请求（上位机→舵机） | 响应（舵机→上位机） |
|--|---------------------|---------------------|
| 结构 | `FF FF ID LEN Inst Params CS` | `FF FF ID LEN Err [Data] CS` |
| LEN 含义 | Inst + Params + CS | Err + Data + CS |
| 无 Instruction | — | 响应没有 Inst 字段 |

#### Checksum

```c
// 请求：校验 ID + LEN + body(含 Inst，不含 CS)
// 响应：校验 ID + LEN + Err + Data（不含 CS）
uint8_t cs = ~(sum & 0xFF);
```

#### 指令参数

| 指令 | Inst | 参数（Inst 之后） |
|------|------|-------------------|
| PING | 0x01 | 无 |
| READ | 0x02 | `addr(1), len(1)` |
| WRITE | 0x03 | `addr(1), data...` |

帧内布局：`frame->data[0]` = Inst，`data[1]` = addr，WRITE 的数据从 `data[2]` 起。

#### ID 与广播

```c
#define STS_ID_BROADCAST  0xFEU

// 匹配本机 ID 或广播
uint8_t id_ok = (id == sts_mem_get_servo_id()) || (id == STS_ID_BROADCAST);

// 广播：仅 PING 回复
uint8_t need_rsp = (id != STS_ID_BROADCAST) || (inst == STS_INST_PING);
```

#### 发送辅助函数（建议在 sts_proto.c 内 static）

```c
static void sts_send_status(uint8_t id, uint8_t err)
{
    uint8_t tx[6];
    uint8_t plen = 2U;
    tx[0] = 0xFF; tx[1] = 0xFF;
    tx[2] = id; tx[3] = plen; tx[4] = err;
    tx[5] = sts_checksum(&tx[2], 1U + plen);
    uart_comm_tx(tx, 6U);
}

static void sts_send_read_response(uint8_t id, uint8_t err,
    const uint8_t *data, uint8_t data_len)
{
    uint8_t tx[STS_FRAME_DATA_MAX + 6U];
    uint8_t plen = (uint8_t)(2U + data_len);
    // FF FF id plen err [data...] cs
    ...
    uart_comm_tx(tx, (uint16_t)(4U + plen));
}
```

#### process_one_frame 模板

```c
static void sts_proto_process_one_frame(const sts_frame_t *frame)
{
    uint8_t inst, err = 0U;

    if (!frame->checksum_ok) return;
    if (!id_match(frame->id)) return;

    inst = frame->data[0];
    switch (inst) {
    case STS_INST_PING:
        if (need_response(frame->id, inst))
            sts_send_status(frame->id, sts_mem_get_error());
        break;

    case STS_INST_READ:
    {
        uint8_t addr = frame->data[1];
        uint8_t rlen = frame->data[2];
        uint8_t buf[STS_FRAME_DATA_MAX];
        if (sts_mem_read(addr, buf, rlen) == rlen) {
            err = sts_mem_get_error();
            if (need_response(frame->id, inst))
                sts_send_read_response(frame->id, err, buf, rlen);
        }
        break;
    }

    case STS_INST_WRITE:
    {
        uint8_t addr = frame->data[1];
        uint8_t wlen = (uint8_t)(frame->length - 3U);
        if (wlen > 0U && sts_mem_write(addr, &frame->data[2], wlen) == wlen) {
            err = sts_mem_get_error();
            if (need_response(frame->id, inst))
                sts_send_status(frame->id, err);
        }
        break;
    }
    default:
        break;
    }
}
```

**注意：** `case` 内声明变量必须用 `{ }` 包裹（C99 要求）。

**改进点（相对当前代码）：**

- 去掉硬编码 `if (frame->id == 0x01)`，改用 `sts_mem_get_servo_id()` + 广播判断
- READ/WRITE 调用 `sts_mem_*`，不要直接操作硬件

---

### 3.3 第三层：sts_mem（内存表，核心）

**文件：** `sts_mem_map.h`、`sts_mem.h`、`sts_mem.c`

#### 设计原则

> 协议层只认地址；Flash / 传感器 / 电机都在 sts_mem 内部按地址分区处理。

#### 一张 RAM 镜像

```c
#define STS_MEM_SIZE  0x80U
static uint8_t s_mem[STS_MEM_SIZE];
```

#### 地址分区与读写规则

| 区域 | 地址 | 读 | 写 | 物理实现 |
|------|------|----|----|----------|
| 版本 | 0x00~0x04 | ✅ | ❌ | `s_mem` 常量 |
| EPROM 配置 | 0x05~0x27 | ✅ | ✅（看锁） | `s_mem` + Flash 镜像 |
| SRAM 控制 | 0x28~0x36, 0x37 | ✅ | ✅ | `s_mem` + 写副作用 |
| SRAM 反馈 | 0x38~0x47 | ✅ | ❌ | READ 前 `refresh` 填入 |
| 保留 | 0x48~0x4F | — | — | — |
| 出厂参数 | 0x50~0x56 | ✅ | ❌ | `s_mem` 常量 |

```c
static uint8_t sts_addr_writable(uint8_t addr)
{
    if (addr < STS_ADDR_SERVO_ID) return 0U;
    if (addr >= STS_ADDR_PRESENT_POS && addr < STS_ADDR_FACTORY_BASE) return 0U;
    if (addr >= STS_ADDR_FACTORY_BASE && addr < STS_ADDR_FACTORY_END) return 0U;
    return 1U;
}
```

#### 对外 API

```c
void sts_mem_init(void);
uint8_t sts_mem_read(uint8_t addr, uint8_t *out, uint8_t len);
uint8_t sts_mem_write(uint8_t addr, const uint8_t *data, uint8_t len);
void sts_mem_refresh_feedback(void);
uint8_t sts_mem_get_error(void);
uint8_t sts_mem_get_servo_id(void);
uint8_t sts_mem_control_active(void);
void sts_mem_set_magnet_ok(uint8_t ok);
```

#### 读路径

```c
uint8_t sts_mem_read(uint8_t addr, uint8_t *out, uint8_t len)
{
    if (out == NULL || len == 0U) return 0U;
    if ((uint16_t)addr + len > STS_MEM_SIZE) return 0U;

    sts_mem_refresh_feedback();   // 先刷新 0x38~ 反馈区

    for (i = 0; i < len; i++)
        out[i] = s_mem[addr + i];
    return len;
}
```

#### 写路径

```c
uint8_t sts_mem_write(uint8_t addr, const uint8_t *data, uint8_t len)
{
    // 1. 越界检查
    // 2. 逐字节写，跳过不可写地址
    // 3. sts_on_write(addr, len) 副作用
    // 4. EPROM 区且 EPROM_LOCK==0 → eeprom_save()
    return len;
}
```

#### 双字节小端

```c
static void put_u16_le(uint8_t addr, int16_t val);
static int16_t get_u16_le(uint8_t addr);
```

GOAL_POS = 2048 → 内存中为 `s_mem[0x2A]=0x00, s_mem[0x2B]=0x08`

#### 单位换算（写入 motor 前 / 从 sensor 读出后）

| 宏 | 值 | 用途 |
|----|-----|------|
| `STS_POS_UNIT_DEG` | 0.087 | 位置 count ↔ ° |
| `STS_ACC_UNIT_DEGS2` | 8.7 | 加速度 |
| `STS_SPEED_UNIT_RPM` | 0.732 | 速度（粗） |
| `STS_SPEED_UNIT_RPM_FINE` | 0.0146 | 速度（细，看 PHASE bit2） |
| `STS_VOLT_UNIT_V` | 0.1 | 电压 |
| `STS_CURR_UNIT_MA` | 6.5 | 电流 |

#### 写寄存器副作用（sts_on_write）

按写入区间 `[addr, addr+len)` 判断（支持批量写）：

| 寄存器 | 地址 | 副作用 |
|--------|------|--------|
| POS_OFFSET | 0x1F | 更新角度零点 |
| POS_P/D/I, SPEED_P/I | 0x15~0x17, 0x25, 0x27 | 更新 `motor_pid` |
| TORQUE_SWITCH | 0x28 | 0=失能；1=使能；2=速度0；128=校准中点 |
| RUN_MODE | 0x21 | 0=位置；1=速度；2=开环PWM |
| GOAL_ACC | 0x29 | 更新加速度 |
| GOAL_POS / RUN_SPEED | 0x2A, 0x2E | 启动位置/速度运动 |
| PWM_OPEN_SPEED | 0x2C | RUN_MODE=2 时开环占空比 |

实现后设 `s_control_active = 1`，供 main 跳过 `motor_test_poll()`。

#### 反馈刷新（sts_mem_refresh_feedback）

从 `motor_context` 采样写入 `s_mem`：

| 寄存器 | 来源 |
|--------|------|
| PRESENT_POS 0x38 | `motor_angle_multi_degree` → deg_to_pos |
| PRESENT_SPEED 0x3A | `motor_speed_degree` → degs_to_speed |
| PRESENT_LOAD 0x3C | `motor_get_duty()` 编码 |
| PRESENT_VOLT 0x3E | `motor_adc_v_bus` |
| PRESENT_CUR 0x45 | `motor_adc_i_bus` |
| GOAL_POS_FB 0x43 | `target_angle` |
| MOVING 0x42 | 位置误差 > 死区 |
| SERVO_STATUS 0x41 | 过压/欠压/过流/磁编丢失 |

`encoder.c` 磁编异常时调用 `sts_mem_set_magnet_ok(0)`。

---

### 3.4 第四层：Flash EPROM 持久化（后期）

参考 xuniduoji `servo_memory.c` 的 `eeprom_load` / `eeprom_save`：

```text
Flash 页 0x0800FC00:
  [0] = 0x55
  [1] = 0xAA
  [2..] = s_mem[0x05 .. 0x27]
```

**启动：**

```c
void sts_mem_init(void)
{
    memset(s_mem, 0, sizeof(s_mem));
    eeprom_load();
    // 拷贝到 s_mem[0x05~0x27]
    // 若无效 → 填 default_eprom[] → eeprom_save()
    // 填版本区、出厂区、SRAM 控制默认值
    sts_apply_pid_from_mem();
}
```

**写入 EPROM 且 EPROM_LOCK(0x37)==0：**

```c
eeprom_save();  // 擦页 + 字写入 Flash
```

**锁：** `STS_ADDR_EPROM_LOCK == 1` 时拒绝写 0x05~0x27（xuniduoji 同）。

---

### 3.5 主循环集成

```c
// Init.c
uart_comm_init();   // 内含 sts_proto_init() → 应再调 sts_mem_init()

// main.c
while (1) {
    // 编码器 / ADC / 速度（已有）
    uart_comm_poll();                    // sts_proto_poll
    if (!sts_mem_control_active())
        motor_test_poll();               // STS 接管时跳过
    motor_control(&motor_context);
}
```

反馈刷新时机（二选一或都用）：

- **方案 A：** 每次 `sts_mem_read` 前 refresh（简单）
- **方案 B：** main 每 20~50ms 调 `sts_mem_refresh_feedback()`（xuniduoji 做法，减轻 READ 延迟）

---

## 4. 建议文件清单

```text
User/servo/
  sts_mem_map.h      地址宏、位定义
  sts_mem.h          对外 API、单位宏
  sts_mem.c          内存表 + Flash + 电机桥接
  sts_proto.h        已有，补充发送函数声明（可选）
  sts_proto.c        已有，补 READ/WRITE + ID 判断
  sts_debug.h/c      可选，Keil 调试
  uart.c             已有
```

Keil `template.uvprojx` 加入 `sts_mem.c`。

---

## 5. 分阶段实现步骤（按顺序做）

### 阶段 1：验证收发包（你已基本完成）

- [x] `uart.c` 字节环 + `g_uart_rx_ring`
- [x] `sts_proto` 组帧 + 帧环
- [x] PING 回复 `FF FF ID 02 00 CS`
- [ ] ID 匹配改用 `sts_mem_get_servo_id()`（需先 stub 返回 1）
- [ ] 广播 PING 0xFE 也能回

**测试：** 串口发 `FF FF 01 02 01 FB`，应回 6 字节。

---

### 阶段 2：sts_mem 骨架（RAM 版，无 Flash）

1. 新建 `sts_mem_map.h`（附录 A 地址表）
2. 新建 `sts_mem.c`，实现：
   - `s_mem[0x80]`
   - `sts_mem_init()` 填默认值
   - `sts_addr_writable`
   - `sts_mem_read` / `sts_mem_write`（无 Flash、无副作用）
   - `sts_mem_get_servo_id()` 读 `s_mem[0x05]`
3. `sts_proto_init()` 里调 `sts_mem_init()`
4. 实现 READ：读 `0x00` len=5 得版本号
5. 实现 WRITE：写 `0x05` 改 ID，再 READ 验证

**测试：**

```text
READ 版本:  FF FF 01 04 02 00 05 F8
WRITE ID=2: FF FF 01 04 03 05 02 F6
```

---

### 阶段 3：反馈区

1. 实现 `sts_mem_refresh_feedback()`
2. 至少填 PRESENT_POS、PRESENT_VOLT
3. READ `0x38` len=2，转动电机角度应变化

**测试：** `FF FF 01 04 02 38 02 C1`

---

### 阶段 4：电机桥接

1. 实现 `sts_on_write()` 各分支
2. TORQUE_SWITCH → `motor_enable/disable`
3. GOAL_POS + RUN_SPEED + GOAL_ACC → 位置模式
4. PID 寄存器 → `motor_pid`
5. `sts_mem_control_active()` + main 里跳过 motor_test
6. `encoder.c` → `sts_mem_set_magnet_ok`
7. 保护逻辑 → SERVO_STATUS + 必要时 `motor_disable`

**测试：** WRITE GOAL_POS，电机应转动。

```text
FF FF 01 05 03 2A 00 08 F7
```

---

### 阶段 5：Flash 持久化

1. 选 Flash 页地址（如 `0x0800FC00`，避开程序区）
2. 实现 `eeprom_load` / `eeprom_save`（照抄 xuniduoji 逻辑）
3. EPROM_LOCK 写保护
4. 断电重启后 ID/PID 等应保持

---

### 阶段 6：扩展指令（可选）

参考 xuniduoji `protocol.c`：

| 指令 | 码 | 说明 |
|------|-----|------|
| REG_WRITE | 0x04 | 预写不执行 |
| ACTION | 0x05 | 触发 REG_WRITE |
| RESTORE | 0x06 | 恢复出厂（除 ID） |
| REBOOT | 0x08 | 软复位 |
| JUMP_BOOT | 0x09 | 跳 Bootloader |
| CALIB | 0x0B | 校准偏移 |
| SYNC_READ | 0x82 | 同步读 |
| SYNC_WRITE | 0x83 | 同步写 |

---

## 6. 数据流走查

### READ 当前位置（0x38, 2 字节）

```text
1. 上位机发: FF FF 01 04 02 38 02 C1
2. uart 中断 → g_uart_rx_ring
3. sts_proto_poll → 组帧 → 帧环
4. process: READ, addr=0x38, len=2
5. sts_mem_read(0x38, buf, 2)
     → refresh_feedback()  // 编码器 → s_mem[0x38,0x39]
     → 拷贝 buf
6. sts_send_read_response(id, err, buf, 2)
7. uart_comm_tx → 上位机收到位置数据
```

### WRITE 目标位置（0x2A, 2048）

```text
1. 上位机发: FF FF 01 05 03 2A 00 08 F7
2. process: WRITE, addr=0x2A, data=[00,08]
3. sts_mem_write(0x2A, data, 2)
     → s_mem[0x2A]=0x00, s_mem[0x2B]=0x08
     → sts_on_write → apply_goal_motion()
         → target_angle = pos_to_deg(2048)
         → angle_control_plan_reset()
         → s_control_active = 1
4. sts_send_status(id, err)
5. main 里 motor_control() 驱动电机到位
```

---

## 7. 测试命令速查

| 操作 | Hex（ID=1） |
|------|-------------|
| PING | `FF FF 01 02 01 FB` |
| READ 版本 5B | `FF FF 01 04 02 00 05 F8` |
| READ 位置 2B | `FF FF 01 04 02 38 02 C1` |
| WRITE 目标 2048 | `FF FF 01 05 03 2A 00 08 F7` |
| WRITE 扭矩关 | `FF FF 01 04 03 28 00 F9` |

波特率：**1 Mbps**（`bap_uart.h` 中 `COMM_BAUDRATE`）。

---

## 8. 常见错误

| 现象 | 原因 |
|------|------|
| 收不到回复 | 波特率不对；未 `uart_comm_tx`；ID 不匹配 |
| PING 回包格式错 | 响应帧无 Inst，应是 `Err` 不是 `0x01` |
| READ 位置不变 | 未 `refresh_feedback` |
| WRITE 无动作 | 未实现 `sts_on_write`；TORQUE_SWITCH=0 |
| 自己发的进 RX 环 | 未 `tx_begin/end` |
| case 里编译警告 | 变量声明需加 `{ }` |
| GOAL_POS 角度不对 | 小端序；未处理 POS_OFFSET |
| motor_test 抢控制 | 未判断 `sts_mem_control_active()` |

---

## 9. Keil 调试 Watch 推荐

```text
g_uart_rx_ring.head / tail
g_sts_rx_assemble.frame_state / id / length
g_sts_rx_frame_count
g_sts_rx_frame[0].data
motor_context.control.target_angle
motor_context.sensor.motor_angle_multi_degree
```

---

## 附录 A — 完整地址表

见 `STS协议自实现指南.md` 附录 A，或 `docs/磁编码STS内存表.docx`。

## 附录 B — sts_mem 初始化默认值参考

```c
// 版本 0x00~0x04
FW 1.0, Endian 0, Model 3.10

// EPROM
SERVO_ID=1, BAUD=0, RESPONSE_LEVEL=1
ANGLE_MIN=0, ANGLE_MAX=4095
VOLT_MAX=120(12.0V), VOLT_MIN=60(6.0V)
PHASE=BRUSHED|HBRIDGE_INT, UNLOAD=VOLTAGE|MAGNET
POS_P/D=128, POS_I=0, SPEED_P/I=128
PROTECT_CURRENT=500, EPROM_LOCK=1

// SRAM 控制
TORQUE_SWITCH=1, GOAL_POS=2048, RUN_SPEED=0, TORQUE_LIMIT=1000
```

## 附录 C — xuniduoji 关键文件阅读顺序

1. `User/servo_memory.h` — 地址定义
2. `User/servo_memory.c` — 内存表 + Flash + 反馈
3. `User/protocol.c` — 协议状态机 + READ/WRITE 分发
4. `User/uart1.c` — 字节环 + 半双工
5. `User/main.c` — 主循环拼装
6. `Bootloader/Flash_IAP.c` — IAP（与 EPROM 分开）

---

*文档版本：2026-07-07 v2 — 含 xuniduoji 参考与当前 sts_proto 进度。*
