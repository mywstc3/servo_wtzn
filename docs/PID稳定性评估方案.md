# PID 稳定性评估方案 — 计划与实现指南

> **用途**：为位置/速度串级 PID 整参提供**可量化、可复现**的稳定性评估方法；本文档为**设计规格**，固件尚未实现。  
> **关联任务**：M-05 电机速度位置闭环 · 实机整参  
> **编写日期**：2026-07-09  
> **状态**：📋 计划已定，代码未改

---

## 0. 背景与目标

### 0.1 问题

当前调试手段以 **Keil Watch 瞬时变量** 和 **VOFA+ 波形** 为主（`error`、`duty`、`plan_speed` 等），只能回答「此刻偏多少」，无法系统回答：

- 阶跃后能否**稳定停住**（不持续振荡）？
- 稳态抖动幅度有多大（limit cycle）？
- 两组 PID 参数谁**更稳**（可对比的标量）？

### 0.2 目标

建立一套 **分环、分阶段** 的稳定性评估体系：

| 层次 | 内容 | 代码改动 |
|------|------|----------|
| **Phase 0** | 标准测试流程 + VOFA 录波 + 离线 Python 分析 | 无 |
| **Phase 1** | MCU `pid_tune` 模块：稳定判定 + 稳态统计 + Watch 变量 | 新增 `pid_tune.c/h` |
| **Phase 2** | VOFA 扩展通道 + 可选自动阶跃触发 | 改 `main.c` / `motor_test.c` |
| **Phase 3**（可选） | 正弦扫频评稳定裕度 | 利用现有 `motor_wave_sine` |

### 0.3 不在本文范围

- 自动寻优 PID（Ziegler-Nichols 等）— 远期
- STS 寄存器暴露稳定性指标 — 需协议扩展，暂不规划
- 电流环稳定性 — M-04 未开始

---

## 1. 控制架构与评估边界

### 1.1 串级结构（现状）

```text
位置模式 (motor_control_mode_position_speed_torque)
  │
  ├─ |e_θ| > 1°  ──→ angle_control_plan() ──→ plan_speed
  │
  └─ |e_θ| ≤ 1°  ──→ location_pid() (PD, Ki=0) ──→ plan_speed
                                                    │
                                                    ▼
                              speed_pid() (PI) ──→ duty ──→ H 桥
```

| 环 | 函数 | 更新率 | 默认参数 | 限幅 |
|----|------|--------|----------|------|
| 位置 PD | `location_pid()` | 250 Hz | Kp=5, Kd=0.05, Ki=0 | output ±60 °/s |
| 速度 PI | `speed_pid()` | 500 Hz | Kp=10, Ki=0.5, Kd=0 | output ±3500 duty |

关键常量（`servo_config.h`）：

- `POS_PLAN_DEADZONE_DEG = 1.0°` — 进入精调区阈值
- `POS_STOP_HYSTERESIS_DEG = 0.25°` — 退出精调区滞回
- `POS_TRIM_SPEED_MAX = 60.0 °/s` — 精调区 PD 最大速度输出

### 1.2 采样率与评估频率

| 信号 | 实际更新率 | 说明 |
|------|------------|------|
| 编码器 / 多圈角 | 1000 Hz | `ENCODER_UPDATE_HZ` |
| 速度观测 | 1000 Hz | **工作区**：MA(4)→跨距差分 SPAN=16 + 真实 Δt→LPF(α=0.3)；**VOFA 验收待做**。旁支另有 LS(S-01)。见 [速度观测改进方案.md](./速度观测改进方案.md) |
| 速度 PID | 500 Hz | `motor_speed_pid_flag` |
| 位置 PID | 250 Hz | `motor_location_pid_flag` |
| VOFA JustFloat | 1000 Hz | 与 `motor_speed_flag` 同步 |

**结论**：

- **速度 ch2 未稳定前不宜做 IAE 基线**：先完成速度观测 VOFA 验收（工作区跨距差分或旁支 LS）。
- **稳态稳定性**（0.5~5 Hz 振荡）用 1000 Hz VOFA 录波足够。
- **MCU 内统计**应在各 PID 实际执行频率采样（500 / 250 Hz），而非仅在 VOFA 点采样。

### 1.3 评估对象划分

| 评估对象 | 何时有效 | 误差定义 |
|----------|----------|----------|
| **速度环** | `motor_cmd_speed` 或位置模式下任意时刻 | `e_v = plan_speed - motor_speed_degree` |
| **位置环（精调区）** | 位置模式且 `in_hold == 1` | `e_θ = target_angle - motor_angle_multi_degree` |
| **饱和监测** | 全程 | `|duty| ≥ DUTY_SAT_THRESH`（建议 3400） |

大行程段（\|e_θ\| > 1°）的振荡来自**轨迹规划**，不应计入位置 PID 稳定性。

---

## 2. 稳定性指标定义

### 2.1 术语

| 符号 | 含义 |
|------|------|
| ε | 稳定带半宽（误差绝对值阈值） |
| T_hold | 连续保持在稳定带内的最短时间 |
| Ts | 调节时间：从测试开始到**首次**满足稳定条件的时间 |
| A_ss | 稳态峰峰值：稳定段内 `max(error) - min(error)` |
| RMS_ss | 稳态均方根：`sqrt(mean(error²))` |
| IAE_ss | 稳态积分绝对误差：`Σ|error| × dt`（仅稳定段） |
| ZCR | 过零率：去均值误差每秒变号次数（稳态应 ≈ 0） |

### 2.2 速度环指标

**测试条件**：`motor_cmd.mode = motor_cmd_speed`，`wave = motor_wave_direct`，`setpoint` 阶跃（如 0 → 60 °/s），`motion_accel` 足够大（≥ 2000 °/s²）。

| 指标 | 公式 / 算法 | 建议通过阈值（初值） |
|------|-------------|----------------------|
| Ts | 首次 \|e_v\| < 2 °/s 且持续 T_hold | 越短越好，记录即可 |
| A_ss | 稳定后 2 s 窗口内 peak-to-peak | < 4 °/s |
| RMS_ss | 稳定后 2 s 窗口 | < 1.5 °/s |
| Mp | `(v_max - v_target) / v_target × 100%` | < 20% |
| duty 饱和率 | 稳定段内 \|duty\| ≥ 3400 的采样占比 | < 5%（否则为物理饱和，非 PID 不稳） |

**不稳定判据**（任一即 FAIL）：

- 10 s 内未进入稳定带
- 稳定后 A_ss 持续增大（发散）
- duty 在稳态周期性大幅摆动而 e_v 看似正常（内环极限环）

### 2.3 位置环指标（精调区）

**测试条件**：`motor_cmd.mode = motor_cmd_position`，`wave = motor_wave_direct`，`setpoint` 阶跃 60°~90°，`motion_speed` / `motion_accel` 固定。

| 指标 | 窗口 | 建议通过阈值（初值） |
|------|------|----------------------|
| 到位 Ts | \|e_θ\| 首次 < 0.5° 且保持 1 s | 记录 |
| 精调 A_ss | 进入 \|e_θ\| ≤ 1° 后 2 s | < 0.25° |
| 精调 RMS_ss | 同上 | < 0.10° |
| 精调 ZCR | 同上，去均值后变号次数 / s | < 2 |

**注意**：位置环 Ki=0，允许小静差（RMS 非零但无振荡）。

### 2.4 综合对比标量

多组 PID 参数 A/B 对比时，推荐优先级：

1. **stable_flag** 必须为 1
2. **A_ss** 越小越好（稳态抖动）
3. **RMS_ss** 越小越好（精度）
4. **Ts** 越短越好（响应速度，次要）

可选单一标量：**IAE_ss**（稳定段积分），便于排序。

---

## 3. 标准测试流程（Phase 0，零代码）

### 3.1 前置条件

1. `DEBUG_JUSTFLOAT = 1`，Rebuild 烧录
2. `sts_mem_control_active() == 0`（本地 `motor_test_poll` 生效）
3. 上电后 `motor_position_home()` 已完成，`angle_start_flag == TRUE`

### 3.2 速度环阶跃测试

Keil Watch 设置：

```text
motor_cmd.mode         = motor_cmd_speed      (1)
motor_cmd.wave         = motor_wave_direct    (0)
motor_cmd.setpoint     = 60.0                 (°/s)
motor_cmd.motion_accel = 2000.0               (°/s²)
```

步骤：

1. 先 `setpoint = 0`，运行 2 s 等稳
2. 改 `setpoint = 60`，VOFA 录制 **≥ 8 s**
3. 重复 3 次，取 A_ss / RMS_ss **中位数**

### 3.3 位置环阶跃测试

```text
motor_cmd.mode         = motor_cmd_position   (2)
motor_cmd.wave         = motor_wave_direct    (0)
motor_cmd.setpoint     = <当前角 + 60°>
motor_cmd.motion_speed = 100.0                (°/s)
motor_cmd.motion_accel = 2000.0               (°/s²)
```

步骤：

1. 记录当前 `motor_context.sensor.motor_angle_multi_degree` 为 home
2. `setpoint = home + 60`，录制 **≥ 10 s**
3. 分析时**只取** \|target - angle\| ≤ 1° 之后的 2 s 窗口

### 3.4 VOFA+ 通道与离线误差

当前 6 通道（`main.c`）：

| ch | 变量 | 用途 |
|----|------|------|
| 0 | `motor_angle_multi_degree` | 实际角 |
| 1 | `target_angle` | 目标角 |
| 2 | `motor_speed_degree` | 实际速度 |
| 3 | `plan_speed` | 规划/内环目标速度 |
| 4 | `target_speed` | 外环最大速度 |
| 5 | `motor_get_duty()` | PWM |

离线计算：

```text
e_v = ch3 - ch2          # 速度跟踪误差
e_θ = ch1 - ch0          # 位置误差
```

### 3.5 Python 离线分析脚本（附录 A）

见本文 **§附录 A**，保存为 `tools/pid_tune_analyze.py`（Phase 0 实施时创建，当前仅规格）。

输出示例：

```text
=== Speed loop ===
settle_time_ms: 420
steady_pp:      2.1 deg/s
steady_rms:     0.8 deg/s
stable:         True

=== Position loop (trim zone) ===
trim_enter_ms:  1850
steady_pp:      0.18 deg
steady_rms:     0.06 deg
stable:         True
```

---

## 4. Phase 1 — MCU `pid_tune` 模块设计

### 4.1 文件与 API

新增文件：

```text
servo_project/User/servo/pid_tune.c
servo_project/User/servo/pid_tune.h
```

核心类型：

```c
typedef enum {
    PID_TUNE_IDLE = 0,
    PID_TUNE_ARMED,
    PID_TUNE_SETTLING,
    PID_TUNE_STABLE,
    PID_TUNE_UNSTABLE,
    PID_TUNE_DONE,
} pid_tune_state_t;

typedef struct {
    pid_tune_state_t state;
    uint8_t  stable;           /* 0/1 最终判定 */
    uint32_t start_ms;         /* g_control_tick @ arm */
    uint32_t settle_ms;        /* 首次满足稳定条件的耗时 */
    uint32_t steady_samples;   /* 稳定段采样计数 */
    float    steady_min;
    float    steady_max;
    float    steady_sum_sq;    /* 用于 RMS */
    float    steady_sum_abs;   /* 用于 IAE_ss */
    float    steady_pp;        /* max - min */
    float    steady_rms;
    float    steady_iae;
    uint16_t zero_crossings;   /* 去均值后变号次数 */
    uint16_t sat_count;        /* duty 饱和计数 */
} pid_tune_result_t;

/* 速度环 / 位置环各一套 */
extern pid_tune_result_t g_pid_tune_speed;
extern pid_tune_result_t g_pid_tune_pos;

void pid_tune_init(void);
void pid_tune_reset(pid_tune_result_t *r);
void pid_tune_arm(pid_tune_result_t *r);   /* Watch 写 1 或检测到 setpoint 阶跃 */

/* 在对应 PID 执行点调用 */
void pid_tune_speed_sample(float error, int16_t duty);
void pid_tune_pos_sample(float error, int16_t duty, uint8_t in_hold);
```

### 4.2 稳定判定状态机

```text
        arm / 检测到阶跃
IDLE ─────────────────→ ARMED
                           │
                           │ 连续 N 次 |e| < ε
                           ▼
                       SETTLING ──→ 计数不足则回 ARMED
                           │
                           │ 连续 N_hold 次 |e| < ε
                           ▼
                        STABLE ──→ 累计 steady_* 统计
                           │
                           │ 超时 TIMEOUT_MS 仍未稳定
                           ▼
                      UNSTABLE / DONE
```

参数表（可放 `pid_tune.h`）：

| 参数 | 速度环 | 位置环（精调） |
|------|--------|----------------|
| 采样率 | 500 Hz | 250 Hz |
| ε | 2.0 °/s | 0.15 ° |
| N_hold（连续样本） | 250（= 500 ms） | 250（= 1 s） |
| TIMEOUT_MS | 10000 | 15000 |
| 稳定段最短长度 | 1000 样本（2 s） | 500 样本（2 s） |
| DUTY_SAT_THRESH | 3400 | 3400 |

### 4.3 集成点

| 文件 | 改动 |
|------|------|
| `pid.c` → `speed_pid()` 末尾 | 调用 `pid_tune_speed_sample(error, duty)` |
| `pid.c` → `location_pid()` 末尾 | 调用 `pid_tune_pos_sample(error, duty, 1)` |
| `motor.c` → 规划段 | `pid_tune_pos_sample(err, duty, 0)` 或不调用 |
| `init.c` 或 `motor_init()` | `pid_tune_init()` |
| `motor_test.c` | setpoint 变化时 `pid_tune_arm()` + `pid_tune_reset()` |

**不修改** PID 算法本身；`pid_tune` 为旁路观测。

### 4.4 Keil Watch 面板

整参时在 Watch 窗口固定：

```text
g_pid_tune_speed.state
g_pid_tune_speed.stable
g_pid_tune_speed.settle_ms
g_pid_tune_speed.steady_pp
g_pid_tune_speed.steady_rms
g_pid_tune_pos.state
g_pid_tune_pos.stable
g_pid_tune_pos.steady_pp
g_pid_tune_pos.steady_rms
motor_context.motor_pid.motor_speed_pid.kp
motor_context.motor_pid.motor_speed_pid.ki
motor_context.motor_pid.motor_angle_pid.kp
motor_context.motor_pid.motor_angle_pid.kd
```

### 4.5 Flash / RAM 估算

- RAM：2 × `pid_tune_result_t` ≈ 80 B
- Flash：状态机 + 浮点统计 ≈ 1~2 KB
- CPU：每 PID 周期若干次 float 运算，可忽略

---

## 5. Phase 2 — VOFA 扩展与自动触发

### 5.1 扩展通道（`JF_MAX_CH` 已为 8）

在 `main.c` 的 `JF_SEND` 中增加（实现时）：

| ch | 内容 |
|----|------|
| 6 | `g_pid_tune_speed.steady_rms` 或瞬时 `motor_speed_pid.error` |
| 7 | `g_pid_tune_pos.steady_pp` 或 `motor_angle_pid.error` |

Phase 0 可先用离线算出的 error，不必改固件。

### 5.2 自动阶跃（可选）

在 `motor_test.c` 增加 `motor_cmd.auto_step`：

- 稳定 2 s 后自动改变 `setpoint`
- 自动 `pid_tune_arm()`，减少人工 Watch 操作

---

## 6. Phase 3 — 正弦扫频（可选）

利用 `motor_wave_sine`：

1. 固定 `motion_speed` 振幅，扫 `step`（相位增量 ∝ 频率）
2. 记录目标 vs 实际幅值比、相位差
3. 增益 ≈ 1 且相位 ≈ -180° 处为临界频率

**优先级低**：阶跃稳定性未通过前不做扫频。

---

## 7. 整参工作流

### 7.1 顺序（强制）

```text
1. 速度环单独阶跃（motor_cmd_speed）
   → stable + A_ss 通过后再进行下一步

2. 位置环阶跃（motor_cmd_position）
   → 只看精调区指标

3. 位置模式全行程 + STS 寄存器写 PID（sts_mem）
   → 验证 FD 软件改参后指标一致

4. 与 ~100°/s 饱和问题交叉验证
   → duty 饱和率高时先查物理/规划，勿盲目加 Kp
```

### 7.2 参数扫描建议

**速度 PI**（固定 Kd=0）：

| 轮次 | Kp | Ki | 说明 |
|------|----|----|------|
| 基线 | 10 | 0.5 | 当前默认 |
| 扫 Kp | 5, 8, 12, 15 | 0.5 | 看 A_ss 与 Mp |
| 扫 Ki | 10 | 0.2, 0.5, 0.8 | 看 RMS_ss 与积分饱和 |

**位置 PD**（Ki=0）：

| 轮次 | Kp | Kd | 说明 |
|------|----|----|------|
| 基线 | 5 | 0.05 | 当前默认 |
| 扫 Kp | 3, 5, 7 | 0.05 | 精调区振荡则减 |
| 扫 Kd | 5 | 0.03, 0.05, 0.08 | 超调大则增 |

每组参数：**3 次重复**，取 `steady_pp` 中位数。

### 7.3 现象 → 调整

| 现象 | 可能原因 | 调整 |
|------|----------|------|
| stable=0，持续振荡 | Kp 过大 | 减 Kp |
| stable=1 但 RMS_ss 大 | Ki 不足或静差 | 速度环增 Ki；位置环可试极小 Ki |
| Mp 大、有超调 | 阻尼不足 | 位置增 Kd；速度可试小 Kd |
| duty 长期 ±3500 | 饱和 | 非 PID 问题，查规划/物理限速 |
| 精调区 ZCR 高 | 位置 Kp 过大 | 减 Kp 或增 Kd |

---

## 8. 实施计划与里程碑

| 阶段 | 内容 | 产出 | 预估 |
|------|------|------|------|
| **Phase 0** | 按 §3 流程录 VOFA，编写并运行 `pid_tune_analyze.py` | 基线 A_ss/RMS 数据表 | 0.5 天 |
| **Phase 1** | 实现 `pid_tune.c/h`，接入 `pid.c`，Keil 编译通过 | Watch 可读 stable/pp/rms | 1 天 |
| **Phase 1 验收** | 速度 + 位置各 3 组参数，MCU 与 Python 结果偏差 < 10% | 验收记录 | 0.5 天 |
| **Phase 2** | VOFA 扩展 + 可选 auto_step | 波形直观对比 | 0.5 天 |
| **Phase 3** | 正弦扫频（可选） | 稳定裕度报告 | 1 天 |

**建议下一步**：先完成 Phase 0，用当前默认 PID 建立基线表格，再决定是否 Phase 1 合入。

---

## 9. 验收标准

### 9.1 Phase 0 验收

- [ ] 速度阶跃 3 次录波，Python 输出 `stable` / `steady_pp` / `steady_rms`
- [ ] 位置阶跃 3 次录波，精调区指标可重复（中位数变异系数 < 20%）
- [ ] 基线数据写入研发日报或 §9.2 表格

### 9.2 Phase 1 验收

- [ ] `g_pid_tune_speed.stable == 1` 在默认速度 PI 下可达成
- [ ] 修改 Kp 明显增大后 `stable == 0` 或 `steady_pp` 显著恶化（区分度有效）
- [ ] 位置精调区 `g_pid_tune_pos` 仅在 `in_hold` 时累计统计
- [ ] 与 Python 离线结果偏差 < 10%

### 9.3 M-05 整参完成判据（建议）

| 环 | 条件 |
|----|------|
| 速度 | stable=1，steady_pp < 4 °/s，steady_rms < 1.5 °/s |
| 位置精调 | stable=1，steady_pp < 0.25°，steady_rms < 0.10° |
| 全行程 | 60° 阶跃，无持续极限环，duty 饱和率 < 5%（额定负载） |

---

## 10. 风险与约束

| 风险 | 缓解 |
|------|------|
| 速度 LPF（α=0.3）掩盖高频振荡 | 同时监测 `duty` 的 steady_pp |
| VOFA 100 Hz 与 PID 500 Hz 不一致 | MCU 统计以 PID 频率为准；VOFA 仅可视化 |
| STS 与 motor_test 双路径 | 整参先用 motor_test；STS 写 PID 后再验证一轮 |
| 测试条件不一致导致误判 | 固定 setpoint、accel、录制时长，参数对比用中位数 |

---

## 附录 A · Python 离线分析脚本规格

> Phase 0 实施时在 `tools/pid_tune_analyze.py` 创建；以下为逻辑规格。

**输入**：VOFA 导出 CSV，列 `t, ch0..ch5`

**参数**（命令行或文件头）：

```python
SPEED_EPS = 2.0          # deg/s
SPEED_HOLD_S = 0.5
POS_EPS = 0.15           # deg
POS_HOLD_S = 1.0
POS_TRIM_DEG = 1.0
STEADY_WINDOW_S = 2.0
TEST_MODE = "speed"      # or "position"
```

**算法概要**：

1. 计算 `e_v` 或 `e_θ`
2. 从 t=0 扫描，找首次连续 HOLD 秒数 \|e\| < ε 的时刻 → `settle_ms`
3. 取 settle 后 STEADY_WINDOW 段 → min, max, RMS, IAE
4. 打印 JSON 或表格行

**依赖**：`numpy`, `pandas`（或纯 csv + stdlib）

---

## 附录 B · 与现有代码映射

| 现有符号 | 路径 |
|----------|------|
| `motor_context.motor_pid.motor_speed_pid` | `servo_config.h` / `pid.c` |
| `motor_context.motor_pid.motor_angle_pid` | 同上 |
| `motor_cmd` | `motor_test.c` |
| `JF_SEND(...)` | `main.c` L44-51 |
| `in_hold` 逻辑 | `motor.c` `motor_position_in_stop_zone()` |
| STS PID 写入 | `sts_mem.c` `sts_apply_pid_from_mem()` |

---

## 附录 C · 文档维护

- 速度观测 S-01 实施前后：对照 [速度观测改进方案.md](./速度观测改进方案.md) §9 验收
- 实现 Phase 1 后：更新本文 §0 状态、§8 里程碑打勾
- 同步 [项目总览-AI交接文档.md](./项目总览-AI交接文档.md) §6 M-05、§8 待办、§9 调试
- 基线数据确定后：回填 §9.2 表格

---

*文档版本 v1.0 · 2026-07-09 · 计划阶段，固件零改动*
