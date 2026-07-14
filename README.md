# servo_wtzn

飞特 STS3215c018 磁编码舵机兼容固件 — GD32F130 自研伺服板（16999-PS26040802）。

## 快速入口

| 文档 | 说明 |
|------|------|
| **[docs/项目总览-AI交接文档.md](docs/项目总览-AI交接文档.md)** | **项目总览（发给新 AI 从这里开始）** |
| [docs/STS完整实现指南.md](docs/STS完整实现指南.md) | STS 协议六阶段路线图 |
| [docs/硬件说明-16999-PS26040802.md](docs/硬件说明-16999-PS26040802.md) | 板卡引脚与 BOM |
| [docs/功能实现任务拆解.md](docs/功能实现任务拆解.md) | 任务清单与验收标准 |

## 工程路径

- Keil 工程：`servo_project/project/template.uvprojx`
- 用户源码：`servo_project/User/`
- 参考工程（协议已实现）：`E:\wtzn\xuniduoji`

## 当前进度（2026-07-10）

- ✅ 电机驱动、ADC、AS5600 编码器、位置/速度闭环
- ✅ STS 协议栈（PING/READ/WRITE + 0x80 内存表 + 电机桥接）
- ✅ T 型/三角位置轨迹规划、Stribeck 静摩擦补偿
- ✅ Flash EPROM 持久化（`sts_eeprom.c`，`0x0800FC00`）
- 🔶 多机总线稳定性、最高速度饱和、位置环整参、速度观测 S-01
- ✅ 中位校准（CALIB 0x0B、TORQUE_SWITCH=128、OFFSET @0x1F Flash 保存）
- ❌ RESTORE/出厂恢复
