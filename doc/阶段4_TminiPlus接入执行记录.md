# 阶段4 T-mini Plus 接入执行记录

更新时间：2026-06-02

## 1. 本次完成内容

本次已经完成阶段4中“雷达功能与雷达和树莓派通信”这一部分，且不依赖小车运动固定。

已完成：

- 官方 `YDLidar-SDK` 已部署到树莓派
- 官方 SDK 已在树莓派成功编译
- 官方 Python 绑定已在树莓派成功编译
- 我们自己的雷达桥接脚本已部署到树莓派
- 树莓派已成功识别 `T-mini Plus`
- 树莓派已成功初始化雷达并完成单次扫描读取

## 2. 树莓派上的实际部署位置

### 官方 SDK

- `~/workspace/balance_car/vendor/YDLidar-SDK`

### 我们自己的脚本

- `~/workspace/balance_car/scripts/tminiplus_bridge.py`

## 3. 当前已经可用的能力

脚本支持：

- `list`
  - 列出候选雷达串口设备
- `scan-once`
  - 执行单次扫描并输出摘要
- `stream`
  - 连续输出扫描摘要 JSON

## 4. 当前实际探测到的设备

当前树莓派已经识别到：

- `ydlidar1: /dev/ttyUSB1`

## 5. 当前实测结果

树莓派已经成功读取到 `Tmini Plus` 的设备信息：

- 型号：`Tmini Plus`
- 固件版本：`1.2`
- 硬件版本：`1`
- 序列号：`2026042200090771`
- 串口：`/dev/ttyUSB1`
- 波特率：`230400`

单次扫描实测结果：

- 点数：`430`
- 扫描频率：约 `9.3Hz`
- 最近目标角度：约 `-0.33°`
- 最近目标距离：约 `0.056m`
- 前方最小距离：约 `0.056m`
- 左前最小距离：约 `0.060m`
- 右前最小距离：约 `0.063m`

这说明：

- 雷达与树莓派的物理通信链路已经打通
- 官方 SDK 与 Python 绑定能够正常工作
- 后续避障和追踪所需的“扇区最小距离”和“最近目标角度/距离”基础数据已经可用

## 6. 当前建议的直接使用命令

### 列出设备

```bash
python3 ~/workspace/balance_car/scripts/tminiplus_bridge.py list
```

### 单次扫描

```bash
python3 ~/workspace/balance_car/scripts/tminiplus_bridge.py --device /dev/ttyUSB1 scan-once
```

### 连续输出扫描摘要

```bash
python3 ~/workspace/balance_car/scripts/tminiplus_bridge.py --device /dev/ttyUSB1 stream --count 20
```

## 7. 输出数据含义

当前脚本输出的核心字段包括：

- `points`
- `scan_frequency_hz`
- `front_min_distance_m`
- `front_left_min_distance_m`
- `front_right_min_distance_m`
- `closest_target_angle_deg`
- `closest_target_distance_m`

这些字段已经足够作为下一步：

- 雷达避障
- 雷达追踪

的控制输入。

## 8. 当前未做内容

本次仍未做：

- 小车运动控制闭环验证
- 雷达避障逻辑输出到 `STM32`
- 雷达追踪逻辑输出到 `STM32`
- `ROS 2` 驱动接入
- `slam` / 建图

## 9. 下一步建议

当前最合适的下一步是：

1. 基于现有扫描摘要做“前向避障”决策逻辑
2. 基于“最近目标角度/距离”做最小雷达追踪逻辑
3. 再把这两种模式的输出接到已经完成的 `STM32 <-> 树莓派` 串口协议上

