# ROS2整体设计与包结构方案

更新时间：2026-06-03

## 1. 文档目标

本文给出当前项目在树莓派侧的 `ROS 2` 可落地整体设计，重点说明：

- 当前已有的 `ROS` 骨架是什么
- 后续应该拆成哪些包
- 每个包内部应该有哪些节点
- 各节点之间如何分工
- 哪些内容继续留在宿主机服务层，哪些进入 `ROS 2`
- 后续开发时的推荐落地顺序

本文的目标不是追求“架构好看”，而是为了让项目在最短周期内稳定交付，并且后续容易迭代。

## 2. 当前现状

### 2.1 树莓派上当前已有的工作区骨架

树莓派当前已经存在工作区目录：

- `~/workspace/balance_car/ws/src`
- `~/workspace/balance_car/ws/build`
- `~/workspace/balance_car/ws/install`

其中已经存在一个原型包：

- `~/workspace/balance_car/ws/src/car_driver`

这个包目前的特点是：

- 类型：`ament_python`
- 入口节点：`driver_node`
- 代码位置：
  - `car_driver/car_driver/driver_node.py`
  - `car_driver/package.xml`
  - `car_driver/setup.py`
- 功能：直接打开 `/dev/ttyUSB0`，读取串口文本并打印

### 2.2 当前骨架的结论

这个 `car_driver` 可以视为“最初 ROS 试通包”，但不建议作为正式架构继续扩展。原因如下：

- 它依赖固定设备名 `/dev/ttyUSB0`，不适合当前项目
- 它是“读文本串口”的思路，而我们现在已经切到二进制协议
- 它没有模式状态机、没有心跳、没有暂停/恢复、没有软关机
- 它没有接入雷达，也没有接入 `K210`
- 它没有参数文件、`launch` 文件、自定义消息与服务

结论：

- 当前树莓派上“有 ROS 目录骨架”
- 但真正可继续复用的核心能力，主要还是宿主机层已经完成的三个脚本：
  - `pi_serial_bridge.py`
  - `tminiplus_bridge.py`
  - `pi_car_coordinator.py`

## 3. 设计原则

后续 `ROS 2` 设计遵循以下原则：

1. 不推翻现有已跑通的宿主机链路。
2. 不让多个程序同时抢占 `STM32` 串口。
3. 不把“整车控制面”和“算法数据面”混在一起。
4. 先做能跑通整车演示的最小结构，再逐步补标准化。
5. 优先保证调试可见性和状态可观察性。

## 4. 总体分层

建议采用两层结构：

### 4.0 当前实际运行路线

截至当前，项目已经正式采用：

- 宿主机服务层
- Docker 容器内 `ROS 2` 功能层

当前实际使用的容器镜像是：

- `ros:humble-ros-base`

选择它的工程原因是：

- 树莓派本机已经缓存该镜像
- 不依赖当前不稳定的外部镜像拉取
- 能快速绕开宿主机上缺失的 `rclpy/launch_ros`

后续如果网络条件改善，可以再评估是否升级为更接近目标版本的镜像环境。

### 4.1 宿主机服务层

这一层继续运行在树莓派宿主机，不强依赖 `ROS 2`。

主要组成：

- `pi_car_coordinator.py`
- `pi_serial_bridge.py`
- `tminiplus_bridge.py`
- `balance-car-coordinator.service`

职责：

- 管理 `STM32 <-> 树莓派` 串口
- 上电初始化握手
- 向 `STM32` 上报：
  - `PI_READY`
  - `LIDAR_READY`
  - `SYSTEM_READY`
- 监听：
  - `START_REQUEST`
  - `MODE_SELECT`
  - `STOP_ASSERT`
  - `STOP_CLEAR`
  - `SHUTDOWN_REQUEST`
- 处理软关机
- 保证树莓派还没进入 `ROS` 时，整车依然能完成基础管理

这一层属于“控制面”。

### 4.2 ROS 2 功能层

这一层负责：

- 雷达标准化数据发布
- 视觉结果标准化
- 建图与导航
- 高层行为编排
- 与宿主机控制层做桥接

这一层属于“数据面/任务面”。

## 5. 包结构设计

建议最终整理为 6 个核心包。

### 5.1 `balance_car_interfaces`

类型：

- `rosidl` 接口包

作用：

- 定义整个项目里统一使用的消息与服务

建议包含：

- `msg/CarState.msg`
- `msg/SystemState.msg`
- `msg/CarEvent.msg`
- `msg/VisionTarget.msg`
- `srv/SetCarMode.srv`
- `srv/StartTask.srv`
- `srv/StopTask.srv`
- `srv/QueryState.srv`

建议字段：

`CarState.msg`

- `uint8 mode`
- `bool stop_flag`
- `bool low_power`
- `float32 move_x`
- `float32 move_z`
- `float32 battery`
- `float32 angle`

`SystemState.msg`

- `bool pi_ready`
- `bool lidar_ready`
- `bool system_ready`
- `bool paused_by_pickup`
- `bool running`
- `bool shutting_down`

`CarEvent.msg`

- `uint8 event_code`
- `string event_name`
- `builtin_interfaces/Time stamp`

`VisionTarget.msg`

- `string target_type`
- `bool detected`
- `float32 x`
- `float32 y`
- `float32 confidence`
- `string extra`

### 5.2 `balance_car_bridge`

类型：

- `ament_python`

作用：

- 作为 `ROS 2` 和宿主机控制层之间的桥

这是后续最核心的包之一。

建议节点：

- `car_bridge_node`
- `car_supervisor_node`

#### `car_bridge_node`

职责：

- 与 `pi_car_coordinator` 交互
- 把宿主机层的整车状态发布为 `ROS` 话题
- 接收 `ROS` 指令并转发给宿主机控制层

建议输入：

- `/cmd_vel`
- `/car/set_mode`
- `/car/start_request`
- `/car/stop_request`

建议输出：

- `/car/state`
- `/car/system_state`
- `/car/events`

注意：

- `car_bridge_node` 不直接占用 `STM32` 串口
- 它应该通过本地 `IPC` 或本地 `TCP/Unix socket` 与 `pi_car_coordinator` 通信

推荐方式：

- 第一版使用 `localhost TCP`
- 宿主机 `pi_car_coordinator` 暴露一个轻量本地接口
- `ROS` 节点从这里收发数据

原因：

- 调试简单
- 不会和 `STM32` 串口抢占冲突
- 后续容器化时也更容易跨宿主机/容器通信

#### `car_supervisor_node`

职责：

- 维护 `ROS` 视角下的整车运行状态
- 统一管理：
  - 是否允许运行
  - 是否处于暂停
  - 当前模式
  - 当前活动任务

它不直接控制硬件，而是统一管理状态。

### 5.3 `balance_car_lidar`

类型：

- `ament_python`

作用：

- 负责雷达数据接入与发布

建议节点：

- `lidar_driver_node`
- `lidar_preprocess_node`

#### `lidar_driver_node`

职责：

- 读取 `T-mini Plus`
- 发布标准 `sensor_msgs/LaserScan`

实现建议：

- 第一版可继续复用当前 `tminiplus_bridge.py` 的雷达接入逻辑
- 封装成 ROS 节点
- 等网络与依赖更稳定后，再评估是否切换到标准驱动链路

建议输出：

- `/scan`

#### `lidar_preprocess_node`

职责：

- 对 `/scan` 做轻量预处理
- 输出适合上层逻辑直接使用的摘要信息

建议输出：

- `/lidar/front_obstacle`
- `/lidar/closest_target`
- `/lidar/summary`

作用：

- 给避障、跟随、展示层直接提供可用信息
- 减少每个行为节点重复解析 `/scan`

### 5.4 `balance_car_perception`

类型：

- `ament_python`

作用：

- 统一管理视觉结果输入

硬件前提：

- 当前项目使用的是 `K210`，不是深度相机

因此这个包的职责不是发布深度图，而是发布 `K210` 已经提取好的“目标结果”。

建议节点：

- `k210_parser_node`
- `vision_router_node`

#### `k210_parser_node`

职责：

- 读取 `K210 -> STM32 -> 树莓派` 或未来的直接结果链路
- 解析：
  - 二维码
  - 线跟踪结果
  - 跟随目标
  - 数字识别
  - 自主学习类别

建议输出：

- `/vision/qr`
- `/vision/line`
- `/vision/follow_target`
- `/vision/mnist`
- `/vision/self_learn`

#### `vision_router_node`

职责：

- 将不同模式下真正有效的视觉结果统一整理后输出

建议输出：

- `/vision/current_target`

### 5.5 `balance_car_behavior`

类型：

- `ament_python`

作用：

- 实现高层任务逻辑

建议节点：

- `task_manager_node`
- `manual_mode_node`
- `follow_mode_node`
- `qr_mode_node`
- `line_mode_node`
- `mnist_mode_node`
- `slam_mode_node`

#### `task_manager_node`

职责：

- 这是整个 `ROS` 层最关键的状态机节点
- 根据当前模式、启动事件、暂停事件决定哪个任务节点处于活跃态

它应该响应：

- 来自 `STM32` 的开始事件
- 模式切换事件
- 被拿起暂停事件
- 低压事件
- 停止事件

它应该输出：

- 当前活跃任务状态
- 给桥接层的运动或行为目标

#### 各模式节点职责

`manual_mode_node`

- 处理蓝牙/上位逻辑的人工控制映射

`follow_mode_node`

- 基于 `K210` 或雷达摘要做目标跟随

`qr_mode_node`

- 响应二维码识别结果，完成指定动作流程

`line_mode_node`

- 处理视觉巡线逻辑

`mnist_mode_node`

- 处理数字识别驱动的任务流程

`slam_mode_node`

- 处理建图或雷达自主演示模式

说明：

- 这些模式节点不一定一开始全部实现
- 第一阶段只需要先实现：
  - `task_manager_node`
  - `follow_mode_node`
  - `slam_mode_node`

### 5.6 `balance_car_bringup`

类型：

- `ament_python` 或纯配置包

作用：

- 统一放：
  - `launch` 文件
  - 参数文件
  - 设备路径配置
  - 模式映射表

建议目录：

- `launch/system.launch.py`
- `launch/lidar_only.launch.py`
- `launch/bridge_only.launch.py`
- `launch/demo.launch.py`
- `config/devices.yaml`
- `config/modes.yaml`
- `config/behavior.yaml`

## 6. 节点关系建议

推荐的节点关系如下：

- `pi_car_coordinator`
  - 宿主机常驻服务
  - 独占 `STM32` 串口
- `car_bridge_node`
  - 和 `pi_car_coordinator` 交换数据
  - 向 ROS 发布 `/car/state`
- `lidar_driver_node`
  - 发布 `/scan`
- `lidar_preprocess_node`
  - 订阅 `/scan`
  - 发布摘要结果
- `k210_parser_node`
  - 发布视觉结果
- `task_manager_node`
  - 订阅车体状态、雷达摘要、视觉结果
  - 决定当前要执行哪个任务
- 各模式节点
  - 提供具体策略输出

## 7. 设备访问边界

这是落地时最重要的一条约束。

### 7.1 `STM32` 串口

只能由一个主控程序独占，推荐继续由：

- `pi_car_coordinator`

占用。

其他 `ROS` 节点不直接打开这个串口。

### 7.2 雷达串口

推荐由：

- `balance_car_lidar/lidar_driver_node`

独占。

后续如果仍然需要宿主机脚本调试，可以：

- 平时跑 ROS 节点
- 调试时再单独停掉 ROS 节点，运行 `tminiplus_bridge.py`

不要同时抢占。

### 7.3 `K210` 数据链路

后续要先明确：

- 是继续经 `STM32` 转发给树莓派
- 还是直接接到树莓派

在没有改线前，先按“经 `STM32` 转发”设计。

## 8. 推荐目录结构

后续推荐在 `ws/src` 下整理成：

```text
ws/src/
  balance_car_interfaces/
  balance_car_bridge/
  balance_car_lidar/
  balance_car_perception/
  balance_car_behavior/
  balance_car_bringup/
```

原来的：

- `car_driver`

建议保留一段时间作为参考原型，但不再继续扩展。

## 9. 推荐启动方式

### 9.1 宿主机自启

由 `systemd` 自启：

- `balance-car-coordinator.service`

它负责：

- 开机握手
- 树莓派状态同步
- 软关机

### 9.2 ROS 启动

由 `launch` 启动：

- `bridge`
- `lidar`
- `perception`
- `behavior`

推荐入口：

- `system.launch.py`

### 9.3 建图单独入口

建图和整车任务分开启动更稳：

- `slam.launch.py`

原因：

- 建图阶段调试量大
- 和整车行为混在一起不利于定位问题

## 10. 推荐开发顺序

为了尽快落地，建议按下面顺序推进。

### 第一步：规范接口包

先做：

- `balance_car_interfaces`

这样后面所有节点接口都能统一。

### 第二步：做桥接包

再做：

- `balance_car_bridge`

目标：

- 把当前宿主机控制层变成 ROS 可见的标准状态和控制接口

### 第三步：雷达 ROS 化

再做：

- `balance_car_lidar`

目标：

- 发布 `/scan`
- 发布前向障碍摘要

### 第四步：行为状态机

再做：

- `balance_car_behavior/task_manager_node`

目标：

- 让整车运行逻辑真正进入 ROS 编排

### 第五步：逐模式接入

最后逐步补：

- 跟随
- 二维码
- 数字识别
- 建图

## 11. 最小可交付版本定义

第一版 ROS 可交付，不需要一口气做全。

最小可交付建议定义为：

- 宿主机服务层稳定运行
- `balance_car_interfaces` 完成
- `balance_car_bridge` 完成
- `balance_car_lidar` 完成基础 `/scan`
- `task_manager_node` 能识别：
  - 模式选择
  - 开始执行
  - 被拿起暂停
  - 返回模式选择
  - 软关机

做到这一步，整个系统就已经具备“ROS 总线结构清晰、后续模式可继续挂接”的基础。

## 12. 最终结论

后续 ROS 设计不应从“旧 `car_driver` 包”继续堆，而应该：

- 保留宿主机控制层
- 在其上新增标准 ROS 层
- 用桥接思路而不是串口直连思路
- 先统一接口，再逐层接入雷达、视觉、行为

一句话概括：

- `pi_car_coordinator` 负责整车控制面
- `ROS 2` 负责感知、建图、任务和行为
- 两者通过本地桥接接口连接

这套结构最符合你当前硬件条件，也最有利于后面稳定交付。
