# balance_car ROS 2 workspace skeleton

这个目录保存树莓派侧 `ROS 2` 工作区源码骨架，对应树莓派上的：

- `~/workspace/balance_car/ws/src`

当前版本重点完成：

- `balance_car_interfaces`：接口包骨架
- `balance_car_bridge`：通过本地 `TCP JSON` 对接 `pi_car_coordinator`
- `balance_car_lidar`：封装 `T-mini Plus` 扫描摘要
- `balance_car_behavior`：基础任务状态管理骨架
- `balance_car_perception`：`K210` 视觉数据解析与结构化发布
- `balance_car_bringup`：启动与配置骨架

当前推荐运行方式：

- 宿主机负责：
  - `pi_car_coordinator.py`
  - `pi_serial_bridge.py`
  - `tminiplus_bridge.py`
- `ROS 2` 运行在 Docker 容器内：
  - 当前实测使用树莓派本机已缓存镜像 `ros:humble-ros-base`
  - 容器配置见 [docker/ros2](F:\stm32_Balance_Car_L\docker\ros2)

当前视觉控制链路已经调整为：

- `K210 -> 树莓派串口 -> balance_car_perception`
- `balance_car_perception -> /vision/target`
- `balance_car_navigation` 中的视觉节点根据当前模式直接调用 backend 下发运动控制
- `STM32` 只负责平衡、底盘执行、蓝牙与基础状态机

注意：

- 已跑通的现有功能仍然由宿主机脚本负责
- `ROS 2` 节点通过本地后端接口接入这些宿主机功能
