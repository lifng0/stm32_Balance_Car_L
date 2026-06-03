# ROS2容器化运行说明

更新时间：2026-06-03

## 1. 当前选定路线

当前项目已经正式切换为：

- 宿主机负责：
  - `pi_car_coordinator`
  - `STM32` 通信
  - 雷达现有宿主机链路
  - 软关机与系统状态机
- Docker 容器负责：
  - `ROS 2` 运行时
  - `ROS 2` 包构建
  - `bridge/behavior/perception/bringup`

当前实际使用的容器镜像为：

- `ros:humble-ros-base`

选择它的原因不是“最终版本只要 Humble”，而是：

- 这台树莓派上已经缓存了该镜像
- 可以立刻落地，不依赖当前拉不稳定的外部镜像源

## 2. 配置文件位置

本地文件：

- [compose.yaml](F:\stm32_Balance_Car_L\docker\ros2\compose.yaml)
- [build_workspace.sh](F:\stm32_Balance_Car_L\docker\ros2\build_workspace.sh)
- [run_bridge.sh](F:\stm32_Balance_Car_L\docker\ros2\run_bridge.sh)
- [run_bringup.sh](F:\stm32_Balance_Car_L\docker\ros2\run_bringup.sh)
- [README.md](F:\stm32_Balance_Car_L\docker\ros2\README.md)

树莓派部署位置：

- `~/workspace/balance_car/docker/ros2`

## 3. 容器方案关键点

### 3.1 网络

使用：

- `network_mode: host`

这样容器内的 `ROS` 节点可以直接访问宿主机上的：

- `127.0.0.1:8765`

也就是 `pi_car_coordinator` 暴露的本地后端接口。

### 3.2 设备

当前容器配置已经挂载：

- `/dev:/dev`

后续如果需要让容器内节点直接访问雷达设备，可以继续在这条线上扩展。

### 3.3 日志与 HOME

为避免 `ros2 launch` 把日志写到 `/.ros` 导致权限错误，当前容器已显式设置：

- `HOME=/tmp/balance_car_ros`
- `ROS_LOG_DIR=/tmp/balance_car_ros/log`

并挂载到宿主机：

- `~/workspace/balance_car/logs/ros2`

## 4. 当前工作区挂载

容器内挂载：

- 宿主机 `~/workspace/balance_car/ws`
  - 容器内 `/workspaces/balance_car/ws`
- 宿主机 `~/workspace/balance_car/scripts`
  - 容器内 `/workspaces/balance_car/scripts`
- 宿主机 `~/workspace/balance_car/vendor`
  - 容器内 `/workspaces/balance_car/vendor`

## 5. 当前验证结果

### 5.1 compose 配置验证

已通过：

- `docker compose -f compose.yaml config`

### 5.2 容器运行时验证

已通过：

- 容器内导入：
  - `rclpy`
  - `launch_ros`

### 5.3 容器内工作区构建验证

已通过：

- `balance_car_interfaces`
- `balance_car_bridge`
- `balance_car_lidar`
- `balance_car_behavior`
- `balance_car_perception`
- `balance_car_bringup`

### 5.4 容器内最小 bringup 启动验证

已实际启动：

- `ros2 launch balance_car_bringup bridge_only.launch.py`

日志显示：

- `bridge_node` 进程已成功启动

## 6. 推荐命令

### 6.1 构建工作区

```bash
cd ~/workspace/balance_car/docker/ros2
docker compose -f compose.yaml run --rm ros2-dev bash -lc "source /opt/ros/humble/setup.bash && /workspaces/balance_car/docker/ros2/build_workspace.sh"
```

### 6.2 运行 bridge

```bash
cd ~/workspace/balance_car/docker/ros2
docker compose -f compose.yaml run --rm ros2-dev bash -lc "source /opt/ros/humble/setup.bash && /workspaces/balance_car/docker/ros2/run_bridge.sh"
```

### 6.3 运行最小 bringup

```bash
cd ~/workspace/balance_car/docker/ros2
docker compose -f compose.yaml run --rm ros2-dev bash -lc "source /opt/ros/humble/setup.bash && /workspaces/balance_car/docker/ros2/run_bringup.sh"
```

## 7. 当前结论

当前 `ROS 2` 容器路线已经从“规划方案”变成了“实测可运行方案”。

也就是说，后续开发已经可以直接默认：

- `STM32/状态机/软关机` 在宿主机层
- `ROS 2` 节点、构建、启动在 Docker 容器层
