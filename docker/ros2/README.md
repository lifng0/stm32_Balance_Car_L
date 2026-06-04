# ROS 2 Docker Route

这套目录用于把树莓派上的 `ROS 2` 运行环境切换到容器内，而不是继续依赖宿主机不完整的 `rclpy/launch_ros`。

当前方案特点：

- 使用树莓派上已缓存的镜像：
  - `ros:humble-ros-base`
- 宿主机继续保留：
  - `pi_car_coordinator.py`
  - `tminiplus_bridge.py`
  - `balance-car-coordinator.service`
- 容器内负责：
  - `ROS 2` 包构建
  - `ROS 2` 节点运行
  - `bridge/behavior/perception/bringup`

## 目录说明

- `compose.yaml`
  - 容器编排入口
- `build_workspace.sh`
  - 在容器内构建工作区
- `run_bridge.sh`
  - 运行 `balance_car_bridge`
- `run_bringup.sh`
  - 运行最小 `bringup`

## 运行前提

- 树莓派宿主机已安装 Docker
- 树莓派宿主机已缓存：
  - `ros:humble-ros-base`
- 宿主机上的源码目录为：
  - `~/workspace/balance_car/ws`
  - `~/workspace/balance_car/scripts`
  - `~/workspace/balance_car/vendor`

## 推荐流程

1. 生成 `.env`
2. `docker compose config`
3. 进入容器执行 `build_workspace.sh`
4. 运行 `run_bridge.sh`
5. 后续再扩展到 `run_bringup.sh`

## 开机自启

如果希望树莓派开机后自动拉起容器内 `ROS 2 bringup`，当前仓库已经提供：

- `tools/start_ros2_bringup.sh`
- `tools/balance_car_ros2.service`

推荐安装方式：

1. 将 `tools/start_ros2_bringup.sh` 部署到：
   - `~/workspace/balance_car/scripts/start_ros2_bringup.sh`
2. 给脚本执行权限：
   - `chmod +x ~/workspace/balance_car/scripts/start_ros2_bringup.sh`
3. 将 `tools/balance_car_ros2.service` 部署到：
   - `/etc/systemd/system/balance_car_ros2.service`
4. 执行：
   - `sudo systemctl daemon-reload`
   - `sudo systemctl enable --now balance_car_ros2.service`

它的启动逻辑是：

- 先等待 `balance-car-coordinator.service` 提供的本地后端 `127.0.0.1:8765` 就绪
- 然后再执行容器内的 `ros2 launch balance_car_bringup system.launch.py`
