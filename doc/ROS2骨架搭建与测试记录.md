# ROS2骨架搭建与测试记录

更新时间：2026-06-03

## 1. 本次完成内容

已完成以下工作：

- 在本地仓库新增 `ros2_ws/src` 目录
- 新建 6 个 `ROS 2` 包骨架：
  - `balance_car_interfaces`
  - `balance_car_bridge`
  - `balance_car_lidar`
  - `balance_car_behavior`
  - `balance_car_perception`
  - `balance_car_bringup`
- 将宿主机协调器 [pi_car_coordinator.py](F:\stm32_Balance_Car_L\tools\pi_car_coordinator.py) 扩展为本地 `TCP JSON` 后端
- 将新的 `ROS 2` 源码包同步到树莓派：
  - `~/workspace/balance_car/ws/src/...`

## 2. 已封装接入的现有功能

### 2.1 整车状态与宿主机控制层

已通过 `balance_car_bridge` 对接：

- `pi_car_coordinator` 的整车状态
- 当前模式状态
- 初始化状态
- 暂停状态
- 最近事件

桥接方式：

- `pi_car_coordinator` 在树莓派本机开启：
  - `127.0.0.1:8765`
- `balance_car_bridge` 后续通过这个本地接口取数

### 2.2 雷达功能

已通过 `balance_car_lidar` 封装当前已跑通的 `T-mini Plus` 接入思路：

- 复用了当前脚本中已经验证过的：
  - SDK 加载
  - 串口自动发现
  - 单帧扫描摘要提取

当前节点定位：

- 第一阶段先发布雷达摘要
- 后续再扩展到标准 `/scan`

### 2.3 行为与感知

已落下骨架：

- `balance_car_behavior/task_manager_node.py`
- `balance_car_perception/k210_parser_node.py`

当前阶段：

- 先以骨架和占位逻辑为主
- 等后续具体模式逻辑接入

## 3. 本次测试结果

### 3.1 本地源码语法测试

已通过：

- `pi_car_coordinator.py`
- `balance_car_bridge`
- `balance_car_lidar`
- `balance_car_behavior`
- `balance_car_perception`
- `balance_car_bringup/launch`

方式：

- `python -m py_compile`

结果：

- 通过

### 3.2 树莓派构建工具链补齐

本次已在树莓派宿主机安装：

- `colcon`
- `python3-colcon-*` 相关核心插件
- `ament-cmake`
- `ament-cmake-core`
- `ament-cmake-python`
- `python3-ament-index`
- `python3-ament-package`
- `python3-catkin-pkg`
- `python3-empy`
- `python3-pytest`

结果：

- `colcon --help` 可用
- `colcon list` 可用

### 3.3 工作区权限修复

已修复：

- `~/workspace/balance_car/ws/build`
- `~/workspace/balance_car/ws/install`
- `~/workspace/balance_car/ws/log`

此前这些目录属主是 `root`，会导致：

- `colcon list`
- `colcon build`

因日志目录权限不足而失败。

修复后：

- 工作区重新归属 `lifng0`
- `colcon` 可正常写入日志与构建产物

### 3.4 树莓派宿主机服务测试

已执行：

- 重启 `balance-car-coordinator.service`
- 检查服务状态
- 查询最近日志

结果：

- 服务正常运行
- 日志显示本地后端已启动：
  - `backend ready tcp://127.0.0.1:8765`

### 3.5 本地后端接口测试

已在树莓派上通过 Python socket 访问：

- `127.0.0.1:8765`

请求：

- `{"cmd":"get_state"}`

结果：

- 成功收到 JSON 状态响应
- 返回内容包含：
  - `system_mode`
  - `pi_ready`
  - `lidar_ready`
  - `system_ready`
  - `last_event_name`
  - `car_state`
  - `stm32_device`

说明：

- 宿主机层和后续 `ROS` 层之间的桥接入口已经打通

### 3.6 ROS 工作区构建测试

本次已在树莓派上成功执行：

```bash
cd ~/workspace/balance_car/ws
colcon build --packages-select \
  balance_car_bridge \
  balance_car_lidar \
  balance_car_behavior \
  balance_car_perception \
  balance_car_bringup
```

结果：

- `balance_car_bridge` 构建成功
- `balance_car_lidar` 构建成功
- `balance_car_behavior` 构建成功
- `balance_car_perception` 构建成功
- `balance_car_bringup` 构建成功

并确认：

- `bringup` 包的 `launch` 和 `config` 文件已正确安装到 `install` 目录

### 3.7 运行时测试

本次已直接尝试启动：

```bash
source ~/workspace/balance_car/ws/install/setup.sh
~/workspace/balance_car/ws/install/balance_car_bridge/lib/balance_car_bridge/bridge_node
```

结果：

- 启动失败
- 明确错误为：
  - `ModuleNotFoundError: No module named 'rclpy'`

这说明：

- 当前树莓派已经具备“构建工具链”
- 但还不具备“完整 ROS 2 Python 运行时”

### 3.8 容器化 ROS 2 运行时测试

本次新增完成：

- 使用树莓派已缓存镜像：
  - `ros:humble-ros-base`
- 新建并部署：
  - `~/workspace/balance_car/docker/ros2/compose.yaml`
  - `~/workspace/balance_car/docker/ros2/build_workspace.sh`
  - `~/workspace/balance_car/docker/ros2/run_bridge.sh`
  - `~/workspace/balance_car/docker/ros2/run_bringup.sh`
  - `~/workspace/balance_car/docker/ros2/.env`

已验证：

- `docker compose config` 通过
- 容器内可导入：
  - `rclpy`
  - `launch_ros`
- 容器内可成功构建：
  - `balance_car_interfaces`
  - `balance_car_bridge`
  - `balance_car_lidar`
  - `balance_car_behavior`
  - `balance_car_perception`
  - `balance_car_bringup`
- 容器内执行：
  - `ros2 launch balance_car_bringup bridge_only.launch.py`
  已看到 `bridge_node` 进程成功启动

结论：

- `ROS 2` 运行时问题已经通过 Docker 路线绕开

## 4. 当前测试边界

本次已经完成：

- 宿主机后端测试
- `colcon` 工具链测试
- 新包工作区构建测试

但仍未完成：

- 容器内 `ROS 2` 节点运行测试
- 容器内 `launch_ros` 启动测试
- 容器内 `balance_car_interfaces` 构建测试

已确认当前环境：

- `python3-serial` 可用
- `colcon` 可用
- `ament_index_python` 可用
- `catkin_pkg` 可用
- `empy` 可用
- `/opt/ros` 下没有标准宿主机发行版目录
- `rclpy` 不可用
- `launch_ros` 不可用
- `rosidl` 默认生成器链路不完整
- 但容器内 `rclpy/launch_ros/rosidl` 可用

这意味着：

- 当前已经可以完成：
  - 源码骨架落地
  - 宿主机功能封装
  - 宿主机构建验证
  - 容器内完整 `ROS` 包构建
  - 容器内最小 `ROS` 启动验证
- 当前正式推荐的运行方式已经是容器路线，而不是宿主机直接运行 `ROS 2`

## 5. 结论

本阶段已经完成两件关键事：

1. 新的 `ROS 2` 包结构已经真正落地，而不是停留在设计文档。
2. 现有已跑通的树莓派功能已经通过宿主机本地后端成功封装，后续 `ROS` 层有了明确、可访问的接入点。
3. 新的 `ROS` 包中已在容器内成功构建 6 个包。
4. 新的 `ROS` 路线已经切换为 Docker 容器内运行。

当前还差的不是“ROS 运行时能否工作”，而是后续把更多实际模式逻辑逐步迁入容器内节点。

## 6. 下一步建议

下一步建议优先处理：

- 把当前宿主机已跑通的雷达/视觉结果进一步桥接到容器内 `ROS` 节点
- 扩展 `system.launch.py`
- 将行为节点从“占位骨架”逐步替换为真实模式逻辑
