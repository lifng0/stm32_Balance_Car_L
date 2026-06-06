# K210与ROS接口文档

更新时间：2026-06-05

## 1. 目标

这份文档只回答一件事：

- 在当前架构下，`K210` 如何接入树莓派后端与 `ROS 2`
- 算法或应用节点如何从 `ROS` 里读取 `K210` 数据
- 算法或应用节点如何从 `ROS` 向 `K210` 发送指令

当前默认架构是：

- `K210 UART2 -> TTL转USB -> 树莓派 USB`
- `树莓派协调器 -> K210 串口`
- `ROS 2 perception 节点 -> 调用树莓派协调器 backend`

`STM32` 不再作为 `K210` 的默认主通信对象。

## 2. 总体链路

### 2.1 上行数据

`K210 -> 树莓派串口 -> pi_car_coordinator.py -> ROS /vision/status_json`

说明：

- `K210` 通过串口发送文本或 `$...#` 帧
- 树莓派协调器解析后写入 `vision_state`
- `balance_car_k210_parser` 节点定时轮询 backend
- ROS 侧发布到 `/vision/status_json`

### 2.2 下行指令

`ROS /vision/k210_command -> balance_car_k210_parser -> pi_car_coordinator.py -> K210 串口`

说明：

- ROS 节点向 `/vision/k210_command` 发布 `std_msgs/String`
- `balance_car_k210_parser` 收到后调用 backend 命令 `set_k210_text`
- backend 直接把字符串写到 `K210` 串口

## 3. 当前K210烧录脚本约定

当前仓库中的测试脚本：

- [canmv_k210_uart_demo.py](F:\stm32_Balance_Car_L\tools\canmv_k210_uart_demo.py)

这份脚本现在已经不是“只做 LCD 测试”的临时文件，而是建议直接烧录到 `K210` 上的主程序。

建议部署方式：

- 在 `CanMV IDE` 中运行确认正常
- 再保存为 `main.py`

它当前具备下面的行为：

- 启动后会向树莓派发送：`$BOOT:UART2_READY#`
- 收到 `PING` 后会回：`$PONG#`
- 收到 `SHOW:xxx` 后会在 LCD 显示 `xxx`，并回：`$SHOW_OK:xxx#`
- 支持 `MODE:IDLE / MODE:DISPLAY / MODE:COLOR`
- 支持 `COLOR:ON / COLOR:OFF`
- 支持 `COLOR:TARGET:RED/GREEN/BLUE/YELLOW/ANY`
- 支持 `STATUS` / `GET:STATUS`
- 在颜色识别模式下会持续回传：
  - `$COLOR:RED,160,120,40,42,1680#`
  - 或 `$COLOR:NONE#`
- 周期性回传状态：
  - `$STATUS:MODE=COLOR,TARGET=ANY,DETECTED=RED,REASON=periodic#`

这意味着即使还没有正式视觉算法，你也可以先用它完成通信联调。

## 4. ROS侧现成接口

### 4.1 输入话题

#### `/vision/status_json`

- 类型：`std_msgs/String`
- 来源：`balance_car_k210_parser`
- 内容：一段 JSON 字符串

常见字段：

```json
{
  "source": "k210_uart",
  "transport": "usb_serial",
  "mode": 0,
  "mode_name": "Normal",
  "valid": true,
  "raw": "PONG",
  "timestamp": 1780646400.0,
  "vision_type": 1,
  "text": "PONG"
}
```

说明：

- 当前最稳妥的用法是优先读取：
  - `valid`
  - `vision_type`
  - `text`
  - `raw`
- 如果后面 `K210` 发的是框选类结果，还会带：
  - `x`
  - `y`
  - `w`
  - `h`
  - `area`

#### `/vision/k210_link_json`

- 类型：`std_msgs/String`
- 来源：`balance_car_k210_parser`
- 内容：K210 链路状态 JSON

示例：

```json
{
  "k210_device": "/dev/serial/by-path/platform-xhci-hcd.1-usb-0:2:1.0-port0",
  "k210_port": "/dev/serial/by-path/platform-xhci-hcd.1-usb-0:2:1.0-port0",
  "k210_ready": true,
  "k210_last_error": "",
  "k210_last_rx_at": 1780646400.0,
  "vision_source": "k210_uart",
  "vision_valid": true,
  "timestamp": 1780646400.2
}
```

用途：

- 判断树莓派是否已经打开 `K210` 串口
- 判断 `K210` 最近是否真的有回包
- 联调时先看这个话题，再判断是不是算法问题

#### `/vision/k210_command_result_json`

- 类型：`std_msgs/String`
- 来源：`balance_car_k210_parser`
- 内容：ROS 下发命令后的 backend 执行结果

示例：

```json
{
  "ok": true,
  "cmd": "set_k210_text",
  "text": "PING",
  "timestamp": 1780646400.4
}
```

用途：

- 确认 ROS 到 backend 的调用是否成功
- 注意：`ok=true` 只代表树莓派已成功写串口，不等于 `K210` 一定执行成功
- 是否真正执行，仍应结合 `/vision/status_json` 或 LCD 画面判断

### 4.2 输出话题

#### `/vision/k210_command`

- 类型：`std_msgs/String`
- 使用方：任意 ROS 节点

用途：

- 向 `K210` 发送文本指令
- 当前测试脚本下推荐直接发简单字符串

推荐示例：

- `PING`
- `SHOW:hello`
- `SHOW:qr mode`
- `MODE:COLOR`
- `COLOR:TARGET:RED`

## 5. 推荐的K210串口协议

### 5.1 联调阶段

联调阶段推荐优先发送文本或短命令：

- `PING`
- `SHOW:hello`
- `MODE:QR`
- `LED:RED`

优势：

- 好看日志
- 好看 LCD
- 易于确认是不是串口/脚本本身的问题

### 5.2 正式阶段

正式阶段建议 `K210` 对上行数据统一发 `$...#` 帧。

例如：

- `$QR:abc123#`
- `$DIGIT:7#`
- `$LINE:120,80,30,10#`
- `$FOLLOW:156,104,48,52#`
- `$COLOR:RED,160,120,40,42,1680#`
- `$STATUS:MODE=COLOR,TARGET=RED,DETECTED=RED,REASON=periodic#`

当前树莓派后端已支持两种输入：

- `$...#` 帧
- 普通换行文本

所以前期可以混用，后期再逐步统一。

## 6. 组员如何在ROS里使用

### 6.1 读取K210数据

Python 节点示例：

```python
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VisionUser(Node):
    def __init__(self):
        super().__init__("vision_user")
        self.sub = self.create_subscription(String, "/vision/status_json", self.on_vision, 10)

    def on_vision(self, msg: String):
        payload = json.loads(msg.data)
        if not payload.get("valid", False):
            return
        text = payload.get("text", "")
        self.get_logger().info(f"k210 text={text}")
```
        
### 6.2 向K210发指令

Python 节点示例：

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VisionCommander(Node):
    def __init__(self):
        super().__init__("vision_commander")
        self.pub = self.create_publisher(String, "/vision/k210_command", 10)
        self.timer = self.create_timer(1.0, self.send_once)
        self.sent = False

    def send_once(self):
        if self.sent:
            return
        msg = String()
        msg.data = "PING"
        self.pub.publish(msg)
        self.sent = True
```

### 6.3 观察回传

建议同时观察：

- `/vision/k210_command_result_json`
- `/vision/status_json`

理想顺序是：

1. `/vision/k210_command_result_json` 出现 `ok=true`
2. `K210 LCD` 出现对应显示
3. `/vision/status_json` 出现 `PONG`、`SHOW_OK:...`、`COLOR:...` 等对应内容

## 7. 节点部署位置

### 7.1 现有节点

当前实际节点位于：

- [k210_parser_node.py](F:\stm32_Balance_Car_L\ros2_ws\src\balance_car_perception\balance_car_perception\k210_parser_node.py)

这个节点已经承担：

- 轮询 `K210` 上行状态
- 发布 ROS 话题
- 接收 ROS 下行指令并调用 backend

### 7.2 启动方式

当前完整系统启动文件：

- [system.launch.py](F:\stm32_Balance_Car_L\ros2_ws\src\balance_car_bringup\launch\system.launch.py)

启动后会自动带起：

- `balance_car_k210_parser`

单独运行命令：

```bash
ros2 run balance_car_perception k210_parser_node
```

完整启动命令：

```bash
ros2 launch balance_car_bringup system.launch.py
```

## 8. 最小联调流程

建议按这个顺序验证：

1. 确认树莓派 backend 正常：

```bash
python3 /home/lifng0/workspace/balance_car/scripts/pi_backend_client.py get-k210-link
```

2. 启动 ROS 节点后看链路话题：

```bash
ros2 topic echo /vision/k210_link_json
```

3. 从 ROS 向 `K210` 发 `PING`：

```bash
ros2 topic pub --once /vision/k210_command std_msgs/msg/String "{data: 'PING'}"
```

4. 看命令结果：

```bash
ros2 topic echo /vision/k210_command_result_json
```

5. 看 `K210` 上行回包：

```bash
ros2 topic echo /vision/status_json
```

如果一切正常，应看到：

- `K210 LCD` 显示变化
- `/vision/k210_command_result_json` 返回 `ok=true`
- `/vision/status_json` 最终出现 `PONG` 或 `SHOW_OK:...`

## 9. 常见问题

### 9.1 `/vision/k210_command_result_json` 是成功，但 `/vision/status_json` 没反应

优先判断：

- `K210` 脚本是否真的在跑
- `K210` 是否只收不回
- `K210` 返回的是不是普通文本且没有换行

### 9.2 `/vision/k210_link_json` 里 `k210_ready=true`，但 `k210_last_rx_at=0`

说明：

- 树莓派已经成功打开串口
- 但到目前为止还没收到任何一帧来自 `K210` 的数据

这通常意味着：

- `K210` 没有发送上行消息
- `TX/RX` 接反
- `K210` 脚本只做显示、没做回传

### 9.3 想给正式视觉算法留接口

建议：

- 上行统一发 `$...#`
- 把业务类型编码到文本头里，例如：
  - `QR:...`
  - `LINE:...`
  - `FOLLOW:...`
- ROS 算法节点只依赖 `/vision/status_json`
- `K210` 控制节点只依赖 `/vision/k210_command`

这样后续替换 `K210` 算法时，不需要改 ROS 主体结构。
