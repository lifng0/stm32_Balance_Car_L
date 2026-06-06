import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class LidarAvoidNode(Node):
    def __init__(self):
        super().__init__('lidar_avoid_node')

        # 严格对齐接口文档 7.2 节的避障核心参数，完全满足你“高于雷达高度才处理”的平面逻辑
        self.warning_distance = 0.80       # 开始预警减速的距离（0.8米）
        self.stop_distance = 0.45          # 触发紧急刹车的危险红线（0.45米）
        
        # 调试期模拟的底盘控制档位速度
        self.forward_speed = 8.0           # 正常前进
        self.slow_speed = 4.0              # 减速前进
        self.turn_speed = 8.0              # 原地转身速度

        # 订阅队友写好的雷达服务发布的唯一 JSON 摘要话题
        self.subscription = self.create_subscription(
            String,
            '/lidar/summary_json',
            self.lidar_callback,
            10
        )
        self.get_logger().info("🚀 [避障大脑已就绪] 正在全神贯注监听 /lidar/summary_json 的变化...")

    def lidar_callback(self, msg):
        """核心决策流：减速 -> 刹车 -> 换向"""
        try:
            # 1. 解包队友雷达发来的数据
            data = json.loads(msg.data or "{}")
            
            # 如果雷达当前无效（比如被队友手挡住了导致死区），处于安全考虑直接静止
            if not data.get("scan_ok", False):
                self.get_logger().info("⏳ 雷达数据不可用或正在重启，小车保持原地待命。")
                return

            # 2. 读取文档规定的三个方向的雷达水平面距离（单位：米）
            front = data.get("front_min_distance_m", 10.0)
            front_left = data.get("front_left_min_distance_m", 10.0)
            front_right = data.get("front_right_min_distance_m", 10.0)

            # 状态控制判断：
            # 【情况 1：正常前进】
            if front > self.warning_distance:
                self.get_logger().info(f"🟢 道路安全 (前方距离:{front:.2f}m) -> 执行：正常速度前进 [{self.forward_speed}]")

            # 【情况 2：进入预警，缓慢蹭着走】
            elif self.stop_distance < front <= self.warning_distance:
                self.get_logger().info(f"🟡 接近障碍 (前方距离:{front:.2f}m) -> 执行：减速缓慢前进 [{self.slow_speed}]")

            # 【情况 3：离得太近了！触发你的专属修正：先刹车，再换向！】
            elif front <= self.stop_distance:
                self.get_logger().warn(f"🔴 危险！前方距离({front:.2f}m)已踩红线！激活保护策略：")
                
                # 第一步：物理刹车（这里打印出动作，后续配合队友的脚本真正控车）
                self.get_logger().warn("   👉 [第一步：紧急刹车]：速度强制归零 0.0，稳住重心，消除向前惯性！")
                
                # 第二步：比对左右两边哪边空旷，往更空的一边给转向速度
                if front_left >= front_right:
                    self.get_logger().info(f"   👉 [第二步：左侧较空] (左:{front_left:.2f}m >= 右:{front_right:.2f}m) -> 决定：向左原地旋转转身")
                else:
                    self.get_logger().info(f"   👉 [第二步：右侧较空] (右:{front_right:.2f}m > 左:{front_left:.2f}m) -> 决定：向右原地旋转转身")

        except Exception as e:
            self.get_logger().error(f"大脑决策发生非预期错误: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LidarAvoidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
