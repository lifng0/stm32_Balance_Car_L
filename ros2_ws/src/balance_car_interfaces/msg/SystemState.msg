# 系统级状态 (system-level state)
# 字段归属见 doc/架构对齐-先决条件基线.md (对应总方案 8.2)
bool pi_ready
bool lidar_ready
bool system_ready
bool paused_by_pickup
bool running
bool shutting_down         # JSON 兼容字段名 shutdown_started

# --- 先决条件阶段新增:替代 JSON 中已被业务依赖的字段 ---
uint8 system_mode          # 系统编排模式(START/STOP/SHUTDOWN 语义),区别于底盘 mode
string ros_ready_reason    # ROS 就绪/未就绪原因,供诊断
bool chassis_online        # STM32 串口在线
bool vision_online         # K210 在线
