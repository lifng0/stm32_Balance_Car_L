# 结构化视觉目标 (structured vision target)
# 字段归属见 doc/架构对齐-先决条件基线.md (对应总方案 8.2 / 8.3)
string target_type
bool detected
float32 x
float32 y
float32 confidence
string extra

# --- 先决条件阶段新增:颜色跟踪锁距所需,替代 JSON 调试字段 ---
float32 target_width       # 目标像素宽度(距离代理量,非真实距离估计)
float32 cx_center          # 目标中心横向像素坐标
