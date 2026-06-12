# 底盘实时状态 (chassis runtime state)
# 字段归属见 doc/架构对齐-先决条件基线.md (对应总方案 8.2)
uint8 mode
string mode_name          # 与 modes.yaml 一致的模式名;替代 JSON 中的 mode_name
bool stop_flag
bool low_power
float32 move_x
float32 move_z
float32 battery
float32 angle
