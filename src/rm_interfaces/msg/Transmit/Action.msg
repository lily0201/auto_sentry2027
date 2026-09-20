#topic:/robot/action 0x0502
#相对于雷达坐标系，规定逆时针为正，顺时针为负，弧度制
float32 max_pitch_w #pitch最大仰角
float32 min_pitch_w #pitch最小俯角
uint8   mode #0：导航控底盘 1：导航控云台
uint8   spin #0：不小陀螺 1：小陀螺
uint8   patrol # 0：云台不巡逻 1：云台巡逻
uint8   robot_aim # 0：不自瞄 1：自瞄（原地不追击）2: 打符模式
uint8   chasing # 0：不在追击状态 1：在追击状态