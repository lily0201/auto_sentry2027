#0x0307 topic:/robot/map
#哨兵机器人或选择半自动控制方式的机器人可通过常规链路向对应的操作手选手端发送路径坐标数据，该路径会在小地图上显示
uint8 intention  #1：到目标点攻击 2：到目标点防守 3：移动到目标点
uint16 start_position_x # 路径起点 x 轴坐标，单位：dm
uint16 start_position_y # 路径起点 y 轴坐标，单位：dm
#增量相较于上一个点位进行计算，共 49 个新点位，X 与 Y 轴增量对应组成点位
int8[49] delta_x # 路径点 x 轴增量数组，单位：dm ，长度为 49
int8[49] delta_y # 路径点 y 轴增量数组，单位：dm ，长度为 49
uint16 sender_id #发送者 ID,需与自身 ID 匹配
