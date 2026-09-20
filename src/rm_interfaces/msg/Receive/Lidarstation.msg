#topic:/robot/lidarstation 0x0301 子内容
uint8 attack_enhance #是否开启双倍易伤
Lidarposition[5] lidarpositions #雷达发来的场上五辆车的坐标信息，依次为英雄，工程，步兵1,步兵2,哨兵
                                #若雷达识别到该车则发送该车位置，未识别到时全部为0