#0x0209 topic:/robot/rfidstatus  有修改
#bit 位值为 1/0 的含义：是否已检测到该增益点 RFID 卡
uint32 base                    #己方基地增益点
uint32 our_central_plateau     #己方中央高地增益点
uint32 his_central_plateau     #对方中央高地增益点
uint32 our_trapezoidal_heights #己方梯形高地增益点
uint32 his_trapezoidal_heights #对方梯形高地增益点
uint32 our_cross_feipo_front   #己方地形跨越增益点（飞坡）（靠近己方一侧飞坡前）
uint32 our_cross_feipo_back    #己方地形跨越增益点（飞坡）（靠近己方一侧飞坡后）
uint32 his_cross_feipo_front   #对方地形跨越增益点（飞坡）（靠近对方一侧飞坡前）
uint32 his_cross_feipo_back    #对方地形跨越增益点（飞坡）（靠近对方一侧飞坡后）
uint32 our_cross_central_below #己方地形跨越增益点（中央高地下方）
uint32 our_cross_central_up    #己方地形跨越增益点（中央高地上方）
uint32 his_cross_central_below #对方地形跨越增益点（中央高地下方）
uint32 his_cross_central_up    #对方地形跨越增益点（中央高地上方）
uint32 our_cross_road_below    #己方地形跨越增益点（公路下方）
uint32 our_cross_road_up       #己方地形跨越增益点（公路上方）
uint32 his_cross_road_below    #对方地形跨越增益点（公路下方）
uint32 his_cross_road_up       #对方地形跨越增益点（公路上方）
uint32 our_fort                #己方堡垒增益点
uint32 outpost                 #己方前哨站增益点
uint32 inside_recovery         #己方与兑换区不重叠的补给区/RMUL 补给区
uint32 outside_recovery        #己方与兑换区重叠的补给区
uint32 our_big_resources       #己方大资源岛增益点
uint32 his_big_resources       #对方大资源岛增益点
uint32 center                  #中心增益点（仅 RMUL 适用）
uint32 his_fort                #对方堡垒增益点