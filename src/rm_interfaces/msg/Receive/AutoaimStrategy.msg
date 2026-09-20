# topic:/robot/strategy 用于将选车逻辑中需要用到的内容从行为树中发布到另一个节点中
# 0-6依次为英雄、工程、步兵3、步兵4、哨兵、前哨站、基地
uint8[7] whitelist #白名单
bool[7] enemy_invincible #是否无敌
uint16[7] enemy_blood #血量