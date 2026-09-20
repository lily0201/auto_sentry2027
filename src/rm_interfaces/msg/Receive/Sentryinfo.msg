#0x020D topic:/robot/sentryinfo
uint32 amount_of_get_ammunition#除远程兑换外，哨兵成功兑换的发弹量，开局为 0，在哨兵成功兑换一定发弹量后，该值将变为哨兵成功兑换的发弹量值
uint32 number_of_get_ammunition#哨兵成功远程兑换发弹量的次数，开局为 0，在哨兵成功远程兑换发弹量后，该值将变为哨兵成功远程兑换发弹量的次数
uint32 number_of_get_blood#哨兵成功远程兑换血量的次数，开局为 0，在哨兵成功远程兑换血量后，该值将变为哨兵成功远程兑换血量的次数。
uint32 confirm_free_resurrection#哨兵机器人当前是否可以确认免费复活，可以确认免费复活时值为1，否则为 0。
uint32 confirm_resurrection_immediately#哨兵机器人当前是否可以兑换立即复活，可以兑换立即复活时值为1，否则为 0。
uint32 cost_of_resurrection#哨兵机器人当前若兑换立即复活需要花费的金币数。
uint32 if_out_fight#哨兵当前是否处于脱战状态，处于脱战状态时为 1，否则为 0。
uint32 projectile_allowance_17mm_of_team#队伍 17mm 允许发弹量的剩余可兑换数。
