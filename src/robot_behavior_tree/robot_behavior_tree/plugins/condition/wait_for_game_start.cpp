#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "robot_behavior_tree/plugins/condition/wait_for_game_start.hpp"

namespace nav2_behavior_tree
{

    WaitForGameStartCondition::WaitForGameStartCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
        gamestatus(0)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        gamestatus_sub_ = node_->create_subscription<rm_interfaces::msg::Gamestatus>(
            "/robot/gamestatus",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&WaitForGameStartCondition::gamestatusCallback, this, std::placeholders::_1),
            sub_option);
        RCLCPP_INFO(node_->get_logger(), "等待比赛开始");
        bt_pub_ = node_->create_publisher<std_msgs::msg::Header>("/bt_decision", 1);

        dangerous_tunnel.resize(4);
        for(int i = 0; i < 4; i++)
            dangerous_tunnel[i] = false;
    }

    BT::NodeStatus WaitForGameStartCondition::tick()
    {
        std_msgs::msg::Header header;
        header.stamp = node_->now();
        bt_pub_ -> publish(header);
        callback_group_executor_.spin_some();
        if (gamestatus == 4)
        {
            //RCLCPP_INFO(node_->get_logger(), "比赛开始");
            return BT::NodeStatus::SUCCESS;
        }
        getInput("if_dafu", if_dafu);
        config().blackboard->set("if_dafu", if_dafu);
        if(gamestatus == 0)
        {
            RCLCPP_INFO(node_->get_logger(), "未开始比赛");
        }
        else if(gamestatus == 1)
        {
            RCLCPP_INFO(node_->get_logger(), "准备阶段");
        }
        else if(gamestatus == 2)
        {
            RCLCPP_INFO(node_->get_logger(), "十五秒裁判系统自检阶段");
        }
        else if(gamestatus == 3)
        {
            RCLCPP_INFO(node_->get_logger(), "五秒倒计时");
        }
        else if(gamestatus == 5)
        {
            RCLCPP_INFO(node_->get_logger(), "比赛结算中");
            //所有blackboard恢复初始化
            config().blackboard->set<uint32_t>("if_in_emergency", false);
            config().blackboard->set<float>("max_pitch", 0.0);
            config().blackboard->set<float>("min_pitch", 0.0);
            config().blackboard->set<int>("mode", 0); // 0--导航，1--巡逻
            config().blackboard->set<int>("spin", 0); // 0--不小陀螺，1--小陀螺
            config().blackboard->set<int>("patrol", 1); // 0--云台不巡逻，1--云台巡逻
            config().blackboard->set<int>("decision_mode", 0); //当前任务模式
            config().blackboard->set<int>("robot_aim", 1); //是否自瞄，0--不自瞄，1--自瞄

            config().blackboard->set<uint32_t>("resurrection", 0); //选择当前复活方式
            config().blackboard->set<bool>("is_cancel_resurrection", false);
            config().blackboard->set<uint16_t>("allowance_shoot_remaining", 300); //当前剩余发弹量
            config().blackboard->set<uint32_t>("buy_bullet_remote_number", 0); //当前记录的远程买弹的总次数
            config().blackboard->set<uint32_t>("buy_bullet_at_recovery", 0); //当前记录的在补给区买弹的总弹量
            config().blackboard->set<uint32_t>("buy_blood_number", 0); //当前记录的远程买血的总次数
            config().blackboard->set<uint32_t>("number_of_bullet_to_buy", 0); //需要在补给区购买的发弹量
            config().blackboard->set<uint16_t>("recovery_bullet_to_acquire", 0); //补给区当前可领的免费发弹量
            config().blackboard->set<uint16_t>("recovery_bullet_acquired", 0); //补给区已经领取的免费发弹量

            config().blackboard->set<bool>("is_hurt", false);
            config().blackboard->set<bool>("is_out_fight", true);
            config().blackboard->set<bool>("is_in_ramp", false); //是否在坡道
            config().blackboard->set<bool>("is_in_chase", false);
            config().blackboard->set<bool>("if_waiting_buy_blood", false);
            config().blackboard->set<bool>("if_waiting_buy_bullet_remote", false);
            config().blackboard->set<bool>("if_waiting_buy_bullet_at_recovery", false);
            config().blackboard->set<double>("distance", 0.0);

            config().blackboard->set<bool>("is_goal_reached", false); //是否到达目标点
            config().blackboard->set<bool>("is_part_of_referee_offline", false); //是否有裁判系统模块离线

            /* 云台手控制内容 */
            config().blackboard->set<bool>("in_command", false);
            config().blackboard->set<std::string>("cmd_keyboard", "");
            config().blackboard->set<bool>("is_force_move_forward", false); //强制前进
            config().blackboard->set<bool>("is_force_move_backward", false); //强制后退
            config().blackboard->set<bool>("is_force_move_left", false); //强制向左
            config().blackboard->set<bool>("is_force_move_right", false); //强制向右
            config().blackboard->set<bool>("is_force_move_complete", true);
            config().blackboard->set<bool>("select_if_go_fort", true);
            config().blackboard->set<bool>("select_force_no_resurrection", false);

            config().blackboard->set<uint16_t>("receiver_id", 0);
            config().blackboard->set<uint16_t>("sender_id", 0);

            pose.header.stamp = node_->now();
            pose.header.frame_id = "map";
            pose.pose.position.x = 0.0;
            pose.pose.position.y = 0.0;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            config().blackboard->set<geometry_msgs::msg::PoseStamped>("goal", pose);

            /* 红蓝方机器人血量 */
            config().blackboard->set<int>("red_hero_blood", 100);
            config().blackboard->set<int>("red_engineer_blood", 300);
            config().blackboard->set<int>("red_infantry3_blood", 100);
            config().blackboard->set<int>("red_infantry4_blood", 100);
            config().blackboard->set<int>("red_sentry_blood", 400);
            config().blackboard->set<int>("red_outpost_blood", 1500);
            config().blackboard->set<int>("red_base_blood", 5000);
            config().blackboard->set<int>("blue_hero_blood", 100);
            config().blackboard->set<int>("blue_engineer_blood", 300);
            config().blackboard->set<int>("blue_infantry3_blood", 100);
            config().blackboard->set<int>("blue_infantry4_blood", 100);
            config().blackboard->set<int>("blue_sentry_blood", 400);
            config().blackboard->set<int>("blue_outpost_blood", 1500);
            config().blackboard->set<int>("blue_base_blood", 5000);

            /* 红蓝方机器人是否在无敌状态 */
            config().blackboard->set<bool>("red_hero_invincible", false);
            config().blackboard->set<bool>("red_engineer_invincible", false);
            config().blackboard->set<bool>("red_infantry3_invincible", false);
            config().blackboard->set<bool>("red_infantry4_invincible", false);
            config().blackboard->set<bool>("red_sentry_invincible", false);
            config().blackboard->set<bool>("red_outpost_invincible", false);
            config().blackboard->set<bool>("red_base_invincible", false);
            config().blackboard->set<bool>("blue_hero_invincible", false);
            config().blackboard->set<bool>("blue_engineer_invincible", false);
            config().blackboard->set<bool>("blue_infantry3_invincible", false);
            config().blackboard->set<bool>("blue_infantry4_invincible", false);
            config().blackboard->set<bool>("blue_sentry_invincible", false);
            config().blackboard->set<bool>("blue_outpost_invincible", false);
            config().blackboard->set<bool>("blue_base_invincible", false);

            // 双方飞镖是否击中基地
            config().blackboard->set<bool>("is_dart_hit_his", false);
            config().blackboard->set<bool>("is_dart_hit_our", false);
            config().blackboard->set<int>("our_dart_hit_count", 0);
            config().blackboard->set<int>("his_dart_hit_count", 0);

            config().blackboard->set<int>("health_threshold", 80);

            /* 白名单 */
            config().blackboard->set<uint8_t>("whitelist_hero", 1);
            config().blackboard->set<uint8_t>("whitelist_engineer", 1);
            config().blackboard->set<uint8_t>("whitelist_infantry3", 1);
            config().blackboard->set<uint8_t>("whitelist_infantry4", 1);
            config().blackboard->set<uint8_t>("whitelist_sentry", 1);
            config().blackboard->set<uint8_t>("whitelist_outpost", 1);
            config().blackboard->set<uint8_t>("whitelist_base", 1);

            //路径上经过的狗洞标号
            config().blackboard->set<int>("tunnel_index", 0);
            //当前是否在狗洞中
            config().blackboard->set<bool>("if_in_tunnel", false);
            //狗洞是否有人堵
            config().blackboard->set<std::vector<bool>>("dangerous_tunnel", dangerous_tunnel);
            //和狗洞可通过情况相关的地图
            config().blackboard->set<std::string>("tunnel_related_map", "");

            //是否进入虚弱状态
            config().blackboard->set<bool>("is_in_weak", false);

            //是否已经死亡过一次
            config().blackboard->set<std::string>("dead_count", "0");

            /* 追击相关 */
            config().blackboard->set<double>("enemy_position_x", 0.0); //追击敌人坐标
            config().blackboard->set<double>("enemy_position_y", 0.0); //追击敌人坐标
            config().blackboard->set<bool>("is_in_chase", false); //是否在受自瞄控制目标点的追击状态

            RCLCPP_INFO(node_->get_logger(), "所有参数已重置");
        }
        return BT::NodeStatus::FAILURE;
    }

    void WaitForGameStartCondition::gamestatusCallback(rm_interfaces::msg::Gamestatus::SharedPtr msg)
    {
        gamestatus = msg->game_progress;
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::WaitForGameStartCondition>("WaitForGameStart");
}
