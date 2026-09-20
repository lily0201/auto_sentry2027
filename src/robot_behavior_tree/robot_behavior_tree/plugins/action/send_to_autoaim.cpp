#include <string>

#include "robot_behavior_tree/plugins/action/send_to_autoaim.hpp"

namespace nav2_behavior_tree
{
    SendToAutoaimAction::SendToAutoaimAction(
         const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf)
          // is_we_are_blue(true)
    {
        // enemy_hp = std::make_shared<rm_interfaces::msg::Hp>();
        // lidarstation_info = std::make_shared<rm_interfaces::msg::Lidarstation>();
        // enemy_position = std::make_shared<rm_interfaces::msg::EnemyPosition>();
        //
        // config().blackboard->get<bool>("is_we_are_blue", is_we_are_blue);
        //
        // node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        // callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        // callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
        // rclcpp::SubscriptionOptions sub_option;
        // sub_option.callback_group = callback_group_;
        //
        // enemy_position_pub_ = node_->create_publisher<rm_interfaces::msg::EnemyPosition>("/EnemyPosition", 10);
        // lidarstation_sub_ = node_->create_subscription<rm_interfaces::msg::Lidarstation>(
        //     "/robot/lidarstation",
        //     rclcpp::SystemDefaultsQoS(),
        //     std::bind(&SendToAutoaimAction::lidarstationCallback, this, std::placeholders::_1),
        //     sub_option
        //     );
        // hp_sub_ = node_->create_subscription<rm_interfaces::msg::Hp>(
        //     "/robot/hp",
        //     rclcpp::SystemDefaultsQoS(),
        //     std::bind(&SendToAutoaimAction::hpCallback, this, std::placeholders::_1),
        //     sub_option
        //     );
    }

    BT::NodeStatus SendToAutoaimAction::tick()
    {
        // config().blackboard->get<bool>("is_we_are_blue", is_we_are_blue);
        // callback_group_executor_.spin_some();
        //
        // for(int i = 0; i < 5; i++)
        // {
        //     rm_interfaces::msg::EnemyPositions enemy;
        //
        //     enemy.header.stamp = node_->get_clock()->now();
        //     if (i == 0)
        //     {
        //         enemy.id = "1";
        //         enemy.x = lidarstation_info->lidarpositions[i].x;
        //         enemy.y = lidarstation_info->lidarpositions[i].y;
        //         enemy.z = lidarstation_info->lidarpositions[i].z;
        //         if(is_we_are_blue)
        //             enemy.hp = enemy_hp->red_1_robot_hp;
        //         else
        //             enemy.hp = enemy_hp->blue_1_robot_hp;
        //     }
        //     else if (i == 1)
        //     {
        //         enemy.id = "2";
        //         enemy.x = lidarstation_info->lidarpositions[i].x;
        //         enemy.y = lidarstation_info->lidarpositions[i].y;
        //         enemy.z = lidarstation_info->lidarpositions[i].z;
        //         if(is_we_are_blue)
        //             enemy.hp = enemy_hp->red_2_robot_hp;
        //         else
        //             enemy.hp = enemy_hp->blue_2_robot_hp;
        //     }
        //     else if (i == 2)
        //     {
        //         enemy.id = "3";
        //         enemy.x = lidarstation_info->lidarpositions[i].x;
        //         enemy.y = lidarstation_info->lidarpositions[i].y;
        //         enemy.z = lidarstation_info->lidarpositions[i].z;
        //         if(is_we_are_blue)
        //             enemy.hp = enemy_hp->red_3_robot_hp;
        //         else
        //             enemy.hp = enemy_hp->blue_3_robot_hp;
        //     }
        //     else if (i == 3)
        //     {
        //         enemy.id = "4";
        //         enemy.x = lidarstation_info->lidarpositions[i].x;
        //         enemy.y = lidarstation_info->lidarpositions[i].y;
        //         enemy.z = lidarstation_info->lidarpositions[i].z;
        //         if(is_we_are_blue)
        //             enemy.hp = enemy_hp->red_4_robot_hp;
        //         else
        //             enemy.hp = enemy_hp->blue_4_robot_hp;
        //     }
        //     else if (i == 4)
        //     {
        //         enemy.id = "sentry";
        //         enemy.x = lidarstation_info->lidarpositions[i].x;
        //         enemy.y = lidarstation_info->lidarpositions[i].y;
        //         enemy.z = lidarstation_info->lidarpositions[i].z;
        //         if(is_we_are_blue)
        //             enemy.hp = enemy_hp->red_7_robot_hp;
        //         else
        //             enemy.hp = enemy_hp->blue_7_robot_hp;
        //     }
        //     if(enemy.x == 0.0 && enemy.y == 0.0 && enemy.z == 0.0)
        //         enemy.attack_enhance = false;
        //     else
        //         enemy.attack_enhance = true;
        //
        //     enemy_position->enemies.push_back(enemy);
        // }
        // enemy_position_pub_->publish(*enemy_position);
        // RCLCPP_INFO(node_->get_logger(), "发送5辆车的数据给自瞄");

        return BT::NodeStatus::SUCCESS;
    }

    // void SendToAutoaimAction::hpCallback(const rm_interfaces::msg::Hp::ConstPtr& msg)
    // {
    //     enemy_hp->blue_1_robot_hp = msg->blue_1_robot_hp;
    //     enemy_hp->blue_2_robot_hp = msg->blue_2_robot_hp;
    //     enemy_hp->blue_3_robot_hp = msg->blue_3_robot_hp;
    //     enemy_hp->blue_4_robot_hp = msg->blue_4_robot_hp;
    //     enemy_hp->blue_7_robot_hp = msg->blue_7_robot_hp;
    //     enemy_hp->blue_base_hp = msg->blue_base_hp;
    //     enemy_hp->blue_outpost_hp = msg->blue_outpost_hp;
    //     enemy_hp->red_1_robot_hp = msg->red_1_robot_hp;
    //     enemy_hp->red_2_robot_hp = msg->red_2_robot_hp;
    //     enemy_hp->red_3_robot_hp = msg->red_3_robot_hp;
    //     enemy_hp->red_4_robot_hp = msg->red_4_robot_hp;
    //     enemy_hp->red_7_robot_hp = msg->red_7_robot_hp;
    //     enemy_hp->red_base_hp = msg->red_base_hp;
    //     enemy_hp->red_outpost_hp = msg->red_outpost_hp;
    // }

    // void SendToAutoaimAction::lidarstationCallback(const rm_interfaces::msg::Lidarstation::ConstPtr& msg)
    // {
    //     lidarstation_info->attack_enhance = msg->attack_enhance;
    //     lidarstation_info->dart_open = msg->dart_open;
    //     for(int i = 0; i < 4; i++)
    //         lidarstation_info->dangerous_tunnel[i] = msg->dangerous_tunnel[i];
    //     for(int i = 0; i < 5; i++)
    //     {
    //         lidarstation_info->lidarpositions[i].x = msg->lidarpositions[i].x;
    //         lidarstation_info->lidarpositions[i].y = msg->lidarpositions[i].y;
    //         lidarstation_info->lidarpositions[i].z = msg->lidarpositions[i].z;
    //     }
    // }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::SendToAutoaimAction>("SendToAutoaim");
}
