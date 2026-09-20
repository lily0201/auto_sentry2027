//
// Created by elsa on 25-3-27.
//

#include <string>

#include "robot_behavior_tree/plugins/action/send_whitelist.hpp"

namespace nav2_behavior_tree
{

    SendWhitelistAction::SendWhitelistAction(
         const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf)
    {
        config().blackboard->get<rclcpp::Node::SharedPtr>("node",node_);
        whitelist_pub_ = node_->create_publisher<rm_interfaces::msg::Whitelist>("/robot/whitelist", 10);

        config().blackboard->get<bool>("red_hero_invincible", red_hero_invincible);
        config().blackboard->get<bool>("red_engineer_invincible", red_engineer_invincible);
        config().blackboard->get<bool>("red_infantry3_invincible", red_infantry3_invincible);
        config().blackboard->get<bool>("red_infantry4_invincible", red_infantry4_invincible);
        config().blackboard->get<bool>("red_sentry_invincible", red_sentry_invincible);
        config().blackboard->get<bool>("blue_hero_invincible", blue_hero_invincible);
        config().blackboard->get<bool>("blue_engineer_invincible", blue_engineer_invincible);
        config().blackboard->get<bool>("blue_infantry3_invincible", blue_infantry3_invincible);
        config().blackboard->get<bool>("blue_infantry4_invincible", blue_infantry4_invincible);
        config().blackboard->get<bool>("blue_sentry_invincible", blue_sentry_invincible);
        config().blackboard->get<bool>("is_we_are_blue", is_we_are_blue);

        config().blackboard->get<uint8_t>("whitelist_hero", whitelist_[1]);
        config().blackboard->get<uint8_t>("whitelist_engineer", whitelist_[2]);
        config().blackboard->get<uint8_t>("whitelist_infantry3", whitelist_[3]);
        config().blackboard->get<uint8_t>("whitelist_infantry4", whitelist_[4]);
        config().blackboard->get<uint8_t>("whitelist_sentry", whitelist_[7]);
        config().blackboard->get<uint8_t>("whitelist_outpost", whitelist_[10]);
        config().blackboard->get<uint8_t>("whitelist_base", whitelist_[11]);
    }

    BT::NodeStatus SendWhitelistAction::tick()
    {
        config().blackboard->get<bool>("red_hero_invincible", red_hero_invincible);
        config().blackboard->get<bool>("red_engineer_invincible", red_engineer_invincible);
        config().blackboard->get<bool>("red_infantry3_invincible", red_infantry3_invincible);
        config().blackboard->get<bool>("red_infantry4_invincible", red_infantry4_invincible);
        config().blackboard->get<bool>("red_sentry_invincible", red_sentry_invincible);
        config().blackboard->get<bool>("blue_hero_invincible", blue_hero_invincible);
        config().blackboard->get<bool>("blue_engineer_invincible", blue_engineer_invincible);
        config().blackboard->get<bool>("blue_infantry3_invincible", blue_infantry3_invincible);
        config().blackboard->get<bool>("blue_infantry4_invincible", blue_infantry4_invincible);
        config().blackboard->get<bool>("blue_sentry_invincible", blue_sentry_invincible);
        config().blackboard->get<bool>("is_we_are_blue", is_we_are_blue);

        config().blackboard->get<uint8_t>("whitelist_hero", whitelist_[1]);
        config().blackboard->get<uint8_t>("whitelist_engineer", whitelist_[2]);
        config().blackboard->get<uint8_t>("whitelist_infantry3", whitelist_[3]);
        config().blackboard->get<uint8_t>("whitelist_infantry4", whitelist_[4]);
        config().blackboard->get<uint8_t>("whitelist_sentry", whitelist_[7]);
        config().blackboard->get<uint8_t>("whitelist_outpost", whitelist_[10]);
        config().blackboard->get<uint8_t>("whitelist_base", whitelist_[11]);

        rm_interfaces::msg::Whitelist whitelist;
        for(int i = 0; i < 10; i++)
            whitelist.robot[i] = 0;

        if(is_we_are_blue) //蓝方,敌方为红方
        {
            whitelist.robot[1] = whitelist_[1] * (uint8_t)(!red_hero_invincible);
            whitelist.robot[2] = whitelist_[2] * (uint8_t)(!red_engineer_invincible);
            whitelist.robot[3] = whitelist_[3] * (uint8_t)(!red_infantry3_invincible);
            whitelist.robot[4] = whitelist_[4] * (uint8_t)(!red_infantry4_invincible);
            whitelist.robot[6] = whitelist_[7] * (uint8_t)(!red_sentry_invincible);
        }
        else //红方,敌方为蓝方
        {
            whitelist.robot[1] = whitelist_[1] * (uint8_t)(!blue_hero_invincible);
            whitelist.robot[2] = whitelist_[2] * (uint8_t)(!blue_engineer_invincible);
            whitelist.robot[3] = whitelist_[3] * (uint8_t)(!blue_infantry3_invincible);
            whitelist.robot[4] = whitelist_[4] * (uint8_t)(!blue_infantry4_invincible);
            whitelist.robot[6] = whitelist_[7] * (uint8_t)(!blue_sentry_invincible);
        }
        whitelist.robot[7] = whitelist_[10]; //前哨站
        whitelist.robot[8] = whitelist_[11]; //基地

        RCLCPP_INFO(node_->get_logger(), "send whitelist 1~4, sentry: %hhu, %hhu, %hhu, %hhu, %hhu",
            whitelist.robot[1], whitelist.robot[2], whitelist.robot[3], whitelist.robot[4], whitelist.robot[6]);
        RCLCPP_INFO(node_->get_logger(), "send whitelist outpost, base: %hhu, %hhu",
            whitelist.robot[7], whitelist.robot[8]);
        whitelist_pub_->publish(whitelist);

        return BT::NodeStatus::SUCCESS;
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::SendWhitelistAction>("SendWhitelist");
}
