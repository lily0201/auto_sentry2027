//
// Created by elsa on 25-5-1.
//

#include <string>

#include "robot_behavior_tree/plugins/action/record_autoaim.hpp"

namespace nav2_behavior_tree
{
    RecordAutoaimAction::RecordAutoaimAction(
        const std::string& action_name,
        const BT::NodeConfiguration& conf)
        : BT::SyncActionNode(action_name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        autoaim_strategy_pub_ = node_->create_publisher<rm_interfaces::msg::AutoaimStrategy>("/robot/strategy", 10);

        get_blackboard_value();
    }

    BT::NodeStatus RecordAutoaimAction::tick()
    {
        get_blackboard_value();
        for(int i = 0; i < 7; i++)
        {
            autoaim_strategy_.whitelist[i] = whitelist[i];
            autoaim_strategy_.enemy_blood[i] = enemy_blood[i];
            autoaim_strategy_.enemy_invincible[i] = enemy_invincible[i];
        }

        autoaim_strategy_pub_->publish(autoaim_strategy_);

        RCLCPP_INFO(node_->get_logger(), "record robot autoaim successfully");

        return BT::NodeStatus::SUCCESS;
    }

    void RecordAutoaimAction::get_blackboard_value()
    {
        config().blackboard->get<bool>("is_we_are_blue", is_we_are_blue);
        config().blackboard->get<uint8_t>("whitelist_hero", whitelist[0]);
        config().blackboard->get<uint8_t>("whitelist_engineer", whitelist[1]);
        config().blackboard->get<uint8_t>("whitelist_infantry3", whitelist[2]);
        config().blackboard->get<uint8_t>("whitelist_infantry4", whitelist[3]);
        config().blackboard->get<uint8_t>("whitelist_sentry", whitelist[4]);
        config().blackboard->get<uint8_t>("whitelist_outpost", whitelist[5]);
        config().blackboard->get<uint8_t>("whitelist_base", whitelist[6]);
        if(is_we_are_blue)
        {
            config().blackboard->get<int>("red_hero_blood", enemy_blood[0]);
            config().blackboard->get<int>("red_engineer_blood", enemy_blood[1]);
            config().blackboard->get<int>("red_infantry3_blood", enemy_blood[2]);
            config().blackboard->get<int>("red_infantry4_blood", enemy_blood[3]);
            config().blackboard->get<int>("red_sentry_blood", enemy_blood[4]);
            config().blackboard->get<int>("red_outpost_blood", enemy_blood[5]);
            config().blackboard->get<int>("red_base_blood", enemy_blood[6]);
            config().blackboard->get<bool>("red_hero_invincible", enemy_invincible[0]);
            config().blackboard->get<bool>("red_engineer_invincible", enemy_invincible[1]);
            config().blackboard->get<bool>("red_infantry3_invincible", enemy_invincible[2]);
            config().blackboard->get<bool>("red_infantry4_invincible", enemy_invincible[3]);
            config().blackboard->get<bool>("red_sentry_invincible", enemy_invincible[4]);
            config().blackboard->get<bool>("red_outpost_invincible", enemy_invincible[5]);
            config().blackboard->get<bool>("red_base_invincible", enemy_invincible[6]);
        }
        else
        {
            config().blackboard->get<int>("blue_hero_blood", enemy_blood[0]);
            config().blackboard->get<int>("blue_engineer_blood", enemy_blood[1]);
            config().blackboard->get<int>("blue_infantry3_blood", enemy_blood[2]);
            config().blackboard->get<int>("blue_infantry4_blood", enemy_blood[3]);
            config().blackboard->get<int>("blue_sentry_blood", enemy_blood[4]);
            config().blackboard->get<int>("blue_outpost_blood", enemy_blood[5]);
            config().blackboard->get<int>("blue_base_blood", enemy_blood[6]);
            config().blackboard->get<bool>("blue_hero_invincible", enemy_invincible[0]);
            config().blackboard->get<bool>("blue_engineer_invincible", enemy_invincible[1]);
            config().blackboard->get<bool>("blue_infantry3_invincible", enemy_invincible[2]);
            config().blackboard->get<bool>("blue_infantry4_invincible", enemy_invincible[3]);
            config().blackboard->get<bool>("blue_sentry_invincible", enemy_invincible[4]);
            config().blackboard->get<bool>("blue_outpost_invincible", enemy_invincible[5]);
            config().blackboard->get<bool>("blue_base_invincible", enemy_invincible[6]);
        }
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::RecordAutoaimAction>("RecordAutoaim");
}
