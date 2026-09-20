//
// Created by elsa on 25-3-27.
//

#include <string>

#include "robot_behavior_tree/plugins/action/set_whitelist.hpp"
namespace nav2_behavior_tree
{
    SetWhitelistAction::SetWhitelistAction(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf)
    {
        config().blackboard->get<rclcpp::Node::SharedPtr>("node",node_);

        getInput("hero", whitelist[1]);
        getInput("engineer", whitelist[2]);
        getInput("infantry3", whitelist[3]);
        getInput("infantry4", whitelist[4]);
        getInput("sentry", whitelist[7]);
        getInput("outpost", whitelist[10]);
        getInput("base", whitelist[11]);
    }

    BT::NodeStatus SetWhitelistAction::tick()
    {
        getInput("hero", whitelist[1]);
        getInput("engineer", whitelist[2]);
        getInput("infantry3", whitelist[3]);
        getInput("infantry4", whitelist[4]);
        getInput("sentry", whitelist[7]);
        getInput("outpost", whitelist[10]);
        getInput("base", whitelist[11]);

        config().blackboard->set<uint8_t>("whitelist_hero", whitelist[1]);
        config().blackboard->set<uint8_t>("whitelist_engineer", whitelist[2]);
        config().blackboard->set<uint8_t>("whitelist_infantry3", whitelist[3]);
        config().blackboard->set<uint8_t>("whitelist_infantry4", whitelist[4]);
        config().blackboard->set<uint8_t>("whitelist_sentry", whitelist[7]);
        config().blackboard->set<uint8_t>("whitelist_outpost", whitelist[10]);
        config().blackboard->set<uint8_t>("whitelist_base", whitelist[11]);

        RCLCPP_INFO(node_->get_logger(), "手动设置白名单: %d, %d, %d, %d, %d, %d, %d",
            whitelist[1], whitelist[2], whitelist[3], whitelist[4], whitelist[7], whitelist[10], whitelist[11]);

        return BT::NodeStatus::SUCCESS;
    }

} // namespace nav2_behavior_tree
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::SetWhitelistAction>("SetWhitelist");
}