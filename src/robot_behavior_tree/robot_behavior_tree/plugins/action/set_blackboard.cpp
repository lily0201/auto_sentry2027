#include <string>

#include "robot_behavior_tree/plugins/action/set_blackboard.hpp"
namespace nav2_behavior_tree
{
    SetBlackboardAction::SetBlackboardAction(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf),
        key(""),
        value(0)
    {
        getInput("key", key);
        getInput("value", value);
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    }

    BT::NodeStatus SetBlackboardAction::tick()
    {
        getInput("key", key);
        getInput("value", value);
        config().blackboard->set<int>(key, value);
        RCLCPP_INFO(node_->get_logger(), "Setting value %d to key %s", value, key.c_str());
        return BT::NodeStatus::SUCCESS;
    }

} // namespace nav2_behavior_tree
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::SetBlackboardAction>("SetBlackboarduser");
}