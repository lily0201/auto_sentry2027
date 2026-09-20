#include <string>

#include "robot_behavior_tree/plugins/action/printf.hpp"
namespace nav2_behavior_tree
{
    PrintfAction::PrintfAction(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf),
          txt("")
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        getInput("txt",txt);
    }

    BT::NodeStatus PrintfAction::tick()
    {   
        getInput("txt",txt);
        RCLCPP_INFO(node_->get_logger(), "%s", txt.c_str());
        return BT::NodeStatus::SUCCESS;
    }

} // namespace nav2_behavior_tree
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::PrintfAction>("Printf");
}