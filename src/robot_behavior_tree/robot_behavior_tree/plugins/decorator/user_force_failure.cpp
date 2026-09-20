#include <string>

#include "robot_behavior_tree/plugins/decorator/user_force_failure.hpp"

namespace nav2_behavior_tree
{
    UserForceFailureDecorator::UserForceFailureDecorator(
        const std::string& name)
        : BT::DecoratorNode(name,{})
    {
    }

    BT::NodeStatus UserForceFailureDecorator::tick()
    {
        setStatus(BT::NodeStatus::RUNNING);

        const BT::NodeStatus child_status = child_node_->executeTick();

        if(StatusCompleted(child_status))
        {
            resetChild();
            return BT::NodeStatus::FAILURE;
        }

        // RUNNING
        return BT::NodeStatus::FAILURE;
    }

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::UserForceFailureDecorator>("UserForceFailure");
}