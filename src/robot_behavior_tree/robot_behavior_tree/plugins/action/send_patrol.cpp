#include <string>

#include "robot_behavior_tree/plugins/action/send_patrol.hpp"

namespace nav2_behavior_tree
{

    SendPatrolAction::SendPatrolAction(
         const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf),
          chasing(0)
    {
        config().blackboard->get<float>("max_pitch",max_pitch_);
        config().blackboard->get<float>("min_pitch",min_pitch_);
        config().blackboard->get<int>("mode",mode);
        config().blackboard->get<int>("spin",spin);
        config().blackboard->get<int>("patrol",patrol);
        config().blackboard->get<int>("robot_aim",robot_aim);
        config().blackboard->get<bool>("is_in_chase",is_in_chase);
        config().blackboard->get<bool>("if_in_tunnel", if_in_tunnel_);
        config().blackboard->get<bool>("is_in_weak", is_in_weak);

        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        patrol_pub_ = node_->create_publisher<rm_interfaces::msg::Action>("/robot/action", 10);
    }

    BT::NodeStatus SendPatrolAction::tick()
    {
        config().blackboard->get<float>("max_pitch",max_pitch_);
        config().blackboard->get<float>("min_pitch",min_pitch_);
        config().blackboard->get<int>("mode",mode);
        config().blackboard->get<int>("spin",spin);
        config().blackboard->get<int>("patrol",patrol);
        config().blackboard->get<int>("robot_aim",robot_aim);
        config().blackboard->get<bool>("is_in_chase",is_in_chase);
        config().blackboard->get<bool>("if_in_tunnel", if_in_tunnel_);
        config().blackboard->get<bool>("is_in_weak", is_in_weak);

        if (if_in_tunnel_) {
            spin = 0;
            patrol = 0;
            robot_aim = 0;
        }
        if (is_in_weak) {
            // mode = 0;
            spin = 0;
            RCLCPP_INFO(node_->get_logger(), "Sending patrol: is_in_weak = true");
        }
        if(is_in_chase)
            chasing = 1;
        else
            chasing = 0;

        rm_interfaces::msg::Action action;
        action.max_pitch_w = max_pitch_;
        action.min_pitch_w = min_pitch_;
        action.mode = mode;
        action.spin = spin;
        action.patrol = patrol;
        action.robot_aim = robot_aim;
        action.chasing = chasing;
        patrol_pub_->publish(action);

        RCLCPP_INFO(node_->get_logger(), "sending mode: %d, spin: %d, patrol: %d, robot_aim: %d, is_in_chase: %d",
            mode, spin, patrol, robot_aim, chasing);

        return BT::NodeStatus::SUCCESS;
        
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::SendPatrolAction>("SendPatrol");
}
