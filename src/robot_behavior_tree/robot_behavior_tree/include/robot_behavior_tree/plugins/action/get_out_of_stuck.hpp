#ifndef GET_OUT_OF_STUCK_HPP_
#define GET_OUT_OF_STUCK_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/twist.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace nav2_behavior_tree
{
    class GetOutOfStuckAction : public BT::SyncActionNode
    {
    public:
        GetOutOfStuckAction(
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        GetOutOfStuckAction() = delete;

        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts()
        {
            return {};
        }

    private:
        void cancel_all_goals();
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        bool is_force_move_complete_;
        uint8_t count;
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
    };

} // namespace nav2_behavior_tree

#endif //
