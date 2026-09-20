#ifndef IF_ROBOT_STUCK_HPP_
#define IF_ROBOT_STUCK_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"



namespace nav2_behavior_tree
{
    class IfRobotStuckCondition : public BT::ConditionNode
    {
    public:
        IfRobotStuckCondition(
            const std::string &condition_name,
            const BT::NodeConfiguration &conf);

        IfRobotStuckCondition() = delete;

        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<int>("stuck_threshold_distance", "stuck_threshold_distance x + y"),
                BT::InputPort<int>("stuck_threshold_cnt", "stuck_threshold_cnt=time(s)*50Hz"),
            };
        }

    private:
        rclcpp::Node::SharedPtr node_;

        double average_robot_pos_x_;
        double average_robot_pos_y_;
        double robot_pos_x_;
        double robot_pos_y_;
        double stuck_threshold_distance_;

        int stuck_cnt_;
        int stuck_threshold_cnt_;
    };

} // namespace nav2_behavior_tree

#endif //
