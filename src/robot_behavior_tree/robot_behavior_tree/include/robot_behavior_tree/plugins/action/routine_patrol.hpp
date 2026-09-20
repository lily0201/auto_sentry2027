#ifndef ROUTINE_PATROL_HPP_
#define ROUTINE_PATROL_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rm_interfaces/msg/truncate_path_info.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{
    class RoutinePatrolAction : public BT::SyncActionNode
    {
    public:
        RoutinePatrolAction(
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        RoutinePatrolAction() = delete;

        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<float>("select_time", "select_time"),
                BT::InputPort<double>("routine_position_x_1", "Destination to plan to"),
                BT::InputPort<double>("routine_position_y_1", "Destination to plan to"),
                BT::InputPort<double>("routine_position_x_2", "Destination to plan to"),
                BT::InputPort<double>("routine_position_y_2", "Destination to plan to"),
                BT::OutputPort<geometry_msgs::msg::PoseStamped>("routine_goal", "Destination to plan to"),
            };
        }

    private:
        rclcpp::Node::SharedPtr node_;

        int count_size_;
        int target_position_index;
        float select_time;

        double routine_position_x[2];
        double routine_position_y[2];

        std::chrono::milliseconds bt_loop_duration_;

        geometry_msgs::msg::PoseStamped pose;
        /// 裁剪路径信息
        rm_interfaces::msg::TruncatePathInfo truncate_path_info_msg_;
        rclcpp::Publisher<rm_interfaces::msg::TruncatePathInfo>::SharedPtr truncate_path_info_pub_;
    };

} // namespace nav2_behavior_tree

#endif //
