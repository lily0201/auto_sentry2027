#ifndef UPDATE_GOAL_HPP_
#define UPDATE_GOAL_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rm_interfaces/msg/truncate_path_info.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include <tf2/LinearMath/Quaternion.h>

namespace nav2_behavior_tree
{

    /**
     * @brief A BT::ConditionNode that listens to a battery topic and
     * returns SUCCESS when battery is low and FAILURE otherwise
     */
    class UpdateGoalAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param action_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        UpdateGoalAction(
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        UpdateGoalAction() = delete;

        /**
         * @brief The main override required by a BT action
         * @return BT::NodeStatus Status of tick execution
         */
        BT::NodeStatus tick() override;

        /**
         * @brief Creates list of BT ports
         * @return BT::PortsList Containing node-specific ports
         */
        static BT::PortsList providedPorts()
        {
            return {
                    BT::InputPort<double>("position_x", "position_x to plan to"),
                    BT::InputPort<double>("position_y", "position_y to plan to"),
                    BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to plan to"),
                };
        }

    private:
        rclcpp::Node::SharedPtr node_;

        /// 目标点x
        double position_x;
        /// 目标点y
        double position_y;
        /// 裁剪路径长度
        double distance;
        /// 打符朝向yaw角度
        double dafu_yaw_angle;

        /// 是否在打符模式
        int if_dafu;

        /// 目标点位姿
        geometry_msgs::msg::PoseStamped pose;

        /// 裁剪路径信息
        rm_interfaces::msg::TruncatePathInfo truncate_path_info_msg_;
        rclcpp::Publisher<rm_interfaces::msg::TruncatePathInfo>::SharedPtr truncate_path_info_pub_;
    };

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
