//
// Created by elsa on 25-3-15.
//

#ifndef FOLLOW_AIM_HPP
#define FOLLOW_AIM_HPP

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rm_interfaces/msg/truncate_path_info.hpp"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "nav2_util/robot_utils.hpp"

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

namespace nav2_behavior_tree
{

    /**
     * @brief A BT::ConditionNode that listens to a battery topic and
     * returns SUCCESS when battery is low and FAILURE otherwise
     */
    class FollowAimAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param action_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        FollowAimAction(
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        FollowAimAction() = delete;

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
                BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to plan to"),
            };
        }

    private:
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<tf2_ros::Buffer> tf_;

        /// map系下的敌人目标点
        geometry_msgs::msg::PoseStamped aim_goal_;
        /// odom_yaw系下的敌人目标点
        geometry_msgs::msg::PoseStamped received_enemy_position_;
        geometry_msgs::msg::PoseStamped last_received_enemy_position_;
        std::string global_frame_;
        std::string robot_base_frame_;

        double transform_tolerance_;
        double distance;

        /// 机器人在map系中的当前位姿
        geometry_msgs::msg::PoseStamped current_pose;
        geometry_msgs::msg::TransformStamped transform;

        /// 裁剪路径信息，和update_goal统一
        rm_interfaces::msg::TruncatePathInfo truncate_path_info_msg_;
        rclcpp::Publisher<rm_interfaces::msg::TruncatePathInfo>::SharedPtr truncate_path_info_pub_;
    };

} // namespace nav2_behavior_tree

#endif //FOLLOW_AIM_HPP
