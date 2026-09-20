#ifndef IF_GOAL_REACHED_HPP_
#define IF_GOAL_REACHED_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_behavior_tree
{

    /**
     * @brief A BT::ConditionNode that listens to a battery topic and
     * returns SUCCESS when battery is low and FAILURE otherwise
     */
    class IsGoalReachedCondition : public BT::ConditionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param condition_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        IsGoalReachedCondition(
            const std::string &condition_name,
            const BT::NodeConfiguration &conf);

        IsGoalReachedCondition() = delete;

        /**
         * @brief The main override required by a BT action
         * @return BT::NodeStatus Status of tick execution
         */
        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "goal"),
            };
        }

    private:
        void AimPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
        rclcpp::Node::SharedPtr node_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aim_pose_sub_;

        std::shared_ptr<tf2_ros::Buffer> tf_;

        bool is_goal_reached;
        bool is_in_chase;

        /// 普通目标点
        geometry_msgs::msg::PoseStamped current_goal;
        /// 自瞄目标点
        geometry_msgs::msg::PoseStamped current_aim_pose_;

        double transform_tolerance_;

        std::string global_frame_;
        std::string robot_base_frame_;
    };

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
