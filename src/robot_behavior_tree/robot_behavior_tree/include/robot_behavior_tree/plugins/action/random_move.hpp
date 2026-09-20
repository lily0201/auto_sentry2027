#ifndef RANDOM_MOVE_HPP_
#define RANDOM_MOVE_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "behaviortree_cpp_v3/action_node.h"

#include <random> // 包含随机数库

namespace nav2_behavior_tree
{

    /**
     * @brief 随机移动节点，每次tick会在指定的四个点中随机选取一个目标点进行导航
     */
    class RandomMoveAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param action_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        RandomMoveAction(
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        RandomMoveAction() = delete;

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
            return
            {
                BT::InputPort<float>("select_time", "select_time"),
                BT::InputPort<double>("random_position_x_1", "Destination to plan to"),
                BT::InputPort<double>("random_position_y_1", "Destination to plan to"),
                BT::InputPort<double>("random_position_x_2", "Destination to plan to"),
                BT::InputPort<double>("random_position_y_2", "Destination to plan to"),
                BT::InputPort<double>("random_position_x_3", "Destination to plan to"),
                BT::InputPort<double>("random_position_y_3", "Destination to plan to"),
                BT::InputPort<double>("random_position_x_4", "Destination to plan to"),
                BT::InputPort<double>("random_position_y_4", "Destination to plan to"),
                BT::OutputPort<geometry_msgs::msg::PoseStamped>("randomgoal", "Destination to plan to"),
            };
        }

    private:
        rclcpp::Node::SharedPtr node_;
        ///两次选取目标点之间的时间间隔
        float select_time;
        ///计数阈值，达到阈值表示已经经过相应的时间
        int count_size_ ;
        std::chrono::milliseconds bt_loop_duration_;
        //指定的4个点
        double random_position_x[4];
        double random_position_y[4];
        geometry_msgs::msg::PoseStamped pose;
    };  

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
