#ifndef IF_APPOINT_ROBOT_IN_TARGET_AREA_HPP_
#define IF_APPOINT_ROBOT_IN_TARGET_AREA_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/robotposition.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief A BT::ConditionNode that listens to a battery topic and
     * returns SUCCESS when battery is low and FAILURE otherwise
     */
    class IfAppointRobotInTargetAreaCondition : public BT::ConditionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param condition_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        IfAppointRobotInTargetAreaCondition(
            const std::string &condition_name,
            const BT::NodeConfiguration &conf);

        IfAppointRobotInTargetAreaCondition() = delete;

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
                BT::InputPort<int>("is_our","1 for our, 0 for their"),
                BT::InputPort<int>("RobotName", "RobotName to appoint"),
                BT::InputPort<double>("position_x", "position_x to plan to"),
                BT::InputPort<double>("position_y", "position_y to plan to"),
                BT::InputPort<double>("range_x", "range_x"),
                BT::InputPort<double>("range_y", "range_y"),
            };
        }

    private:
        /**
         * @brief Callback function for battery topic
         * @param msg Shared pointer to sensor_msgs::msg::BatteryState message
         */
        void robotpositionCallback(rm_interfaces::msg::Robotposition::SharedPtr msg);

        rclcpp::Node::SharedPtr node_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        rclcpp::Subscription<rm_interfaces::msg::Robotposition>::SharedPtr robotposition_sub_;

        bool is_we_are_blue_;
        bool is_in_target_area_;

        int is_our;
        int robotname;

        double position_x;
        double position_y;
        double range_x;
        double range_y;
    };

} // namespace nav2_behavior_tree

#endif //
