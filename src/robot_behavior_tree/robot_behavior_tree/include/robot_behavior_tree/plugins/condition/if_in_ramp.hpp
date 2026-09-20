#ifndef IF_IN_RAMP_HPP_
#define IF_IN_RAMP_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "tf2_ros/buffer.h"

namespace nav2_behavior_tree
{

    /**
     * @brief A BT::ConditionNode that listens to a battery topic and
     * returns SUCCESS when battery is low and FAILURE otherwise
     */
    class IfInRampCondition : public BT::ConditionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param condition_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        IfInRampCondition(
            const std::string &condition_name,
            const BT::NodeConfiguration &conf);

        IfInRampCondition() = delete;

        /**
         * @brief The main override required by a BT action
         * @return BT::NodeStatus Status of tick execution
         */
        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
                BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame"),
            };
        }

    private:
        std::string global_frame_;
        std::string robot_base_frame_;
        rclcpp::Node::SharedPtr node_;
        double transform_tolerance_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        bool is_in_ramp;
        double formatAngle(double angle);
    };

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
