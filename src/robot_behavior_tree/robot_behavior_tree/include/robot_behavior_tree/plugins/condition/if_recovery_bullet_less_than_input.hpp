//
// Created by elsa on 25-4-19.
//

#ifndef IF_RECOVERY_BULLET_LESS_THAN_INPUT_HPP
#define IF_RECOVERY_BULLET_LESS_THAN_INPUT_HPP

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief A BT::ConditionNode that listens to a battery topic and
     * returns SUCCESS when battery is low and FAILURE otherwise
     */
    class IfRecoveryBulletLessThanInputCondition : public BT::ConditionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param condition_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        IfRecoveryBulletLessThanInputCondition(
            const std::string &condition_name,
            const BT::NodeConfiguration &conf);

        IfRecoveryBulletLessThanInputCondition() = delete;

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
                BT::InputPort<uint32_t>("recovery_bullet_threshold", "recovery_bullet threshold for low recovery_bullet condition"),
            };
        }

    private:
        /**
         * @brief Callback function for battery topic
         * @param msg Shared pointer to sensor_msgs::msg::BatteryState message
         */
        rclcpp::Node::SharedPtr node_;
        uint32_t recovery_bullet_threshold_;
        uint16_t recovery_bullet_to_acquire; //补给区当前可领的免费发弹量
    };

} // namespace nav2_behavior_tree

#endif //IF_RECOVERY_BULLET_LESS_THAN_INPUT_HPP
