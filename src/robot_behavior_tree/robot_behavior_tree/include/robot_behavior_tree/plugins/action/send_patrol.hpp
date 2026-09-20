#ifndef SEND_PATROL_HPP_
#define SEND_PATROL_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/action.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief A BT::ConditionNode that listens to a battery topic and
     * returns SUCCESS when battery is low and FAILURE otherwise
     */
    class SendPatrolAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param action_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        SendPatrolAction(
           const std::string &action_name,
            const BT::NodeConfiguration &conf);

        SendPatrolAction() = delete;

        /**
         * @brief The main override required by a BT action
         * @return BT::NodeStatus Status of tick execution
         */
        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts()
        {
            return {};
        }

    private:
        float max_pitch_;
        float min_pitch_;
        int mode;
        int spin;
        int patrol;
        int robot_aim;
        int chasing;

        bool if_in_tunnel_;
        bool is_in_weak;
        bool is_in_chase;

        rclcpp::Publisher<rm_interfaces::msg::Action>::SharedPtr patrol_pub_;
        rclcpp::Node::SharedPtr node_;
    };

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
