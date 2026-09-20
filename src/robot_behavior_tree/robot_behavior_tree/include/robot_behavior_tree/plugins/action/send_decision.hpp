#ifndef SEND_DECISION_HPP_
#define SEND_DECISION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/decision.hpp"
#include "rm_interfaces/msg/hp.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief A BT::ConditionNode that listens to a battery topic and
     * returns SUCCESS when battery is low and FAILURE otherwise
     */
    class SendDecisionAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param action_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        SendDecisionAction(
           const std::string &action_name,
            const BT::NodeConfiguration &conf);

        SendDecisionAction() = delete;

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
        void hpCallback(rm_interfaces::msg::Hp::SharedPtr msg);

        uint32_t  resurrection; //当前选择的复活方式
        uint32_t buy_bullet_at_recovery; //在补给区购买的弹量（递增式）
        uint32_t buy_bullet_remote_number; //远程买弹的次数（递增式）
        uint32_t buy_blood_number; //远程买血的次数（递增式）

        uint16_t sentry_blood;

        bool is_we_are_blue_;
        bool select_force_no_resurrection;

        rclcpp::Publisher<rm_interfaces::msg::Decision>::SharedPtr decision_pub_;
        rclcpp::Node::SharedPtr node_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        rclcpp::Subscription<rm_interfaces::msg::Hp>::SharedPtr hp_sub_;
    };

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
