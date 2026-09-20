//
// Created by elsa on 25-3-27.
//

#ifndef SEND_WHITELIST_HPP
#define SEND_WHITELIST_HPP

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/whitelist.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief A BT::ConditionNode that listens to a battery topic and
     * returns SUCCESS when battery is low and FAILURE otherwise
     */
    class SendWhitelistAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param action_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        SendWhitelistAction(
           const std::string &action_name,
            const BT::NodeConfiguration &conf);

        SendWhitelistAction() = delete;

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
        rclcpp::Publisher<rm_interfaces::msg::Whitelist>::SharedPtr whitelist_pub_;
        rclcpp::Node::SharedPtr node_;

        bool red_hero_invincible;
        bool red_engineer_invincible;
        bool red_infantry3_invincible;
        bool red_infantry4_invincible;
        bool red_sentry_invincible;
        bool blue_hero_invincible;
        bool blue_engineer_invincible;
        bool blue_infantry3_invincible;
        bool blue_infantry4_invincible;
        bool blue_sentry_invincible;
        bool is_we_are_blue;

        uint8_t whitelist_[12];
    };

} // namespace nav2_behavior_tree

#endif //SEND_WHITELIST_HPP
