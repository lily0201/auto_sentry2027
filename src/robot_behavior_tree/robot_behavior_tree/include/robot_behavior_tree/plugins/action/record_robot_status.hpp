//
// Created by elsa on 25-4-12.
//

#ifndef RECORD_ROBOT_STATUS_HPP
#define RECORD_ROBOT_STATUS_HPP

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/gamestatus.hpp"
#include "rm_interfaces/msg/buff.hpp"
#include "rm_interfaces/msg/sentryinfo.hpp"
#include "rm_interfaces/msg/rfidstatus.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief 记录机器人状态节点，主要判断哨兵补给区买弹/远程买弹是否成功
     * @brief 补给区买弹判断：如果rfid在补给区 & 当前要买的发弹量不为0 & 在补给区买弹总量和裁判系统同步，则认为补给区买弹成功
     * @brief 补给区免费发弹量判断：读比赛剩余时间，每过60秒增加100发当前可领的免费发弹量；若哨兵回到补给区则将当前可领的免费发弹量清零，并记录已经领取的免费发弹量
     * @brief 远程买弹判断：如果哨兵远程买弹次数与裁判系统同步（不为0），则认为远程买弹成功；超时判断和错误判断和买血相同
     */
    class RecordRobotStatusAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param action_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        RecordRobotStatusAction(
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        RecordRobotStatusAction() = delete;

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
            return {};
        }

    private:
        /**
         * @brief Callback function 裁判系统比赛剩余时间
         * @param msg Shared pointer to rm_interfaces::msg::Gamestatus message
         */
        void gamestatusCallback(rm_interfaces::msg::Gamestatus::SharedPtr msg);
        /**
         * @brief Callback function 裁判系统sentry_info
         * @param msg Shared pointer to rm_interfaces::msg::Sentryinfo message
         */
        void sentryinfoCallback(rm_interfaces::msg::Sentryinfo::SharedPtr msg);
        /**
         * @brief Callback function 裁判系统rfid
         * @param msg Shared pointer to rm_interfaces::msg::Rfidstatus message
         */
        void rfidstatusCallback(rm_interfaces::msg::Rfidstatus::SharedPtr msg);
        /**
         * @brief Callback function 裁判系统buff
         * @param msg Shared pointer to rm_interfaces::msg::Buff message
         */
        void buffCallback(rm_interfaces::msg::Buff::SharedPtr msg);

        rclcpp::Node::SharedPtr node_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        rclcpp::Subscription<rm_interfaces::msg::Gamestatus>::SharedPtr gamestatus_sub_;
        rclcpp::Subscription<rm_interfaces::msg::Sentryinfo>::SharedPtr sentryinfo_sub_;
        rclcpp::Subscription<rm_interfaces::msg::Rfidstatus>::SharedPtr rfidstatus_sub_;
        rclcpp::Subscription<rm_interfaces::msg::Buff>::SharedPtr buff_sub_;

        ///当前记录的远程买弹的总次数
        uint32_t buy_bullet_remote_number;
        ///当前记录的在补给区买弹的总弹量
        uint32_t buy_bullet_at_recovery;
        ///需要在补给区购买的发弹量
        uint32_t number_of_bullet;
        ///补给区当前可领的免费发弹量
        uint16_t recovery_bullet_to_acquire;
        ///补给区已经领取的免费发弹量
        uint16_t recovery_bullet_acquired;

        bool is_we_are_blue;
        bool if_rfid_in_recovery;
        bool if_waiting_buy_bullet_remote;
        bool if_waiting_buy_bullet_at_recovery;
        bool is_in_weak;
        ///当前比赛剩余时间
        int current_record_time;
        int count_buy_bullet_remote;
        int count_buy_bullet_at_recovery;
        ///计数时间阈值
        int count_size;
        int buy_bullet_at_recovery_count_size;
        ///行为树每次循环需要的时间
        std::chrono::milliseconds bt_loop_duration_;

        rm_interfaces::msg::Sentryinfo received_sentryinfo_;
    };

} // namespace nav2_behavior_tree

#endif //RECORD_ROBOT_STATUS_HPP
