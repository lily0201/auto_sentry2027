#ifndef RECORD_ROBOT_BLOOD_HPP_
#define RECORD_ROBOT_BLOOD_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/hp.hpp"
#include "rm_interfaces/msg/sentryinfo.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief 记录机器人血量节点，记录从裁判系统读到的己方和敌方所有机器人、前哨站、基地的血量，并进行以下判断：
     * @brief 1.机器人无敌判断：机器人血量为0认为进入无敌状态，当机器人血量从0增加时开始计时，10s后认为脱离无敌状态
     * @brief 2.判断两方飞镖命中基地的情况：根据基地前后血量差判断是否被飞镖命中，若命中是命中了什么类型的靶子；并计算致盲时间，致盲时间内都认为处于飞镖命中状态
     * @brief 3.判断哨兵是否处于受伤状态：前一次存下的血量大于当前从裁判系统读到的血量则认为处于受伤状态
     * @brief 4.判断哨兵是否远程买血成功：如果哨兵当前从裁判系统读到的血量 - 前一次存下的血量 > 100，且远程买血次数与裁判系统同步，则认为买血成功
     * @brief 4.若处于等待远程买血判断状态则开始计时，10s未买血成功则认为该次购买失败，将数据和裁判系统同步；其余错误情况也直接将数据和裁判系统同步
     */
    class RecordRobotBloodAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param action_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        RecordRobotBloodAction(
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        RecordRobotBloodAction() = delete;

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
         * @brief Callback function 裁判系统机器人血量
         * @param msg Shared pointer to rm_interfaces::msg::Hp message
         */
        void hpCallback(rm_interfaces::msg::Hp::SharedPtr msg);
        /**
         * @brief Callback function 裁判系统sentry_info
         * @param msg Shared pointer to rm_interfaces::msg::Sentryinfo message
         */
        void sentryinfoCallback(rm_interfaces::msg::Sentryinfo::SharedPtr msg);
        /**
         * @brief 机器人无敌判断函数，机器人血量为0认为进入无敌状态，当机器人血量从0增加时开始计时，10s后认为脱离无敌状态
         */
        void invincible_judge();
        /**
         * @brief 根据基地前后血量差判断是否被飞镖命中，若命中是命中了什么类型的靶子；并计算致盲时间，致盲时间内都认为处于飞镖命中状态
         */
        void dart_hit_judge();

        rclcpp::Node::SharedPtr node_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        rclcpp::Subscription<rm_interfaces::msg::Hp>::SharedPtr hp_sub_;
        rclcpp::Subscription<rm_interfaces::msg::Sentryinfo>::SharedPtr sentryinfo_sub_;

        ///机器人无敌判断 & 买血是否成功计数时间阈值
        int count_size;
        ///我方飞镖击中后致盲计数时间阈值
        int count_size_our_dart;
        ///敌方飞镖击中后致盲计数时间阈值
        int count_size_his_dart;

        /* 机器人相关血量 */
        int red_hero;
        int red_engineer;
        int red_infantry3;
        int red_infantry4;
        int red_sentry;
        int red_outpost;
        int red_base;
        int blue_hero;
        int blue_engineer;
        int blue_infantry3;
        int blue_infantry4;
        int blue_sentry;
        int blue_outpost;
        int blue_base;

        /* 在callback前获取的血量，用于前后对比判断 */
        ///用于判断是否受伤以及买血是否成功
        int last_sentry;
        ///用于判断飞镖是否命中
        int last_our_base;
        int last_his_base;

        ///己方哨兵的血量
        int our_sentry_blood;
        ///己方基地的血量
        int our_base_blood;
        ///敌方基地的血量
        int his_base_blood;

        /* 每个机器人的时间计数，用于判断当前是否在无敌状态 */
        int count_red_hero;
        int count_red_engineer;
        int count_red_infantry3;
        int count_red_infantry4;
        int count_red_sentry;
        int count_blue_hero;
        int count_blue_engineer;
        int count_blue_infantry3;
        int count_blue_infantry4;
        int count_blue_sentry;
        int count_buy_blood;
        ///我方飞镖击中后致盲剩余时间计数
        int count_our_dart;
        ///敌方飞镖击中后致盲剩余时间计数
        int count_his_dart;

        ///飞镖命中次数
        int our_dart_hit_count;
        int his_dart_hit_count;

        /* 机器人是否处于无敌状态 */
        bool red_hero_invincible;
        bool red_engineer_invincible;
        bool red_infantry3_invincible;
        bool red_infantry4_invincible;
        bool red_sentry_invincible;
        bool red_outpost_invincible;
        bool red_base_invincible;
        bool blue_hero_invincible;
        bool blue_engineer_invincible;
        bool blue_infantry3_invincible;
        bool blue_infantry4_invincible;
        bool blue_sentry_invincible;
        bool blue_outpost_invincible;
        bool blue_base_invincible;

        bool is_we_are_blue;
        ///我方飞镖是否命中敌方基地
        bool is_dart_hit_our;
        ///敌方飞镖是否命中我方基地
        bool is_dart_hit_his;
        ///哨兵是否处于受伤状态
        bool is_hurt;
        ///当前是否在等待远程买血判断状态
        bool if_waiting_buy_blood;
        ///当前记录的远程买血的总次数
        uint32_t buy_blood_number;
        ///行为树每次循环需要的时间
        std::chrono::milliseconds bt_loop_duration_;

        rm_interfaces::msg::Sentryinfo received_sentryinfo_;
    };

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
