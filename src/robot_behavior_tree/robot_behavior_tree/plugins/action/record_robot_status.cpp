//
// Created by elsa on 25-4-12.
//

#include <string>

#include "robot_behavior_tree/plugins/action/record_robot_status.hpp"

namespace nav2_behavior_tree
{
    RecordRobotStatusAction::RecordRobotStatusAction(
        const std::string& action_name,
        const BT::NodeConfiguration& conf)
        : BT::SyncActionNode(action_name, conf),
          if_rfid_in_recovery(false), if_waiting_buy_bullet_remote(false), if_waiting_buy_bullet_at_recovery(false),
          current_record_time(0), count_buy_bullet_remote(0), count_buy_bullet_at_recovery(0), is_in_weak(false)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        gamestatus_sub_ = node_->create_subscription<rm_interfaces::msg::Gamestatus>(
            "/robot/gamestatus",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&RecordRobotStatusAction::gamestatusCallback, this, std::placeholders::_1),
            sub_option);
        sentryinfo_sub_ = node_->create_subscription<rm_interfaces::msg::Sentryinfo>(
            "/robot/sentryinfo",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&RecordRobotStatusAction::sentryinfoCallback, this, std::placeholders::_1),
            sub_option);
        rfidstatus_sub_ = node_->create_subscription<rm_interfaces::msg::Rfidstatus>(
            "/robot/rfidstatus",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&RecordRobotStatusAction::rfidstatusCallback, this, std::placeholders::_1),
            sub_option);
        buff_sub_ = node_->create_subscription<rm_interfaces::msg::Buff>(
            "/robot/buff",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&RecordRobotStatusAction::buffCallback, this, std::placeholders::_1),
            sub_option);

        bt_loop_duration_ = config().blackboard->get<std::chrono::milliseconds>("bt_loop_duration");
        count_size = 10 * 1000 / bt_loop_duration_.count();
        buy_bullet_at_recovery_count_size = 2 * 1000 / bt_loop_duration_.count();

        config().blackboard->get<bool>("is_we_are_blue", is_we_are_blue);

        config().blackboard->get<uint32_t>("buy_bullet_remote_number", buy_bullet_remote_number);
        config().blackboard->get<uint32_t>("buy_bullet_at_recovery", buy_bullet_at_recovery);

        config().blackboard->get<uint32_t>("number_of_bullet_to_buy", number_of_bullet);
        config().blackboard->get<uint16_t>("recovery_bullet_to_acquire", recovery_bullet_to_acquire);
        config().blackboard->get<uint16_t>("recovery_bullet_acquired", recovery_bullet_acquired);
    }

    BT::NodeStatus RecordRobotStatusAction::tick()
    {
        config().blackboard->get<bool>("is_we_are_blue", is_we_are_blue);
        /* 在callback之前获取用于前后对比的变量 */
        config().blackboard->get<uint16_t>("recovery_bullet_to_acquire", recovery_bullet_to_acquire);
        config().blackboard->get<uint16_t>("recovery_bullet_acquired", recovery_bullet_acquired);
        /* 获取关于买弹和远程买弹的变量 */
        config().blackboard->get<uint32_t>("buy_bullet_remote_number", buy_bullet_remote_number);
        config().blackboard->get<uint32_t>("buy_bullet_at_recovery", buy_bullet_at_recovery);
        config().blackboard->get<uint32_t>("number_of_bullet_to_buy", number_of_bullet);

        callback_group_executor_.spin_some();

        /* 再根据rfid是否在补给区内更新当前可领的免费发弹量和已经领取的免费发弹量 */
        if(if_rfid_in_recovery)
        {
            recovery_bullet_acquired += recovery_bullet_to_acquire;
            recovery_bullet_to_acquire = 0;
            config().blackboard->set<uint16_t>("recovery_bullet_to_acquire", recovery_bullet_to_acquire);
            config().blackboard->set<uint16_t>("recovery_bullet_acquired", recovery_bullet_acquired);
        }
        RCLCPP_INFO(node_->get_logger(), "当前补给区可领免费发弹量：%d，已经领取的免费发弹量: %d",
            recovery_bullet_to_acquire, recovery_bullet_acquired);

        /* 判断远程买弹请求是否成功 */
        if(buy_bullet_remote_number == received_sentryinfo_.number_of_get_ammunition && 
        received_sentryinfo_.number_of_get_ammunition != 0)
        {
            RCLCPP_INFO(node_->get_logger(), "远程买弹成功");
            if_waiting_buy_bullet_remote = false;
        }
        else if(buy_bullet_remote_number - received_sentryinfo_.number_of_get_ammunition == 1)
        {
            count_buy_bullet_remote++;
            if_waiting_buy_bullet_remote = true;
            if(count_buy_bullet_remote >= count_size)
            {
                count_buy_bullet_remote = 0;
                buy_bullet_remote_number = received_sentryinfo_.number_of_get_ammunition;
                RCLCPP_INFO(node_->get_logger(), "远程买弹超时，重置次数与裁判系统相同");
                if_waiting_buy_bullet_remote = false;
            }
        }
        else
        {
            buy_bullet_remote_number = received_sentryinfo_.number_of_get_ammunition;
            RCLCPP_INFO(node_->get_logger(), "远程买弹错误，重置次数与裁判系统相同");
            if_waiting_buy_bullet_remote = false;
        }

        RCLCPP_INFO(node_->get_logger(), "--------if_rfid_in_recovery: %d---------", if_rfid_in_recovery);
        /* 判断补给区买弹请求是否成功 */
        if(if_rfid_in_recovery && number_of_bullet != 0 &&
            received_sentryinfo_.amount_of_get_ammunition == buy_bullet_at_recovery)
        {
            RCLCPP_INFO(node_->get_logger(), "在补给点买弹成功，买了%u发弹", number_of_bullet);
            if_waiting_buy_bullet_at_recovery = false;
        }
        else if (if_rfid_in_recovery && number_of_bullet != 0 &&
            received_sentryinfo_.amount_of_get_ammunition < buy_bullet_at_recovery)
        {
            count_buy_bullet_at_recovery++;
            if_waiting_buy_bullet_at_recovery = true;
            if (count_buy_bullet_at_recovery >= buy_bullet_at_recovery_count_size)
            {
                count_buy_bullet_at_recovery = 0;
                buy_bullet_at_recovery = received_sentryinfo_.amount_of_get_ammunition;
                RCLCPP_INFO(node_->get_logger(), "补给点买弹错误，同步可买发弹量为 %u", buy_bullet_at_recovery);
                if_waiting_buy_bullet_at_recovery = false;
            }
        }
        else if (if_rfid_in_recovery && number_of_bullet != 0 &&
            received_sentryinfo_.amount_of_get_ammunition > buy_bullet_at_recovery)
        {
            buy_bullet_at_recovery = received_sentryinfo_.amount_of_get_ammunition;
            RCLCPP_INFO(node_->get_logger(), "补给点买弹错误，重置次数与裁判系统相同 %u", buy_bullet_at_recovery);
            if_waiting_buy_bullet_at_recovery = false;
        }

        config().blackboard->set<uint32_t>("buy_bullet_at_recovery", buy_bullet_at_recovery);
        config().blackboard->set<uint32_t>("buy_bullet_remote_number", buy_bullet_remote_number);
        config().blackboard->set<bool>("if_waiting_buy_bullet_remote", if_waiting_buy_bullet_remote);
        config().blackboard->set<bool>("if_waiting_buy_bullet_at_recovery", if_waiting_buy_bullet_at_recovery);

        RCLCPP_INFO(node_->get_logger(), "record robot status successfully");

        return BT::NodeStatus::SUCCESS;
    }

    void RecordRobotStatusAction::gamestatusCallback(rm_interfaces::msg::Gamestatus::SharedPtr msg)
    {
        current_record_time = msg->stage_remain_time;
        if(current_record_time % 60 == 0)
        {
            recovery_bullet_to_acquire += 100; //更新可领的免费发弹量
            config().blackboard->set<uint16_t>("recovery_bullet_to_acquire", recovery_bullet_to_acquire);
        }
    }

    void RecordRobotStatusAction::sentryinfoCallback(rm_interfaces::msg::Sentryinfo::SharedPtr msg)
    {
        received_sentryinfo_.amount_of_get_ammunition = msg->amount_of_get_ammunition;
        received_sentryinfo_.number_of_get_ammunition = msg->number_of_get_ammunition;
    }

    void RecordRobotStatusAction::rfidstatusCallback(rm_interfaces::msg::Rfidstatus::SharedPtr msg)
    {
        if_rfid_in_recovery = msg->inside_recovery || msg->outside_recovery;
    }

    void RecordRobotStatusAction::buffCallback(rm_interfaces::msg::Buff::SharedPtr msg)
    {
        RCLCPP_INFO(node_->get_logger(), "record msg->remaining_energy: %hhu", msg->remaining_energy);
        if (!(msg->remaining_energy & 0b00001000) && msg->remaining_energy!=0x32) {
            is_in_weak = true;
            RCLCPP_INFO(node_->get_logger(), "底盘剩余能量<5%%，即将进入虚弱状态");
        }
        else if (msg->remaining_energy & 0b00000001 || msg->remaining_energy!=0x32) {
            is_in_weak = false;
            RCLCPP_INFO(node_->get_logger(), "底盘剩余能量>50%%");
        }
        config().blackboard->set<bool>("is_in_weak", is_in_weak);
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::RecordRobotStatusAction>("RecordRobotStatus");
}
