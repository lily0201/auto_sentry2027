#include <string>

#include "robot_behavior_tree/plugins/action/buy_bullet_in_recovery.hpp"
namespace nav2_behavior_tree
{
    BuyBulletInRecoveryAction::BuyBulletInRecoveryAction(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf),
          number_of_bullet(0)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        sentryinfo_sub_ = node_->create_subscription<rm_interfaces::msg::Sentryinfo>(
            "/robot/sentryinfo",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&BuyBulletInRecoveryAction::sentryinfoCallback, this, std::placeholders::_1),
            sub_option);

        getInput("number_of_bullet", number_of_bullet);
        config().blackboard->get<uint32_t>("buy_bullet_at_recovery", buy_bullet_at_recovery);
        config().blackboard->get<bool>("if_waiting_buy_bullet_at_recovery", if_waiting_buy_bullet_at_recovery);
    }

    BT::NodeStatus BuyBulletInRecoveryAction::tick()
    {
        //更新裁判系统数据，获取黑板中的数据
        getInput("number_of_bullet", number_of_bullet); //从端口中获取需要购买的发弹量
        config().blackboard->get<uint32_t>("buy_bullet_at_recovery", buy_bullet_at_recovery);
        config().blackboard->get<bool>("if_waiting_buy_bullet_at_recovery", if_waiting_buy_bullet_at_recovery);
        callback_group_executor_.spin_some();

        config().blackboard->set<uint32_t>("number_of_bullet_to_buy", number_of_bullet); //记录本次购买的弹量
        RCLCPP_INFO(node_->get_logger(), "if_waiting_buy_bullet_at_recovery: %d", if_waiting_buy_bullet_at_recovery);

        //发送补给区买弹请求/等待补给区买弹判断是否成功
        if(received_sentryinfo_.amount_of_get_ammunition == buy_bullet_at_recovery &&
            !if_waiting_buy_bullet_at_recovery)
        {
            buy_bullet_at_recovery += number_of_bullet;
            config().blackboard->set<uint32_t>("buy_bullet_at_recovery", buy_bullet_at_recovery);
            RCLCPP_INFO(node_->get_logger(), "发送在补给点购买%u发弹的请求，等待中", number_of_bullet);
            if_waiting_buy_bullet_at_recovery = true; //当前在等待补给区买弹状态
        }
        else if (received_sentryinfo_.amount_of_get_ammunition < buy_bullet_at_recovery)
        {
            RCLCPP_INFO(node_->get_logger(), "等待补给区买弹请求处理中");
            if_waiting_buy_bullet_at_recovery = true;
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "补给点买弹请求错误，等待同步！");
            if_waiting_buy_bullet_at_recovery = false;
        }
        config().blackboard->set<bool>("if_waiting_buy_bullet_at_recovery", if_waiting_buy_bullet_at_recovery);

        RCLCPP_INFO(node_->get_logger(), "当前要在补给区购买的发弹量: %d, 在补给区购买的总发弹量: %d, 裁判系统数据: %d",
            number_of_bullet, buy_bullet_at_recovery, received_sentryinfo_.amount_of_get_ammunition);

        return BT::NodeStatus::SUCCESS;
    }

    void BuyBulletInRecoveryAction::sentryinfoCallback(rm_interfaces::msg::Sentryinfo::SharedPtr msg)
    {
        received_sentryinfo_.amount_of_get_ammunition = msg->amount_of_get_ammunition; //获取来自裁判系统的买血总次数，用于同步判断
    }

} // namespace nav2_behavior_tree
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::BuyBulletInRecoveryAction>("BuyBulletInRecovery");
}