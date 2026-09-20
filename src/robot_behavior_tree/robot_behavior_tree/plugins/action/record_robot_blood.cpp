#include <string>

#include "robot_behavior_tree/plugins/action/record_robot_blood.hpp"

namespace nav2_behavior_tree
{
    RecordRobotBloodAction::RecordRobotBloodAction(
        const std::string& action_name,
        const BT::NodeConfiguration& conf)
        : BT::SyncActionNode(action_name, conf),
          red_hero(100), red_engineer(300), red_infantry3(100), red_infantry4(100),
          red_sentry(400), red_outpost(1500), red_base(5000),
          blue_hero(100), blue_engineer(300), blue_infantry3(100), blue_infantry4(100),
          blue_sentry(400), blue_outpost(1500), blue_base(5000),
          last_sentry(400), last_our_base(5000), last_his_base(5000),
          our_sentry_blood(400), our_base_blood(5000), his_base_blood(5000),
          is_hurt(false), if_waiting_buy_blood(false), count_buy_blood(0),
          count_red_hero(0), count_red_engineer(0), count_red_infantry3(0), count_red_infantry4(0), count_red_sentry(0),
          count_blue_hero(0), count_blue_engineer(0), count_blue_infantry3(0), count_blue_infantry4(0),
          count_blue_sentry(0), count_our_dart(0), count_his_dart(0),
          red_hero_invincible(false), red_engineer_invincible(false), red_infantry3_invincible(false),
          red_infantry4_invincible(false), red_sentry_invincible(false), red_outpost_invincible(false),
          red_base_invincible(false), blue_hero_invincible(false), blue_engineer_invincible(false),
          blue_infantry3_invincible(false), blue_infantry4_invincible(false), blue_sentry_invincible(false),
          blue_outpost_invincible(false), blue_base_invincible(false),
          is_dart_hit_our(false), is_dart_hit_his(false)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        hp_sub_ = node_->create_subscription<rm_interfaces::msg::Hp>(
            "/robot/hp",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&RecordRobotBloodAction::hpCallback, this, std::placeholders::_1),
            sub_option);
        sentryinfo_sub_ = node_->create_subscription<rm_interfaces::msg::Sentryinfo>(
            "/robot/sentryinfo",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&RecordRobotBloodAction::sentryinfoCallback, this, std::placeholders::_1),
            sub_option);

        bt_loop_duration_ = config().blackboard->get<std::chrono::milliseconds>("bt_loop_duration");
        config().blackboard->get<bool>("is_we_are_blue", is_we_are_blue);
        config().blackboard->get<uint32_t>("buy_blood_number", buy_blood_number);

        config().blackboard->get<int>("our_dart_hit_count", our_dart_hit_count);
        config().blackboard->get<int>("his_dart_hit_count", his_dart_hit_count);

        count_size = 10 * 1000 / bt_loop_duration_.count(); // 10s
        count_size_our_dart = 0 * 1000 / bt_loop_duration_.count();
        count_size_his_dart = 0 * 1000 / bt_loop_duration_.count();
    }

    BT::NodeStatus RecordRobotBloodAction::tick()
    {
        //更新裁判系统数据以及从黑板中更新数据
        config().blackboard->get<bool>("is_we_are_blue", is_we_are_blue);
        config().blackboard->get<int>("our_dart_hit_count", our_dart_hit_count);
        config().blackboard->get<int>("his_dart_hit_count", his_dart_hit_count);
        /* 在callback之前获取用于前后对比的变量 */
        config().blackboard->get<uint32_t>("buy_blood_number", buy_blood_number);
        if (is_we_are_blue)
        {
            config().blackboard->get<int>("blue_sentry_blood", last_sentry);
            config().blackboard->get<int>("blue_base_blood", last_our_base);
            config().blackboard->get<int>("red_base_blood", last_his_base);
        }
        else
        {
            config().blackboard->get<int>("red_sentry_blood", last_sentry);
            config().blackboard->get<int>("red_base_blood", last_our_base);
            config().blackboard->get<int>("blue_base_blood", last_his_base);
        }
        callback_group_executor_.spin_some();
        if(is_we_are_blue)
        {
            our_sentry_blood = blue_sentry;
            our_base_blood = blue_base;
            his_base_blood = red_base;
        }
        else
        {
            our_sentry_blood = red_sentry;
            our_base_blood = red_base;
            his_base_blood = blue_base;
        }

        invincible_judge(); //机器人无敌判断
        dart_hit_judge(); //飞镖命中判断

        RCLCPP_INFO(node_->get_logger(), "our_sentry_blood: %d, last_sentry: %d", our_sentry_blood, last_sentry);
        // RCLCPP_INFO(node_->get_logger(),"if_waiting_buy_blood: %d", if_waiting_buy_blood);

        if (our_sentry_blood==0)
        {
            config().blackboard->set<std::string>("dead_count", "1");
            RCLCPP_INFO(node_->get_logger(), "set dead_count: 1");
        }
        /* 判断哨兵是否处于受伤状态以及是否远程买血成功 */
        if (last_sentry > our_sentry_blood)
        {
            is_hurt = true;
        }
        else
        {
            is_hurt = false;
            // if_waiting_buy_blood = false;
            if(our_sentry_blood - last_sentry > 100 && buy_blood_number == received_sentryinfo_.number_of_get_blood)
            {
                buy_blood_number = received_sentryinfo_.number_of_get_blood;
                RCLCPP_INFO(node_->get_logger(), "买血成功");
                if_waiting_buy_blood = false;
            }
            else if(our_sentry_blood - last_sentry < 100 && buy_blood_number - received_sentryinfo_.number_of_get_blood == 1)
            {
                count_buy_blood++;
                if_waiting_buy_blood = true;
                if(count_buy_blood >= count_size)
                {
                    buy_blood_number = received_sentryinfo_.number_of_get_blood;
                    count_buy_blood = 0;
                    RCLCPP_INFO(node_->get_logger(), "买血超时，已重置次数与裁判系统同步");
                    if_waiting_buy_blood = false;
                }
            }
            else if(buy_blood_number < received_sentryinfo_.number_of_get_blood ||
                buy_blood_number - received_sentryinfo_.number_of_get_blood >= 2)
            {
                buy_blood_number = received_sentryinfo_.number_of_get_blood;
                RCLCPP_INFO(node_->get_logger(), "买血计数错误，与裁判系统同步");
                if_waiting_buy_blood = false;
            }
        }

        /* 更新血量，无敌状态，飞镖命中情况和是否受伤 */
        config().blackboard->set<int>("red_hero_blood", red_hero);
        config().blackboard->set<int>("red_engineer_blood", red_engineer);
        config().blackboard->set<int>("red_infantry3_blood", red_infantry3);
        config().blackboard->set<int>("red_infantry4_blood", red_infantry4);
        config().blackboard->set<int>("red_sentry_blood", red_sentry);
        config().blackboard->set<int>("red_outpost_blood", red_outpost);
        config().blackboard->set<int>("red_base_blood", red_base);
        config().blackboard->set<int>("blue_hero_blood", blue_hero);
        config().blackboard->set<int>("blue_engineer_blood", blue_engineer);
        config().blackboard->set<int>("blue_infantry3_blood", blue_infantry3);
        config().blackboard->set<int>("blue_infantry4_blood", blue_infantry4);
        config().blackboard->set<int>("blue_sentry_blood", blue_sentry);
        config().blackboard->set<int>("blue_outpost_blood", blue_outpost);
        config().blackboard->set<int>("blue_base_blood", blue_base);

        config().blackboard->set<int>("our_dart_hit_count", our_dart_hit_count);
        config().blackboard->set<int>("his_dart_hit_count", his_dart_hit_count);

        config().blackboard->set<bool>("red_hero_invincible", red_hero_invincible);
        config().blackboard->set<bool>("red_engineer_invincible", red_engineer_invincible);
        config().blackboard->set<bool>("red_infantry3_invincible", red_infantry3_invincible);
        config().blackboard->set<bool>("red_infantry4_invincible", red_infantry4_invincible);
        config().blackboard->set<bool>("red_sentry_invincible", red_sentry_invincible);
        config().blackboard->set<bool>("red_outpost_invincible", red_outpost_invincible);
        config().blackboard->set<bool>("red_base_invincible", red_base_invincible);
        config().blackboard->set<bool>("blue_hero_invincible", blue_hero_invincible);
        config().blackboard->set<bool>("blue_engineer_invincible", blue_engineer_invincible);
        config().blackboard->set<bool>("blue_infantry3_invincible", blue_infantry3_invincible);
        config().blackboard->set<bool>("blue_infantry4_invincible", blue_infantry4_invincible);
        config().blackboard->set<bool>("blue_sentry_invincible", blue_sentry_invincible);
        config().blackboard->set<bool>("blue_outpost_invincible", blue_outpost_invincible);
        config().blackboard->set<bool>("blue_base_invincible", blue_base_invincible);

        config().blackboard->set<uint32_t>("buy_blood_number", buy_blood_number);
        config().blackboard->set<bool>("is_dart_hit_his", is_dart_hit_his);
        config().blackboard->set<bool>("is_dart_hit_our", is_dart_hit_our);
        config().blackboard->set<bool>("is_hurt", is_hurt);
        config().blackboard->set<bool>("if_waiting_buy_blood", if_waiting_buy_blood);

        RCLCPP_INFO(node_->get_logger(), "record robot blood successfully: %d", our_sentry_blood);

        return BT::NodeStatus::SUCCESS;
    }

    void RecordRobotBloodAction::invincible_judge()
    {
        /* 判断无敌剩余时间 */
        if (red_hero > 0 && red_hero_invincible)
        {
            count_red_hero++;
            if (count_red_hero >= count_size)
            {
                RCLCPP_INFO(node_->get_logger(), "红方英雄无敌解除");
                red_hero_invincible = false;
                count_red_hero = 0;
            }
        }
        if (red_engineer > 0 && red_engineer_invincible)
        {
            count_red_engineer++;
            if (count_red_engineer >= count_size)
            {
                RCLCPP_INFO(node_->get_logger(), "红方工程无敌解除");
                red_engineer_invincible = false;
                count_red_engineer = 0;
            }
        }
        if (red_infantry3 > 0 && red_infantry3_invincible)
        {
            count_red_infantry3++;
            if (count_red_infantry3 >= count_size)
            {
                RCLCPP_INFO(node_->get_logger(), "红方步兵3无敌解除");
                red_infantry3_invincible = false;
                count_red_infantry3 = 0;
            }
        }
        if (red_infantry4 > 0 && red_infantry4_invincible)
        {
            count_red_infantry4++;
            if (count_red_infantry4 >= count_size)
            {
                RCLCPP_INFO(node_->get_logger(), "红方步兵4无敌解除");
                red_infantry4_invincible = false;
                count_red_infantry4 = 0;
            }
        }
        if (red_sentry > 0 && red_sentry_invincible)
        {
            count_red_sentry++;
            if (count_red_sentry >= count_size)
            {
                RCLCPP_INFO(node_->get_logger(), "红方哨兵无敌解除");
                red_sentry_invincible = false;
                count_red_sentry = 0;
            }
        }
        if (blue_hero > 0 && blue_hero_invincible)
        {
            count_blue_hero++;
            if (count_blue_hero >= count_size)
            {
                RCLCPP_INFO(node_->get_logger(), "蓝方英雄无敌解除");
                blue_hero_invincible = false;
                count_blue_hero = 0;
            }
        }
        if (blue_engineer > 0 && blue_engineer_invincible)
        {
            count_blue_engineer++;
            if (count_blue_engineer >= count_size)
            {
                RCLCPP_INFO(node_->get_logger(), "蓝方工程无敌解除");
                blue_engineer_invincible = false;
                count_blue_engineer = 0;
            }
        }
        if (blue_infantry3 > 0 && blue_infantry3_invincible)
        {
            count_blue_infantry3++;
            if (count_blue_infantry3 >= count_size)
            {
                RCLCPP_INFO(node_->get_logger(), "蓝方步兵3无敌解除");
                blue_infantry3_invincible = false;
                count_blue_infantry3 = 0;
            }
        }
        if (blue_infantry4 > 0 && blue_infantry4_invincible)
        {
            count_blue_infantry4++;
            if (count_blue_infantry4 >= count_size)
            {
                RCLCPP_INFO(node_->get_logger(), "蓝方步兵4无敌解除");
                blue_infantry4_invincible = false;
                count_blue_infantry4 = 0;
            }
        }
        if (blue_sentry > 0 && blue_sentry_invincible)
        {
            count_blue_sentry++;
            if (count_blue_sentry >= count_size)
            {
                RCLCPP_INFO(node_->get_logger(), "蓝方哨兵无敌解除");
                blue_sentry_invincible = false;
                count_blue_sentry = 0;
            }
        }
        if(red_outpost > 0)
        {
            red_outpost_invincible = false;
            red_base_invincible = true;
        }
        else if(red_outpost == 0)
        {
            red_outpost_invincible = true;
            red_base_invincible = false;
        }
        if(blue_outpost > 0)
        {
            blue_outpost_invincible = false;
            blue_base_invincible = true;
        }
        else if(blue_outpost == 0)
        {
            blue_outpost_invincible = true;
            blue_base_invincible = false;
        }
    }

    void RecordRobotBloodAction::dart_hit_judge()
    {
        count_our_dart++;
        count_his_dart++;
        /* 判断两方飞镖命中基地的情况 */
        if (last_our_base - our_base_blood >= 625 && last_our_base != 0 && our_base_blood != 0)
        {
            is_dart_hit_his = true;
            his_dart_hit_count++;
            if(last_our_base - our_base_blood >= 1200)
            {
                count_size_his_dart += 15 * 1000 / bt_loop_duration_.count();
                RCLCPP_INFO(node_->get_logger(), "我方基地被飞镖命中随机移动目标");
            }
            else
            {
                switch(his_dart_hit_count)
                {
                    case 1 : count_size_his_dart += 10 * 1000 / bt_loop_duration_.count(); break;
                    case 2 : count_size_his_dart += 5 * 1000 / bt_loop_duration_.count(); break;
                    case 3 : count_size_his_dart += 3 * 1000 / bt_loop_duration_.count(); break;
                    case 4 : count_size_his_dart += 2 * 1000 / bt_loop_duration_.count(); break;
                    default: break;
                }
                RCLCPP_INFO(node_->get_logger(), "我方基地被飞镖命中");
            }
        }
        else
        {
            if(count_his_dart >= count_size_his_dart)
            {
                count_his_dart = 0;
                count_size_his_dart = 0;
                is_dart_hit_his = false;
                // RCLCPP_INFO(node_->get_logger(), "我方致盲时间结束");
            }
            else
                is_dart_hit_his = true;
        }

        if (last_his_base - his_base_blood >= 625 && last_his_base != 0 && his_base_blood != 0)
        {
            is_dart_hit_our = true;
            our_dart_hit_count++;
            if(last_his_base - his_base_blood >= 1200)
            {
                count_size_our_dart += 15 * 1000 / bt_loop_duration_.count();
                RCLCPP_INFO(node_->get_logger(), "敌方基地被飞镖命中随机移动目标");
            }
            else
            {
                switch(our_dart_hit_count)
                {
                case 1 : count_size_our_dart += 10 * 1000 / bt_loop_duration_.count(); break;
                case 2 : count_size_our_dart += 5 * 1000 / bt_loop_duration_.count(); break;
                case 3 : count_size_our_dart += 3 * 1000 / bt_loop_duration_.count(); break;
                case 4 : count_size_our_dart += 2 * 1000 / bt_loop_duration_.count(); break;
                default: break;
                }
                RCLCPP_INFO(node_->get_logger(), "敌方基地被飞镖命中");
            }
        }
        else
        {
            if(count_our_dart >= count_size_our_dart)
            {
                count_our_dart = 0;
                count_size_our_dart = 0;
                is_dart_hit_our = false;
                // RCLCPP_INFO(node_->get_logger(), "敌方致盲时间结束");
            }
            else
                is_dart_hit_our = true;
        }
    }

    void RecordRobotBloodAction::hpCallback(rm_interfaces::msg::Hp::SharedPtr msg)
    {
        red_hero = msg->red_1_robot_hp;
        red_engineer = msg->red_2_robot_hp;
        red_infantry3 = msg->red_3_robot_hp;
        red_infantry4 = msg->red_4_robot_hp;
        red_sentry = msg->red_7_robot_hp;
        red_outpost = msg->red_outpost_hp;
        red_base = msg->red_base_hp;
        blue_hero = msg->blue_1_robot_hp;
        blue_engineer = msg->blue_2_robot_hp;
        blue_infantry3 = msg->blue_3_robot_hp;
        blue_infantry4 = msg->blue_4_robot_hp;
        blue_sentry = msg->blue_7_robot_hp;
        blue_outpost = msg->blue_outpost_hp;
        blue_base = msg->blue_base_hp;

        //血量为0则认为进入无敌状态
        if (red_hero == 0)
        {
            red_hero_invincible = true;
        }
        if (red_engineer == 0)
        {
            red_engineer_invincible = true;
        }
        if (red_infantry3 == 0)
        {
            red_infantry3_invincible = true;
        }
        if (red_infantry4 == 0)
        {
            red_infantry4_invincible = true;
        }
        if (red_sentry == 0)
        {
            red_sentry_invincible = true;
        }
        if(red_outpost == 0)
        {
            red_outpost_invincible = true;
        }
        if(red_outpost > 0)
        {
            red_base_invincible = true;
        }
        if (blue_hero == 0)
        {
            blue_hero_invincible = true;
        }
        if (blue_engineer == 0)
        {
            blue_engineer_invincible = true;
        }
        if (blue_infantry3 == 0)
        {
            blue_infantry3_invincible = true;
        }
        if (blue_infantry4 == 0)
        {
            blue_infantry4_invincible = true;
        }
        if (blue_sentry == 0)
        {
            blue_sentry_invincible = true;
        }
        if(blue_outpost == 0)
        {
            blue_outpost_invincible = true;
        }
        if(blue_outpost > 0)
        {
            blue_base_invincible = true;
        }
    }

    void RecordRobotBloodAction::sentryinfoCallback(rm_interfaces::msg::Sentryinfo::SharedPtr msg)
    {
        received_sentryinfo_.number_of_get_blood = msg->number_of_get_blood; //当前已经远程买血的总次数
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::RecordRobotBloodAction>("RecordRobotBlood");
}
