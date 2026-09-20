//
// Created by mijiao on 23-11-20.
//

#ifndef ROBOT_SERIAL_ROBOT_SERIAL_H
#define ROBOT_SERIAL_ROBOT_SERIAL_H

#include <iomanip>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "sentry_msg.h"
#include "robot_message.h"
#include "robot_referee.h"
#include "robot_sentry.h"

#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class RobotSerial : public rclcpp::Node {
private:
    //uint16_t sender_id = 0x107;
    sentry::SentrySerial sentrySerial; //串口
    rclcpp::Clock rosClock;

    int dog_cnt_;
    int dog_threshold_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds period_ns;
    double frequency; //计时器频率

    // 初始化发布者和订阅者
    rclcpp::Publisher<rm_interfaces::msg::Buff>::SharedPtr BuffPublisher;
    rclcpp::Publisher<rm_interfaces::msg::Dartinfo>::SharedPtr DartinfoPublisher;
    rclcpp::Publisher<rm_interfaces::msg::Event>::SharedPtr EventPublisher;
    rclcpp::Publisher<rm_interfaces::msg::Gamestatus>::SharedPtr GamestatusPublisher;
    rclcpp::Publisher<rm_interfaces::msg::Hp>::SharedPtr HpPublisher;
    rclcpp::Publisher<rm_interfaces::msg::Interaction>::SharedPtr InteractionPublisher;
    rclcpp::Publisher<rm_interfaces::msg::Mapcommand>::SharedPtr MapcommandPublisher;
    rclcpp::Publisher<rm_interfaces::msg::Projectileallowance>::SharedPtr ProjectileallowancePublisher;
    rclcpp::Publisher<rm_interfaces::msg::HurtData>::SharedPtr HurtDataPublisher;
    rclcpp::Publisher<rm_interfaces::msg::Rfidstatus>::SharedPtr RfidstatusPublisher;
    rclcpp::Publisher<rm_interfaces::msg::Robotp>::SharedPtr RobotpPublisher;
    rclcpp::Publisher<rm_interfaces::msg::Robotposition>::SharedPtr RobotpositionPublisher;
    rclcpp::Publisher<rm_interfaces::msg::Robotstatus>::SharedPtr RobotstatusPublisher;
    rclcpp::Publisher<rm_interfaces::msg::Sentryinfo>::SharedPtr SentryinfoPublisher;
    rclcpp::Publisher<rm_interfaces::msg::Lidarstation>::SharedPtr LidarstationPublisher;
    rclcpp::Publisher<rm_interfaces::msg::AngleError>::SharedPtr AngleErrorPublisher;

    rclcpp::Subscription<rm_interfaces::msg::Action>::SharedPtr ActionSubscription;
    rclcpp::Subscription<rm_interfaces::msg::Decision>::SharedPtr DecisionSubscription;
    rclcpp::Subscription<rm_interfaces::msg::Map>::SharedPtr MapSubscription;
    rclcpp::Subscription<rm_interfaces::msg::Info>::SharedPtr InfoSubscription;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr VelocitySubscription;
    rclcpp::Subscription<rm_interfaces::msg::Whitelist>::SharedPtr WhitelistSubscription;

    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf_broadcaster;


    // ros回调函数
    void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void actionCallback(const rm_interfaces::msg::Action::SharedPtr msg);
    void sentry_cmd_Callback(const rm_interfaces::msg::Decision::SharedPtr msg);
    void custom_info_Callback(const rm_interfaces::msg::Info::SharedPtr msg);
    void map_data_Callback(const rm_interfaces::msg::Map::SharedPtr msg);
    void whitelistCallback(const rm_interfaces::msg::Whitelist::SharedPtr msg);

    void timer_callback();

public:
    explicit RobotSerial();
};
 
#endif //ROBOT_SERIAL_ROBOT_SERIAL_H
