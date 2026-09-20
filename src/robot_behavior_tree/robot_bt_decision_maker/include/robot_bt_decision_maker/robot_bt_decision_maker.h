#include <nav2_behavior_tree/behavior_tree_engine.hpp>
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/create_timer_ros.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rm_interfaces/msg/action.hpp"

class DecisionMakerNode : public rclcpp::Node
{
public:
    /*
     * @brief 构造函数,初始化决策树所需参数和导航参数
     * @param std::string name节点名称
     */
    explicit DecisionMakerNode(std::string name);

    /*
     * @brief 运行决策树节点
     * @return bt_->run行为树正常运行函数
     */
    nav2_behavior_tree::BtStatus runBehaviorTree();
    /*
     * @brief 等待行为树服务被激活以及雷达定位启动
     */
    void waitNav2();
    /*
     * @brief 加载行为树
     * @param std::string &bt_xml_filename 行为树xml文件名的引用
     * @param BT::Blackboard::Ptr blackboard 行为树执行用的blackboard
     * @return 是否加载成功
     */
    bool loadBehaviorTree(const std::string &bt_xml_filename, BT::Blackboard::Ptr blackboard);

private:
    // Parameters
    int loop_duration_in_millisec_; // 行为树循环时间间隔（频率，毫秒）
    int server_timeout_in_millisec_; // 服务等待超时时间（毫秒）
    std::vector<std::string> plugin_lib_names_; // 加载插件
    std::string bt_xml_filename_; // 行为树xml文件名
    bool is_we_are_blue_; //判断是否是蓝方

    std::unique_ptr<nav2_behavior_tree::BehaviorTreeEngine> bt_; // 行为树Engine，需要用plugin_lib_names来初始化
    BT::Tree tree_; // 行为树
    BT::Blackboard::Ptr blackboard_; // 黑板全局变量
    std::chrono::milliseconds bt_loop_duration_; // 行为树循环时间间隔（频率，毫秒）
    std::chrono::milliseconds server_timeout_; // 服务等待超时时间（毫秒）
    std::chrono::milliseconds wait_for_service_timeout; // 服务等待超时时间（毫秒）
    std::string client_node_name_; // 客户端节点名
    rclcpp::Node::SharedPtr client_node_; // 客户端节点

    std::shared_ptr<tf2_ros::Buffer> tfbuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tflistener_;
    geometry_msgs::msg::PoseStamped pose;
    rclcpp::Publisher<rm_interfaces::msg::Action>::SharedPtr p_pub_;
    rm_interfaces::msg::Action patrol_;

    ///红方前哨狗洞四个范围端点 x
    std::vector<double> red_outpost_tunnel_x_;
    ///红方前哨狗洞四个范围端点 y
    std::vector<double> red_outpost_tunnel_y_;
    ///红方沟槽（香蕉道）六个范围端点 x
    std::vector<double> red_banana_tunnel_x_;
    ///红方沟槽（香蕉道）六个范围端点 y
    std::vector<double> red_banana_tunnel_y_;
    ///蓝方前哨狗洞四个范围端点 x
    std::vector<double> blue_outpost_tunnel_x_;
    ///蓝方前哨狗洞四个范围端点 y
    std::vector<double> blue_outpost_tunnel_y_;
    ///蓝方沟槽（香蕉道）六个范围端点 x
    std::vector<double> blue_banana_tunnel_x_;
    ///蓝方沟槽（香蕉道）六个范围端点 y
    std::vector<double> blue_banana_tunnel_y_;
    ///狗洞是否有人堵
    std::vector<bool> dangerous_tunnel;

    /* 切换参数相关 */
    std::string odom_frame_gimbal; //控云台导航规划坐标系
    std::string odom_frame_chassis; //控底盘导航规划坐标系
    // 控云台导航参数
    double dt_ref_gimbal;
    double max_global_plan_lookahead_dist_gimbal;
    double max_vel_x_gimbal;
    double max_vel_y_gimbal;
    double max_vel_x_backwards_gimbal;
    double max_vel_theta_gimbal;
    double acc_lim_x_gimbal;
    double acc_lim_y_gimbal;
    double acc_lim_theta_gimbal;
    // 控底盘导航参数
    double dt_ref_chassis;
    double max_global_plan_lookahead_dist_chassis;
    double max_vel_x_chassis;
    double max_vel_y_chassis;
    double max_vel_x_backwards_chassis;
    double max_vel_theta_chassis;
    double acc_lim_x_chassis;
    double acc_lim_y_chassis;
    double acc_lim_theta_chassis;
    // 小陀螺导航参数
    double dt_ref_spin;
    double max_global_plan_lookahead_dist_spin;
    double max_vel_x_spin;
    double max_vel_y_spin;
    double max_vel_x_backwards_spin;
    double max_vel_theta_spin;
    double acc_lim_x_spin;
    double acc_lim_y_spin;
    double acc_lim_theta_spin;

    /* 追击高低阈值 */
    double chasing_higher_limit;
    double chasing_lower_limit;

    // 打符红蓝方角度（弧度）
    double dafu_yaw_angle_blue;
    double dafu_yaw_angle_red;
    double dafu_yaw_angle;
};