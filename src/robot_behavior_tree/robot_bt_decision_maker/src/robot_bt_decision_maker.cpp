#include "robot_bt_decision_maker/robot_bt_decision_maker.h"

DecisionMakerNode::DecisionMakerNode(std::string name) : Node(name)
{
    RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());
    this->declare_parameter("loop_duration_in_millisec", 10);
    this->declare_parameter("server_timeout_in_millisec", 100);
    this->declare_parameter("plugin_lib_names", std::vector<std::string>());
    this->declare_parameter("bt_xml_filename", std::string(""));
    this->declare_parameter("is_we_are_blue", true);
    this->declare_parameter<std::vector<double>>("red_outpost_tunnel_x", std::vector<double>{0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("red_outpost_tunnel_y", std::vector<double>{0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("red_banana_tunnel_x", std::vector<double>{0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("red_banana_tunnel_y", std::vector<double>{0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("blue_outpost_tunnel_x", std::vector<double>{0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("blue_outpost_tunnel_y", std::vector<double>{0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("blue_banana_tunnel_x", std::vector<double>{0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("blue_banana_tunnel_y", std::vector<double>{0.0, 0.0, 0.0, 0.0});
    //切换参数相关
    this->declare_parameter("odom_frame_gimbal", std::string(""));
    this->declare_parameter("odom_frame_chassis", std::string(""));
    // 控云台导航参数
    this->declare_parameter("dt_ref_gimbal", 0.4);
    this->declare_parameter("max_global_plan_lookahead_dist_gimbal", 1.45);
    this->declare_parameter("max_vel_x_gimbal", 3.0);
    this->declare_parameter("max_vel_y_gimbal", 3.0);
    this->declare_parameter("max_vel_x_backwards_gimbal", 0.1);
    this->declare_parameter("max_vel_theta_gimbal", 2.0);
    this->declare_parameter("acc_lim_x_gimbal", 4.0);
    this->declare_parameter("acc_lim_y_gimbal", 4.0);
    this->declare_parameter("acc_lim_theta_gimbal", 5.0);
    // 控底盘导航参数
    this->declare_parameter("dt_ref_chassis", 0.6);
    this->declare_parameter("max_global_plan_lookahead_dist_chassis", 1.65);
    this->declare_parameter("max_vel_x_chassis", 3.0);
    this->declare_parameter("max_vel_y_chassis", 3.0);
    this->declare_parameter("max_vel_x_backwards_chassis", 0.1);
    this->declare_parameter("max_vel_theta_chassis", 1.7);
    this->declare_parameter("acc_lim_x_chassis", 5.0);
    this->declare_parameter("acc_lim_y_chassis", 5.0);
    this->declare_parameter("acc_lim_theta_chassis", 0.6);
    // 小陀螺导航参数
    this->declare_parameter("dt_ref_spin", 0.4);
    this->declare_parameter("max_global_plan_lookahead_dist_spin", 1.45);
    this->declare_parameter("max_vel_x_spin", 2.7);
    this->declare_parameter("max_vel_y_spin", 2.7);
    this->declare_parameter("max_vel_x_backwards_spin", 2.7);
    this->declare_parameter("max_vel_theta_spin", 3.0);
    this->declare_parameter("acc_lim_x_spin", 4.0);
    this->declare_parameter("acc_lim_y_spin", 4.0);
    this->declare_parameter("acc_lim_theta_spin", 5.0);
    this->declare_parameter("chasing_higher_limit", 0.06);
    this->declare_parameter("chasing_lower_limit", -0.3);
    // 打符红蓝方角度
    this->declare_parameter("dafu_yaw_angle_blue", -2.53);
    this->declare_parameter("dafu_yaw_angle_red", 0.61);

    this->get_parameter("loop_duration_in_millisec", loop_duration_in_millisec_);
    bt_loop_duration_ = std::chrono::milliseconds(loop_duration_in_millisec_);
    this->get_parameter("server_timeout_in_millisec", server_timeout_in_millisec_);
    server_timeout_ = std::chrono::milliseconds(server_timeout_in_millisec_);
    wait_for_service_timeout = server_timeout_;
    this->get_parameter("plugin_lib_names", plugin_lib_names_);
    this->get_parameter("bt_xml_filename", bt_xml_filename_);
    this->get_parameter("is_we_are_blue", is_we_are_blue_);
    this->get_parameter<std::vector<double>>("red_outpost_tunnel_x", red_outpost_tunnel_x_);
    this->get_parameter<std::vector<double>>("red_outpost_tunnel_y", red_outpost_tunnel_y_);
    this->get_parameter<std::vector<double>>("red_banana_tunnel_x", red_banana_tunnel_x_);
    this->get_parameter<std::vector<double>>("red_banana_tunnel_y", red_banana_tunnel_y_);
    this->get_parameter<std::vector<double>>("blue_outpost_tunnel_x", blue_outpost_tunnel_x_);
    this->get_parameter<std::vector<double>>("blue_outpost_tunnel_y", blue_outpost_tunnel_y_);
    this->get_parameter<std::vector<double>>("blue_banana_tunnel_x", blue_banana_tunnel_x_);
    this->get_parameter<std::vector<double>>("blue_banana_tunnel_y", blue_banana_tunnel_y_);
    this->get_parameter("odom_frame_gimbal", odom_frame_gimbal);
    this->get_parameter("odom_frame_chassis", odom_frame_chassis);
    this->get_parameter("dt_ref_gimbal", dt_ref_gimbal);
    this->get_parameter("max_global_plan_lookahead_dist_gimbal", max_global_plan_lookahead_dist_gimbal);
    this->get_parameter("max_vel_x_gimbal", max_vel_x_gimbal);
    this->get_parameter("max_vel_y_gimbal", max_vel_y_gimbal);
    this->get_parameter("max_vel_x_backwards_gimbal", max_vel_x_backwards_gimbal);
    this->get_parameter("max_vel_theta_gimbal", max_vel_theta_gimbal);
    this->get_parameter("acc_lim_x_gimbal", acc_lim_x_gimbal);
    this->get_parameter("acc_lim_y_gimbal", acc_lim_y_gimbal);
    this->get_parameter("acc_lim_theta_gimbal", acc_lim_theta_gimbal);
    this->get_parameter("dt_ref_chassis", dt_ref_chassis);
    this->get_parameter("max_global_plan_lookahead_dist_chassis", max_global_plan_lookahead_dist_chassis);
    this->get_parameter("max_vel_x_chassis", max_vel_x_chassis);
    this->get_parameter("max_vel_y_chassis", max_vel_y_chassis);
    this->get_parameter("max_vel_x_backwards_chassis", max_vel_x_backwards_chassis);
    this->get_parameter("max_vel_theta_chassis", max_vel_theta_chassis);
    this->get_parameter("acc_lim_x_chassis", acc_lim_x_chassis);
    this->get_parameter("acc_lim_y_chassis", acc_lim_y_chassis);
    this->get_parameter("acc_lim_theta_chassis", acc_lim_theta_chassis);
    this->get_parameter("dt_ref_spin", dt_ref_spin);
    this->get_parameter("max_global_plan_lookahead_dist_spin", max_global_plan_lookahead_dist_spin);
    this->get_parameter("max_vel_x_spin", max_vel_x_spin);
    this->get_parameter("max_vel_y_spin", max_vel_y_spin);
    this->get_parameter("max_vel_x_backwards_spin", max_vel_x_backwards_spin);
    this->get_parameter("max_vel_theta_spin", max_vel_theta_spin);
    this->get_parameter("acc_lim_x_spin", acc_lim_x_spin);
    this->get_parameter("acc_lim_y_spin", acc_lim_y_spin);
    this->get_parameter("acc_lim_theta_spin", acc_lim_theta_spin);
    this->get_parameter("chasing_higher_limit", chasing_higher_limit);
    this->get_parameter("chasing_lower_limit", chasing_lower_limit);
    this->get_parameter("dafu_yaw_angle_blue", dafu_yaw_angle_blue);
    this->get_parameter("dafu_yaw_angle_red", dafu_yaw_angle_red);

    if(is_we_are_blue_)
        dafu_yaw_angle = dafu_yaw_angle_blue;
    else
        dafu_yaw_angle = dafu_yaw_angle_red;

    p_pub_ = this->create_publisher<rm_interfaces::msg::Action>("/robot/action", 1);
    patrol_.max_pitch_w = 1.0;
    patrol_.min_pitch_w = -1.0;
    patrol_.mode = 0;
    patrol_.spin = 0;
    patrol_.patrol = 0;
    patrol_.robot_aim = 1;
    waitNav2();
    pose.header.stamp = now();
    pose.header.frame_id = "map";
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    dangerous_tunnel.resize(4);
    for(int i = 0; i < 4; i++)
        dangerous_tunnel[i] = false;
    client_node_name_ = this->get_name();
    auto options = rclcpp::NodeOptions().arguments(
        {
            "--ros-args",
            "-r",
            std::string("__node:=") +
            client_node_name_ + "_rclcpp_node",
            "--"
        });
    client_node_ = std::make_shared<rclcpp::Node>("_", options);

    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("robot_bt_decision_maker");
    bt_xml_filename_ = pkg_share_dir + bt_xml_filename_;
    bt_ = std::make_unique<nav2_behavior_tree::BehaviorTreeEngine>(plugin_lib_names_);
    tfbuffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface());
    tfbuffer_->setCreateTimerInterface(timer_interface);
    tfbuffer_->setUsingDedicatedThread(true);
    tflistener_ = std::make_shared<tf2_ros::TransformListener>(*tfbuffer_, this, false);

    // 初始化blackboard
    blackboard_ = BT::Blackboard::create();
    // Put items on the blackboard
    blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_); // NOLINT
    blackboard_->set<std::chrono::milliseconds>("server_timeout", server_timeout_); // NOLINT
    blackboard_->set<std::chrono::milliseconds>("wait_for_service_timeout", wait_for_service_timeout); // NOLINT
    blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_); // NOLINT
    blackboard_->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tfbuffer_); // NOLINT

    blackboard_->set<bool>("is_we_are_blue", is_we_are_blue_);
    blackboard_->set<bool>("if_in_mode_changing", false);
    blackboard_->set<double>("robot_pos_x", 0.0);
    blackboard_->set<double>("robot_pos_y", 0.0);

    /* mode相关 */
    blackboard_->set<float>("max_pitch", 0);
    blackboard_->set<float>("min_pitch", 0);
    blackboard_->set<int>("mode", 2); // 0--导航控底盘，1--导航控云台
    blackboard_->set<int>("spin", 0); // 0--不小陀螺，1--小陀螺
    blackboard_->set<int>("patrol", 0); // 0--云台不巡逻，1--云台巡逻
    blackboard_->set<int>("robot_aim", 1); // 0--不自瞄，1--自瞄

    /* sentry decision */
    blackboard_->set<uint32_t>("resurrection", 0); //选择当前复活方式
    blackboard_->set<uint16_t>("allowance_shoot_remaining", 300); //当前剩余发弹量
    blackboard_->set<uint32_t>("buy_bullet_remote_number", 0); //当前记录的远程买弹的总次数
    blackboard_->set<uint32_t>("buy_bullet_at_recovery", 0); //当前记录的在补给区买弹的总弹量
    blackboard_->set<uint32_t>("buy_blood_number", 0); //当前记录的远程买血的总次数
    blackboard_->set<uint32_t>("number_of_bullet_to_buy", 0); //需要在补给区购买的发弹量
    blackboard_->set<uint16_t>("recovery_bullet_to_acquire", 0); //补给区当前可领的免费发弹量
    blackboard_->set<uint16_t>("recovery_bullet_acquired", 0); //补给区已经领取的免费发弹量
	blackboard_->set<bool>("if_waiting_buy_blood", false);
    blackboard_->set<bool>("if_waiting_buy_bullet_remote", false);
    blackboard_->set<bool>("if_waiting_buy_bullet_at_recovery", false);

    blackboard_->set<bool>("is_hurt", false); //是否在受伤状态
    blackboard_->set<bool>("is_out_fight", true); //机器人是否脱战
    blackboard_->set<bool>("is_in_ramp", false); //是否在坡道
    blackboard_->set<bool>("is_cancel_resurrection", false); //是否取消哨兵复活
    blackboard_->set<bool>("is_goal_reached", false); //是否到达目标点
    blackboard_->set<bool>("is_part_of_referee_offline", false); //是否有裁判系统模块离线

    blackboard_->set<double>("distance", 0.0);

    /* 云台手控制内容 */
    blackboard_->set<bool>("in_command", false);
    blackboard_->set<std::string>("cmd_keyboard", "");
    blackboard_->set<bool>("is_force_move_forward", false); //强制前进
    blackboard_->set<bool>("is_force_move_backward", false); //强制后退
    blackboard_->set<bool>("is_force_move_left", false); //强制向左
    blackboard_->set<bool>("is_force_move_right", false); //强制向右
    blackboard_->set<bool>("is_force_move_complete", true);
    blackboard_->set<bool>("select_if_go_fort", true);
    blackboard_->set<bool>("select_force_no_resurrection", false);

    blackboard_->set<uint16_t>("receiver_id", 0);
    blackboard_->set<uint16_t>("sender_id", 0);

    blackboard_->set<geometry_msgs::msg::PoseStamped>("goal", pose);

    /* 红蓝方机器人血量 */
    blackboard_->set<int>("red_hero_blood", 100);
    blackboard_->set<int>("red_engineer_blood", 300);
    blackboard_->set<int>("red_infantry3_blood", 100);
    blackboard_->set<int>("red_infantry4_blood", 100);
    blackboard_->set<int>("red_sentry_blood", 400);
    blackboard_->set<int>("red_outpost_blood", 1500);
    blackboard_->set<int>("red_base_blood", 5000);
    blackboard_->set<int>("blue_hero_blood", 100);
    blackboard_->set<int>("blue_engineer_blood", 300);
    blackboard_->set<int>("blue_infantry3_blood", 100);
    blackboard_->set<int>("blue_infantry4_blood", 100);
    blackboard_->set<int>("blue_sentry_blood", 400);
    blackboard_->set<int>("blue_outpost_blood", 1500);
    blackboard_->set<int>("blue_base_blood", 5000);

    /* 红蓝方机器人是否在无敌状态 */
    blackboard_->set<bool>("red_hero_invincible", false);
    blackboard_->set<bool>("red_engineer_invincible", false);
    blackboard_->set<bool>("red_infantry3_invincible", false);
    blackboard_->set<bool>("red_infantry4_invincible", false);
    blackboard_->set<bool>("red_sentry_invincible", false);
    blackboard_->set<bool>("red_outpost_invincible", false);
    blackboard_->set<bool>("red_base_invincible", false);
    blackboard_->set<bool>("blue_hero_invincible", false);
    blackboard_->set<bool>("blue_engineer_invincible", false);
    blackboard_->set<bool>("blue_infantry3_invincible", false);
    blackboard_->set<bool>("blue_infantry4_invincible", false);
    blackboard_->set<bool>("blue_sentry_invincible", false);
    blackboard_->set<bool>("blue_outpost_invincible", false);
    blackboard_->set<bool>("blue_base_invincible", false);

    // 双方飞镖是否击中基地
    blackboard_->set<bool>("is_dart_hit_his", false);
    blackboard_->set<bool>("is_dart_hit_our", false);
    blackboard_->set<int>("our_dart_hit_count", 0);
    blackboard_->set<int>("his_dart_hit_count", 0);

    blackboard_->set<int>("health_threshold", 80);
    blackboard_->set<int>("decision_mode", 0); //当前任务模式

    /* 白名单 */
    blackboard_->set<uint8_t>("whitelist_hero", 1);
    blackboard_->set<uint8_t>("whitelist_engineer", 1);
    blackboard_->set<uint8_t>("whitelist_infantry3", 1);
    blackboard_->set<uint8_t>("whitelist_infantry4", 1);
    blackboard_->set<uint8_t>("whitelist_sentry", 1);
    blackboard_->set<uint8_t>("whitelist_outpost", 1);
    blackboard_->set<uint8_t>("whitelist_base", 1);

    //四个狗洞范围端点的 x, y坐标
    blackboard_->set<std::vector<double>>("red_outpost_tunnel_x", red_outpost_tunnel_x_);
    blackboard_->set<std::vector<double>>("red_outpost_tunnel_y", red_outpost_tunnel_y_);
    blackboard_->set<std::vector<double>>("red_banana_tunnel_x", red_banana_tunnel_x_);
    blackboard_->set<std::vector<double>>("red_banana_tunnel_y", red_banana_tunnel_y_);
    blackboard_->set<std::vector<double>>("blue_outpost_tunnel_x", blue_outpost_tunnel_x_);
    blackboard_->set<std::vector<double>>("blue_outpost_tunnel_y", blue_outpost_tunnel_y_);
    blackboard_->set<std::vector<double>>("blue_banana_tunnel_x", blue_banana_tunnel_x_);
    blackboard_->set<std::vector<double>>("blue_banana_tunnel_y", blue_banana_tunnel_y_);
    //路径上经过的狗洞标号
    blackboard_->set<int>("tunnel_index", 0);
    //当前是否在狗洞中
    blackboard_->set<bool>("if_in_tunnel", false);
    //狗洞是否有人堵
    blackboard_->set<std::vector<bool>>("dangerous_tunnel", dangerous_tunnel);
    //和狗洞可通过情况相关的地图
    blackboard_->set<std::string>("tunnel_related_map", "");

    //是否进入虚弱状态
    blackboard_->set<bool>("is_in_weak", false);

    //追击相关
    blackboard_->set<double>("enemy_position_x", 0.0); //追击敌人坐标x
    blackboard_->set<double>("enemy_position_y", 0.0); //追击敌人坐标y
    blackboard_->set<bool>("is_in_chase", false); //是否在受自瞄控制目标点的追击状态
    blackboard_->set<double>("chasing_higher_limit", chasing_higher_limit); //追击高阈值
    blackboard_->set<double>("chasing_lower_limit", chasing_lower_limit); //追击低阈值

    //切换参数相关
    //坐标系
    blackboard_->set<std::string>("odom_frame_gimbal", odom_frame_gimbal);
    blackboard_->set<std::string>("odom_frame_chassis", odom_frame_chassis);
    // 控云台导航参数
    blackboard_->set<double>("dt_ref_gimbal", dt_ref_gimbal);
    blackboard_->set<double>("max_global_plan_lookahead_dist_gimbal", max_global_plan_lookahead_dist_gimbal);
    blackboard_->set<double>("max_vel_x_gimbal", max_vel_x_gimbal);
    blackboard_->set<double>("max_vel_y_gimbal", max_vel_y_gimbal);
    blackboard_->set<double>("max_vel_x_backwards_gimbal", max_vel_x_backwards_gimbal);
    blackboard_->set<double>("max_vel_theta_gimbal", max_vel_theta_gimbal);
    blackboard_->set<double>("acc_lim_x_gimbal", acc_lim_x_gimbal);
    blackboard_->set<double>("acc_lim_y_gimbal", acc_lim_y_gimbal);
    blackboard_->set<double>("acc_lim_theta_gimbal", acc_lim_theta_gimbal);
    // 控底盘导航参数
    blackboard_->set<double>("dt_ref_chassis", dt_ref_chassis);
    blackboard_->set<double>("max_global_plan_lookahead_dist_chassis", max_global_plan_lookahead_dist_chassis);
    blackboard_->set<double>("max_vel_x_chassis", max_vel_x_chassis);
    blackboard_->set<double>("max_vel_y_chassis", max_vel_y_chassis);
    blackboard_->set<double>("max_vel_x_backwards_chassis", max_vel_x_backwards_chassis);
    blackboard_->set<double>("max_vel_theta_chassis", max_vel_theta_chassis);
    blackboard_->set<double>("acc_lim_x_chassis", acc_lim_x_chassis);
    blackboard_->set<double>("acc_lim_y_chassis", acc_lim_y_chassis);
    blackboard_->set<double>("acc_lim_theta_chassis", acc_lim_theta_chassis);
    // 小陀螺导航参数
    blackboard_->set<double>("dt_ref_spin", dt_ref_spin);
    blackboard_->set<double>("max_global_plan_lookahead_dist_spin", max_global_plan_lookahead_dist_spin);
    blackboard_->set<double>("max_vel_x_spin", max_vel_x_spin);
    blackboard_->set<double>("max_vel_y_spin", max_vel_y_spin);
    blackboard_->set<double>("max_vel_x_backwards_spin", max_vel_x_backwards_spin);
    blackboard_->set<double>("max_vel_theta_spin", max_vel_theta_spin);
    blackboard_->set<double>("acc_lim_x_spin", acc_lim_x_spin);
    blackboard_->set<double>("acc_lim_y_spin", acc_lim_y_spin);
    blackboard_->set<double>("acc_lim_theta_spin", acc_lim_theta_spin);

    // 死亡计数
    blackboard_->set<std::string>("dead_count", "0");

    //打符相关
    blackboard_->set<bool>("if_dafu", true); //当前是否准备去打符
    blackboard_->set<double>("dafu_yaw_angle", dafu_yaw_angle); //打符朝向yaw角度

    if (!loadBehaviorTree(bt_xml_filename_, blackboard_))
    {
        RCLCPP_ERROR(this->get_logger(), "加载行为树失败.");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "bt_xml: %s", bt_xml_filename_.c_str());
    tflistener_ = std::make_shared<tf2_ros::TransformListener>(*tfbuffer_);
}

nav2_behavior_tree::BtStatus DecisionMakerNode::runBehaviorTree()
{
    auto is_canceling = [this]() -> bool
    {
        return false;
    };
    auto on_loop = [this]() -> void
    {
        // RCLCPP_INFO(this->get_logger(), "行为树正在运行...");
        rclcpp::spin_some(this->get_node_base_interface());
    };
    // Run the Behavior Tree
    // on_loop和is_canceling都必须是一个函数
    return bt_->run(&tree_, on_loop, is_canceling, bt_loop_duration_);
}

void DecisionMakerNode::waitNav2()
{
    std::string node_service = "/bt_navigator/get_state";
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client = this->create_client<
        lifecycle_msgs::srv::GetState>(node_service); //请求行为树服务的状态
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for bt_navigator to be available");
        p_pub_->publish(patrol_);
    }
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    std::string state = "unknown";
    // while (state != "active")
    // {
    //     RCLCPP_INFO(this->get_logger(), "等待定位程序启动");
    //     // p_pub_->publish(patrol_);
    //     auto future = client->async_send_request(request);
    //     rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, std::chrono::seconds(2));
    //     auto result = future.get(); // 获取结果并存储在变量中
    //     if (result)
    //     {
    //         state = result->current_state.label;
    //     }
    //     else
    //     {
    //         RCLCPP_INFO(this->get_logger(), "请求超时");
    //     }
    //     rclcpp::Rate(1).sleep();
    // }
}

bool DecisionMakerNode::loadBehaviorTree(const std::string& bt_xml_filename, BT::Blackboard::Ptr blackboard)
{
    // Read the input BT XML from the specified file into a string
    std::ifstream xml_file(bt_xml_filename);

    if (!xml_file.good())
    {
        RCLCPP_ERROR(this->get_logger(), "Couldn't open input XML file: %s", bt_xml_filename.c_str());
        return false;
    }

    //auto xml_string = std::string(
    //std::istreambuf_iterator<char>(xml_file),
    // std::istreambuf_iterator<char>());

    // Create the Behavior Tree from the XML input
    try
    {
        tree_ = bt_->createTreeFromFile(bt_xml_filename, blackboard);
        for (auto& blackboard : tree_.blackboard_stack) //set注册所有全局变量
        {
            blackboard->set<rclcpp::Node::SharedPtr>("node", client_node_); // NOLINT
            blackboard->set<std::chrono::milliseconds>("server_timeout", server_timeout_); // NOLINT
            blackboard->set<std::chrono::milliseconds>("wait_for_service_timeout", wait_for_service_timeout);
            blackboard->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_); // NOLINT
            blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tfbuffer_); // NOLINT

            blackboard->set<bool>("is_we_are_blue", is_we_are_blue_);
            blackboard->set<bool>("if_in_mode_changing", false);
            blackboard->set<double>("robot_pos_x", 0.0);
            blackboard->set<double>("robot_pos_y", 0.0);

            blackboard->set<float>("max_pitch", 0);
            blackboard->set<float>("min_pitch", 0);
            blackboard->set<int>("mode", 2);
            blackboard->set<int>("spin", 0);
            blackboard->set<int>("patrol", 0);
            blackboard->set<int>("robot_aim", 1);

            blackboard->set<uint32_t>("resurrection", 0);
            blackboard->set<uint16_t>("allowance_shoot_remaining", 300);
            blackboard->set<uint32_t>("buy_bullet_remote_number", 0);
            blackboard->set<uint32_t>("buy_bullet_at_recovery", 0);
            blackboard->set<uint32_t>("buy_blood_number", 0);
            blackboard->set<uint32_t>("number_of_bullet_to_buy", 0);
            blackboard->set<uint16_t>("recovery_bullet_to_acquire", 0);
            blackboard->set<uint16_t>("recovery_bullet_acquired", 0);
            blackboard->set<bool>("if_waiting_buy_blood", false);
    		blackboard->set<bool>("if_waiting_buy_bullet_remote", false);
    		blackboard->set<bool>("if_waiting_buy_bullet_at_recovery", false);

            blackboard->set<bool>("is_hurt", false);
            blackboard->set<bool>("is_out_fight", true);
            blackboard->set<bool>("is_in_ramp", false);
            blackboard->set<bool>("is_cancel_resurrection", false);
            blackboard->set<bool>("is_goal_reached", false);
            blackboard->set<bool>("is_part_of_referee_offline", false);

            blackboard->set<double>("distance", 0.0);

            blackboard->set<bool>("in_command", false);
            blackboard->set<std::string>("cmd_keyboard", "");
            blackboard->set<bool>("is_force_move_forward", false);
            blackboard->set<bool>("is_force_move_backward", false);
            blackboard->set<bool>("is_force_move_left", false);
            blackboard->set<bool>("is_force_move_right", false);
            blackboard->set<bool>("is_force_move_complete", true);
            blackboard->set<bool>("select_if_go_fort", true);
            blackboard->set<bool>("select_force_no_resurrection", false);

            blackboard->set<uint16_t>("receiver_id", 0);
            blackboard->set<uint16_t>("sender_id", 0);

            blackboard->set<geometry_msgs::msg::PoseStamped>("goal", pose);

            blackboard->set<int>("red_hero_blood", 100);
            blackboard->set<int>("red_engineer_blood", 300);
            blackboard->set<int>("red_infantry3_blood", 100);
            blackboard->set<int>("red_infantry4_blood", 100);
            blackboard->set<int>("red_sentry_blood", 400);
            blackboard->set<int>("red_outpost_blood", 1500);
            blackboard->set<int>("red_base_blood", 5000);
            blackboard->set<int>("blue_hero_blood", 100);
            blackboard->set<int>("blue_engineer_blood", 300);
            blackboard->set<int>("blue_infantry3_blood", 100);
            blackboard->set<int>("blue_infantry4_blood", 100);
            blackboard->set<int>("blue_sentry_blood", 400);
            blackboard->set<int>("blue_outpost_blood", 1500);
            blackboard->set<int>("blue_base_blood", 5000);

            blackboard->set<bool>("red_hero_invincible", false);
            blackboard->set<bool>("red_engineer_invincible", false);
            blackboard->set<bool>("red_infantry3_invincible", false);
            blackboard->set<bool>("red_infantry4_invincible", false);
            blackboard->set<bool>("red_sentry_invincible", false);
            blackboard->set<bool>("red_outpost_invincible", false);
            blackboard->set<bool>("red_base_invincible", false);
            blackboard->set<bool>("blue_hero_invincible", false);
            blackboard->set<bool>("blue_engineer_invincible", false);
            blackboard->set<bool>("blue_infantry3_invincible", false);
            blackboard->set<bool>("blue_infantry4_invincible", false);
            blackboard->set<bool>("blue_sentry_invincible", false);
            blackboard->set<bool>("blue_outpost_invincible", false);
            blackboard->set<bool>("blue_base_invincible", false);

            blackboard->set<bool>("is_dart_hit_his", false);
    		blackboard->set<bool>("is_dart_hit_our", false);
            blackboard->set<int>("our_dart_hit_count", 0);
    		blackboard->set<int>("his_dart_hit_count", 0);

            blackboard->set<int>("health_threshold", 80);
            blackboard->set<int>("decision_mode", 0);

            blackboard->set<uint8_t>("whitelist_hero", 1);
            blackboard->set<uint8_t>("whitelist_engineer", 1);
            blackboard->set<uint8_t>("whitelist_infantry3", 1);
            blackboard->set<uint8_t>("whitelist_infantry4", 1);
            blackboard->set<uint8_t>("whitelist_sentry", 1);
            blackboard->set<uint8_t>("whitelist_outpost", 1);
            blackboard->set<uint8_t>("whitelist_base", 1);

            blackboard->set<std::vector<double>>("red_outpost_tunnel_x", red_outpost_tunnel_x_);
            blackboard->set<std::vector<double>>("red_outpost_tunnel_y", red_outpost_tunnel_y_);
            blackboard->set<std::vector<double>>("red_banana_tunnel_x", red_banana_tunnel_x_);
            blackboard->set<std::vector<double>>("red_banana_tunnel_y", red_banana_tunnel_y_);
            blackboard->set<std::vector<double>>("blue_outpost_tunnel_x", blue_outpost_tunnel_x_);
            blackboard->set<std::vector<double>>("blue_outpost_tunnel_y", blue_outpost_tunnel_y_);
            blackboard->set<std::vector<double>>("blue_banana_tunnel_x", blue_banana_tunnel_x_);
            blackboard->set<std::vector<double>>("blue_banana_tunnel_y", blue_banana_tunnel_y_);
            blackboard->set<int>("tunnel_index", 0);
            blackboard->set<bool>("if_in_tunnel", false);
            blackboard->set<std::string>("tunnel_related_map", "");

            blackboard->set<bool>("is_in_weak", false);

            blackboard->set<double>("enemy_position_x", 0.0);
            blackboard->set<double>("enemy_position_y", 0.0);
            blackboard->set<bool>("is_in_chase", false);
            blackboard->set<double>("chasing_higher_limit", chasing_higher_limit);
            blackboard->set<double>("chasing_lower_limit", chasing_lower_limit);

            blackboard->set<std::string>("odom_frame_gimbal", odom_frame_gimbal);
            blackboard->set<std::string>("odom_frame_chassis", odom_frame_chassis);

            blackboard->set<double>("dt_ref_gimbal", dt_ref_gimbal);
            blackboard->set<double>("max_global_plan_lookahead_dist_gimbal", max_global_plan_lookahead_dist_gimbal);
            blackboard->set<double>("max_vel_x_gimbal", max_vel_x_gimbal);
            blackboard->set<double>("max_vel_y_gimbal", max_vel_y_gimbal);
            blackboard->set<double>("max_vel_x_backwards_gimbal", max_vel_x_backwards_gimbal);
            blackboard->set<double>("max_vel_theta_gimbal", max_vel_theta_gimbal);
            blackboard->set<double>("acc_lim_x_gimbal", acc_lim_x_gimbal);
            blackboard->set<double>("acc_lim_y_gimbal", acc_lim_y_gimbal);
            blackboard->set<double>("acc_lim_theta_gimbal", acc_lim_theta_gimbal);

            blackboard->set<double>("dt_ref_chassis", dt_ref_chassis);
            blackboard->set<double>("max_global_plan_lookahead_dist_chassis", max_global_plan_lookahead_dist_chassis);
            blackboard->set<double>("max_vel_x_chassis", max_vel_x_chassis);
            blackboard->set<double>("max_vel_y_chassis", max_vel_y_chassis);
            blackboard->set<double>("max_vel_x_backwards_chassis", max_vel_x_backwards_chassis);
            blackboard->set<double>("max_vel_theta_chassis", max_vel_theta_chassis);
            blackboard->set<double>("acc_lim_x_chassis", acc_lim_x_chassis);
            blackboard->set<double>("acc_lim_y_chassis", acc_lim_y_chassis);
            blackboard->set<double>("acc_lim_theta_chassis", acc_lim_theta_chassis);

            blackboard->set<double>("dt_ref_spin", dt_ref_spin);
            blackboard->set<double>("max_global_plan_lookahead_dist_spin", max_global_plan_lookahead_dist_spin);
            blackboard->set<double>("max_vel_x_spin", max_vel_x_spin);
            blackboard->set<double>("max_vel_y_spin", max_vel_y_spin);
            blackboard->set<double>("max_vel_x_backwards_spin", max_vel_x_backwards_spin);
            blackboard->set<double>("max_vel_theta_spin", max_vel_theta_spin);
            blackboard->set<double>("acc_lim_x_spin", acc_lim_x_spin);
            blackboard->set<double>("acc_lim_y_spin", acc_lim_y_spin);
            blackboard->set<double>("acc_lim_theta_spin", acc_lim_theta_spin);

            blackboard->set<std::string>("dead_count", "0");

            blackboard->set<bool>("if_dafu", true);
            blackboard->set<double>("dafu_yaw_angle", dafu_yaw_angle);
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Exception when loading BT: %s", e.what());
        return false;
    }
    return true;
}
