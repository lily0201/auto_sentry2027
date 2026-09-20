#include "lidar_monitor/lidar_monitor.h"

LidarMonitorNode::LidarMonitorNode(std::string node_name)
    : Node("lidar_monitor_node")
{
    RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", node_name.c_str());
    this->declare_parameter("lidar_topic_1", "/livox/lidar_192_168_1_187");
    this->declare_parameter("lidar_topic_2", "/livox/lidar_192_168_1_104");
    this->declare_parameter("imu_topic_1", "/livox/imu_192_168_1_187");
    this->declare_parameter("imu_topic_2", "/livox/imu_192_168_1_104");
    this->declare_parameter("livox_frame_id_1", "livox_192_168_1_187");
    this->declare_parameter("livox_frame_id_2", "livox_192_168_1_104");
    this->declare_parameter("output_lidar_topic", "/livox/lidar");
    this->declare_parameter("output_imu_topic", "/livox/imu");
    this->declare_parameter("lidar_1_dog", 1000);
    this->declare_parameter("lidar_2_dog", 1000);
    this->declare_parameter("frequency", 100.0);

    this->get_parameter("lidar_topic_1", lidar_topic_1_);
    this->get_parameter("lidar_topic_2", lidar_topic_2_);
    this->get_parameter("imu_topic_1", imu_topic_1_);
    this->get_parameter("imu_topic_2", imu_topic_2_);
    this->get_parameter("output_lidar_topic", output_lidar_topic_);
    this->get_parameter("output_imu_topic", output_imu_topic_);
    this->get_parameter("livox_frame_id_1", livox_frame_id_1_);
    this->get_parameter("livox_frame_id_2", livox_frame_id_2_);
    this->get_parameter("lidar_1_dog", lidar_1_dog_);
    this->get_parameter("lidar_2_dog", lidar_2_dog_);
    this->get_parameter("frequency", frequency);

    lidar_flag_ = 0;
    pub_cnt_ = 0;
    print_cnt_ = 0;
    imu_print_cnt_ = 0;
    lidar_1_cnt_ = 0;
    lidar_2_cnt_ = 0;
    lidar_flag_ = 0;
    period_ns = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / frequency));

    lidar_1_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lidar_topic_1_, 10,
        std::bind(&LidarMonitorNode::lidar1Callback, this, std::placeholders::_1));
    lidar_2_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lidar_topic_2_, 10,
        std::bind(&LidarMonitorNode::lidar2Callback, this, std::placeholders::_1));
    imu_1_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic_1_, 10,
        std::bind(&LidarMonitorNode::imu1Callback, this, std::placeholders::_1));
    imu_2_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic_2_, 10,
        std::bind(&LidarMonitorNode::imu2Callback, this, std::placeholders::_1));
    lidar_flag_pub_ = this->create_publisher<std_msgs::msg::Int32>("/lidar_flag", 10);
    output_lidar_pub_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>(output_lidar_topic_, 10);
    output_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(output_imu_topic_, 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(period_ns, std::bind(&LidarMonitorNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "%s节点初始化完成.", node_name.c_str());
}

void LidarMonitorNode::lidar1Callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
{
    lidar_1_cnt_ = 0;
    if(lidar_flag_ == 1) //只有187雷达正常，187雷达变为主雷达直至104恢复
    {
        //187雷达转回自己的坐标系
        livox_ros_driver2::msg::CustomMsg::SharedPtr output_msg = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
        output_msg->points.reserve(msg->point_num);
        geometry_msgs::msg::TransformStamped lidar_two_to_one =
                tf_buffer_->lookupTransform("livox_192_168_1_187","livox_192_168_1_104",  rclcpp::Time(), rclcpp::Duration::from_seconds(0.5));
        Eigen::Affine3d transformLidarTwo2One=tf2::transformToEigen(lidar_two_to_one);
        lidar_transformed(msg, output_msg,transformLidarTwo2One);

        // auto tf=rclcpp::Clock().now();
        // double tf_second=tf.seconds();
        // RCLCPP_INFO(this->get_logger(), "transform finsh use %f",tf_second-begin_second,"s");
        output_msg->header.stamp = msg->header.stamp;
        output_msg->header.frame_id = livox_frame_id_1_;
        output_msg->timebase = msg->timebase;
        output_msg->point_num = output_msg->points.size();
        output_msg->lidar_id = msg->lidar_id;
        output_msg->rsvd = msg->rsvd;

        output_lidar_pub_->publish(*output_msg);
    }
}

void LidarMonitorNode::lidar2Callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
{
    lidar_2_cnt_ = 0;
    if(lidar_flag_ == 2) { //只有104雷达正常
        livox_ros_driver2::msg::CustomMsg::SharedPtr output_msg = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
        output_msg->points=msg->points;
        output_msg->header.stamp = msg->header.stamp;
        output_msg->header.frame_id = livox_frame_id_2_;
        output_msg->timebase = msg->timebase;
        output_msg->point_num = output_msg->points.size();
        output_msg->lidar_id = msg->lidar_id;
        output_msg->rsvd = msg->rsvd;

        output_lidar_pub_->publish(*output_msg);
    }
}

void LidarMonitorNode::timer_callback()
{
    lidar_1_cnt_++;
    lidar_2_cnt_++;

    if (lidar_1_cnt_ <= lidar_1_dog_ && lidar_2_cnt_ <= lidar_2_dog_){
        //RCLCPP_INFO(this->get_logger(), "lidar working");
        lidar_flag_ = 0; //两个雷达都正常
    }
    else if (lidar_1_cnt_ <= lidar_1_dog_ && lidar_2_cnt_ > lidar_2_dog_){
        lidar_flag_ = 1;
        print_cnt_++;
        if(print_cnt_ > 60)
            RCLCPP_ERROR(this->get_logger(), "lose lidar 2");
    }
    else if (lidar_1_cnt_ > lidar_1_dog_ && lidar_2_cnt_ <= lidar_2_dog_){
        lidar_flag_ = 2;
        print_cnt_++;
        if(print_cnt_ > 60)
            RCLCPP_ERROR(this->get_logger(), "lose lidar 1");
    }
    else{
        print_cnt_++;
        if(print_cnt_ > 60)
            RCLCPP_ERROR(this->get_logger(), "lose lidar! reinitializing!");
        lidar_flag_ = 3; //两个雷达都不正常 TODO 重定位
    }

    pub_cnt_++;
    if (pub_cnt_ > 25){
        pub_cnt_ = 0;
        std_msgs::msg::Int32::SharedPtr msg = std::make_shared<std_msgs::msg::Int32>();
        msg->data = lidar_flag_;
        lidar_flag_pub_->publish(*msg);
        //RCLCPP_INFO(this->get_logger(), "publishing lidar flag");
    }
}

void LidarMonitorNode::lidar_transformed(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& lidar_in,
                                         livox_ros_driver2::msg::CustomMsg::SharedPtr& lidar_out,
                                         Eigen::Affine3d transformLidarTwo2One)
{
    for (auto& point : lidar_in->points)
    {
        //注释内容都为测试计算时间用
        // auto begin=rclcpp::Clock().now();
        // int32_t begin_second=begin.nanoseconds();

        Eigen::Vector3d pointHomogeneous(point.x, point.y, point.z);
        Eigen::Vector3d transformedPoint = transformLidarTwo2One * pointHomogeneous;
        livox_ros_driver2::msg::CustomPoint point_msg;
        // auto tf=rclcpp::Clock().now();
        // int32_t tf_second=tf.nanoseconds();
        // RCLCPP_INFO(this->get_logger(), "mat use %d",tf_second-begin_second,"s");

        point_msg.x = transformedPoint(0);
        point_msg.y = transformedPoint(1);
        point_msg.z = transformedPoint(2);
        point_msg.reflectivity = point.reflectivity;
        point_msg.tag = point.tag;
        point_msg.line = point.line;
        point_msg.offset_time = point.offset_time;
        // auto filter=rclcpp::Clock().now();
        // int32_t filter_second=filter.nanoseconds();
        // RCLCPP_INFO(this->get_logger(), "give value use %d",filter_second-tf_second,"s");

        lidar_out->points.push_back(point_msg);
        // auto end=rclcpp::Clock().now();
        // int32_t end_second=end.nanoseconds();
        // RCLCPP_INFO(this->get_logger(), "pushback use %d",end_second-filter_second,"s");
    }
}

void LidarMonitorNode::imu1Callback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu)
{
    if(lidar_flag_ == 1){
        output_imu_pub_->publish(*imu);
        imu_print_cnt_++;
        if(imu_print_cnt_ > 200)
            RCLCPP_ERROR(this->get_logger(), "lose lidar imu104");
    }
}

void LidarMonitorNode::imu2Callback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu)
{
    if(lidar_flag_ == 0 || lidar_flag_ == 2){
        output_imu_pub_->publish(*imu);
    }
}