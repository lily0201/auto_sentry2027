#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <iostream>
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#define MAT_FROM_ARRAY(v)        v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8],v[9],v[10],v[11],v[12],v[13],v[14],v[15]

/**
 * @brief 雷达监测节点，用于监控两个激光雷达和IMU数据状态
 *
 * 该节点实现以下功能：
 * - 订阅两个激光雷达的点云数据和IMU数据
 * - 实现雷达看门狗机制，检测雷达数据是否超时
 * - 在主雷达故障时自动切换到副雷达
 * - 进行坐标系转换，将副雷达数据转换到主雷达坐标系
 * - 发布雷达状态标志和处理后的点云/IMU数据
 */
class LidarMonitorNode: public rclcpp::Node{
public:
    /**
     * @brief 构造函数，初始化雷达监测节点
     * @param node_name 节点名称
     */
    explicit LidarMonitorNode(std::string node_name);

private:
    /** @brief 雷达187点云订阅器 */
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_1_sub_;
    /** @brief 雷达104(主雷达)点云订阅器 */
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_2_sub_;
    /** @brief IMU 1数据订阅器 */
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_1_sub_;
    /** @brief IMU 2数据订阅器 */
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_2_sub_;
    /** @brief 雷达状态标志发布器 */
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr lidar_flag_pub_;
    /** @brief 处理后的点云数据发布器 */
    rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr output_lidar_pub_;
    /** @brief 处理后的IMU数据发布器 */
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr output_imu_pub_;
    /** @brief TF变换监听器 */
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    /** @brief TF变换缓冲区 */
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    /** @brief 雷达状态标志 (0:双雷达正常, 1:仅雷达1正常, 2:仅雷达2正常, 3:双雷达故障) */
    int lidar_flag_;
    /** @brief 雷达1看门狗阈值 */
    int lidar_1_dog_;
    /** @brief 雷达2看门狗阈值 */
    int lidar_2_dog_;
    /** @brief 雷达1计数器 (超过阈值表示数据超时) */
    int lidar_1_cnt_;
    /** @brief 雷达2计数器 (超过阈值表示数据超时) */
    int lidar_2_cnt_;
    /** @brief 定时器频率 (Hz) */
    double frequency;
    /** @brief 发布计数器 */
    int pub_cnt_;
    /** @brief 打印计数器 */
    int print_cnt_;
    /** @brief IMU打印计数器 */
    int imu_print_cnt_;
    /** @brief 副雷达到主雷达的坐标变换矩阵 */
    Eigen::Matrix4f transformLidarTwo2One = Eigen::Matrix4f::Identity();
    /** @brief 坐标变换矩阵参数 */
    std::vector<double> LidarTwo2OneMaxtrix;

    /** @brief 雷达1点云话题名称 */
    std::string lidar_topic_1_;
    /** @brief 雷达2点云话题名称 */
    std::string lidar_topic_2_;
    /** @brief IMU 1话题名称 */
    std::string imu_topic_1_;
    /** @brief IMU 2话题名称 */
    std::string imu_topic_2_;
    /** @brief 雷达1坐标系ID */
    std::string livox_frame_id_1_;
    /** @brief 雷达2坐标系ID */
    std::string livox_frame_id_2_;
    /** @brief 输出点云话题名称 */
    std::string output_lidar_topic_;
    /** @brief 输出IMU话题名称 */
    std::string output_imu_topic_;

    /** @brief 主定时器 */
    rclcpp::TimerBase::SharedPtr timer_;
    /** @brief 定时器周期 (毫秒) */
    std::chrono::milliseconds period_ns;

    /**
     * @brief 雷达187点云数据回调函数
     * @param msg 雷达点云消息指针
     * @note 重置雷达1看门狗计数器，当主雷达故障时转换并发布点云数据
     */
    void lidar1Callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);

    /**
     * @brief 雷达104(主雷达)点云数据回调函数
     * @param msg 雷达点云消息指针
     * @note 重置雷达2看门狗计数器，当主雷达正常时直接发布点云数据
     */
    void lidar2Callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);

    /**
     * @brief 定时器回调函数，实现雷达看门狗功能
     * @note 定期检查雷达数据接收状态，更新雷达标志位并发布状态信息
     */
    void timer_callback();

    /**
     * @brief 点云坐标变换函数
     * @param lidar_in 输入点云消息
     * @param lidar_out 输出点云消息
     * @param transformLidarTwo2One 坐标变换矩阵
     * @note 将副雷达点云数据转换到主雷达坐标系
     */
    void lidar_transformed(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &lidar_in,
                          livox_ros_driver2::msg::CustomMsg::SharedPtr &lidar_out,
                          Eigen::Affine3d transformLidarTwo2One);

    /**
     * @brief IMU 1数据回调函数
     * @param imu IMU消息指针
     * @note 当使用雷达1时发布IMU数据
     */
    void imu1Callback(const sensor_msgs::msg::Imu::ConstSharedPtr &imu);

    /**
     * @brief IMU 2数据回调函数
     * @param imu IMU消息指针
     * @note 当使用雷达2或双雷达正常时发布IMU数据
     */
    void imu2Callback(const sensor_msgs::msg::Imu::ConstSharedPtr &imu);
};