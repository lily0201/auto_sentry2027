#include <costmap_converter/costmap_to_dynamic_obstacles/costmap_to_dynamic_obstacles.h>

#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// 插件宏：把这个类注册成pluginlib插件，Nav2可以动态加载这个类
PLUGINLIB_EXPORT_CLASS(costmap_converter::CostmapToDynamicObstacles, costmap_converter::BaseCostmapToPolygons)

namespace costmap_converter
{

//====小白注释====
// 构造函数：插件被创建的时候执行
CostmapToDynamicObstacles::CostmapToDynamicObstacles() : BaseCostmapToDynamicObstacles()
{
  ego_vel_.x = ego_vel_.y = ego_vel_.z = 0; // 机器人自身速度，初始清零
  costmap_ = nullptr;                       // 代价地图指针，初始为空
//  dynamic_recfg_ = nullptr; // ROS1动态调参组件，ROS2版本直接注释掉不用
}

//====小白注释====
// 析构函数：插件销毁的时候执行，释放资源
CostmapToDynamicObstacles::~CostmapToDynamicObstacles()
{
//  if(dynamic_recfg_ != nullptr)
//    delete dynamic_recfg_;
}

//====小白注释====
// initialize：插件初始化函数，插件加载完成会调用一次
// 作用：订阅话题、读取yaml参数、创建算法对象（背景差分、斑点检测、跟踪器）
void CostmapToDynamicObstacles::initialize(rclcpp::Node::SharedPtr nh)
{
  BaseCostmapToPolygons::initialize(nh); // 调用父类初始化

  costmap_ = nullptr;

  //====小白注释====
  // 订阅odom里程计话题
  // 目的：拿到机器人自己跑多快！后面要抵消机器人移动带来的“虚假物体运动”
  odom_sub_ = nh->create_subscription<nav_msgs::msg::Odometry>(
              odom_topic_,
              rclcpp::SystemDefaultsQoS(),
              std::bind(&CostmapToDynamicObstacles::odomCallback, this, std::placeholders::_1));

  // 参数：是否还要输出墙这类静态障碍物
  nh->get_parameter_or<bool>("publish_static_obstacles", publish_static_obstacles_, publish_static_obstacles_);

  //////////////////////////////////
  // Foreground detection parameters 【背景差分参数：区分动态物体和静态墙】
  BackgroundSubtractor::Params bg_sub_params;

  bg_sub_params.alpha_slow = 0.3;
  nh->get_parameter_or<double>("alpha_slow", bg_sub_params.alpha_slow, bg_sub_params.alpha_slow);

  bg_sub_params.alpha_fast = 0.85;
  nh->get_parameter_or<double>("alpha_fast", bg_sub_params.alpha_fast, bg_sub_params.alpha_fast);

  bg_sub_params.beta = 0.85;
  nh->get_parameter_or<double>("beta", bg_sub_params.beta, bg_sub_params.beta);

  bg_sub_params.min_occupancy_probability = 180;
  nh->get_parameter_or<double>("min_occupancy_probability", bg_sub_params.min_occupancy_probability, bg_sub_params.min_occupancy_probability);

  bg_sub_params.min_sep_between_fast_and_slow_filter = 80;
  nh->get_parameter_or<double>("min_sep_between_slow_and_fast_filter", bg_sub_params.min_sep_between_fast_and_slow_filter, bg_sub_params.min_sep_between_fast_and_slow_filter);

  bg_sub_params.max_occupancy_neighbors = 100;
  nh->get_parameter_or<double>("max_occupancy_neighbors", bg_sub_params.max_occupancy_neighbors, bg_sub_params.max_occupancy_neighbors);

  bg_sub_params.morph_size = 1;
  nh->get_parameter_or<int>("morph_size", bg_sub_params.morph_size, bg_sub_params.morph_size);

  // 创建背景差分算法实例
  bg_sub_ = std::unique_ptr<BackgroundSubtractor>(new BackgroundSubtractor(bg_sub_params));

  ////////////////////////////
  // Blob detection parameters 【斑点检测参数：找图片里一块块白色障碍物】
  BlobDetector::Params blob_det_params;

  blob_det_params.filterByColor = true; // 固定：检测亮白色的斑点
  blob_det_params.blobColor = 255;      // 只提取白色物体
  blob_det_params.thresholdStep = 256;  // 输入已经是二值黑白图，不用再做阈值
  blob_det_params.minThreshold = 127;
  blob_det_params.maxThreshold = 255;
  blob_det_params.minRepeatability = 1;

  blob_det_params.minDistBetweenBlobs = 10;
  nh->get_parameter_or<float>("min_distance_between_blobs", blob_det_params.minDistBetweenBlobs, blob_det_params.minDistBetweenBlobs);

  blob_det_params.filterByArea = true;  // 开启面积过滤：太大太小斑点直接扔掉
  nh->get_parameter_or<bool>("filter_by_area", blob_det_params.filterByArea, blob_det_params.filterByArea);

  blob_det_params.minArea = 3; // 像素数小于3的噪点，忽略
  nh->get_parameter_or<float>("min_area", blob_det_params.minArea, blob_det_params.minArea);

  blob_det_params.maxArea = 300;
  nh->get_parameter_or<float>("max_area", blob_det_params.maxArea, blob_det_params.maxArea);

  blob_det_params.filterByCircularity = true; // 圆度过滤
  nh->get_parameter_or<bool>("filter_by_circularity", blob_det_params.filterByCircularity, blob_det_params.filterByCircularity);

  blob_det_params.minCircularity = 0.2;
  nh->get_parameter_or<float>("min_circularity", blob_det_params.minCircularity, blob_det_params.minCircularity);

  blob_det_params.maxCircularity = 1;
  nh->get_parameter_or<float>("max_circularity", blob_det_params.maxCircularity, blob_det_params.maxCircularity);

  blob_det_params.filterByInertia = true; // 根据物体细长程度过滤
  nh->get_parameter_or<bool>("filter_by_intertia", blob_det_params.filterByInertia, blob_det_params.filterByInertia);

  blob_det_params.minInertiaRatio = 0.2;
  nh->get_parameter_or<float>("min_inertia_ratio", blob_det_params.minInertiaRatio, blob_det_params.minInertiaRatio);

  blob_det_params.maxInertiaRatio = 1;
  nh->get_parameter_or<float>("max_intertia_ratio", blob_det_params.maxInertiaRatio, blob_det_params.maxInertiaRatio);

  blob_det_params.filterByConvexity = false;
  nh->get_parameter_or<bool>("filter_by_convexity", blob_det_params.filterByConvexity, blob_det_params.filterByConvexity);

  blob_det_params.minConvexity = 0;
  nh->get_parameter_or<float>("min_convexity", blob_det_params.minConvexity, blob_det_params.minConvexity);

  blob_det_params.maxConvexity = 1;
  nh->get_parameter_or<float>("max_convexity", blob_det_params.maxConvexity, blob_det_params.maxConvexity);

  // 创建斑点检测器
  blob_det_ = BlobDetector::create(blob_det_params);

  ////////////////////////////////////
  // Tracking parameters 【目标跟踪器参数：匹配前后帧物体，卡尔曼算速度】
  CTracker::Params tracker_params;
  tracker_params.dt = 0.2;  // 卡尔曼滤波时间步长
  nh->get_parameter_or<float>("dt", tracker_params.dt, tracker_params.dt);

  tracker_params.dist_thresh = 60.0; // 两帧物体距离超过这个值，认为不是同一个物体
  nh->get_parameter_or<float>("dist_thresh", tracker_params.dist_thresh, tracker_params.dist_thresh);

  tracker_params.max_allowed_skipped_frames = 3; // 物体最多丢3帧，超过就删除这个目标
  nh->get_parameter_or<int>("max_allowed_skipped_frames", tracker_params.max_allowed_skipped_frames, tracker_params.max_allowed_skipped_frames);

  tracker_params.max_trace_length = 10; // 保存历史轨迹点数量
  nh->get_parameter_or<int>("max_trace_length", tracker_params.max_trace_length, tracker_params.max_trace_length);

  // 创建跟踪器实例
  tracker_ = std::unique_ptr<CTracker>(new CTracker(tracker_params));


  ////////////////////////////////////
  // Static costmap conversion parameters 静态障碍物转换器插件
  std::string static_converter_plugin = "costmap_converter::CostmapToPolygonsDBSMCCH";
  nh->get_parameter_or<std::string>("static_converter_plugin", static_converter_plugin, static_converter_plugin);
  loadStaticCostmapConverterPlugin(static_converter_plugin, nh);


  // ROS1动态调参，ROS2全部注释废弃
//  dynamic_recfg_ = new dynamic_reconfigure::Server<CostmapToDynamicObstaclesConfig>(nh);
//  dynamic_reconfigure::Server<CostmapToDynamicObstaclesConfig>::CallbackType cb = boost::bind(&CostmapToDynamicObstacles::reconfigureCB, this, _1, _2);
//  dynamic_recfg_->setCallback(cb);
}

//====小白注释====
// compute()：最最重要！每一帧代价地图过来就跑这个函数
// 完整流水线全部在这里：背景差分→斑点检测→跟踪→算速度→组装输出消息
void CostmapToDynamicObstacles::compute()
{
  // 如果代价地图图片是空，直接返回啥也不干
  if (costmap_mat_.empty())
    return;

  /////////////////////////// Foreground detection ////////////////////////////////////
  // Dynamic obstacles are separated from static obstacles
  //====小白注释====
  // origin_x origin_y：地图原点换算成图片像素坐标
  int origin_x = round(costmap_->getOriginX() / costmap_->getResolution());
  int origin_y = round(costmap_->getOriginY() / costmap_->getResolution());

  //====小白注释====
  // bg_sub_->apply：背景差分核心函数！
  // 输入：整张代价地图图片 costmap_mat_
  // 输出：fg_mask_ 前景掩码图，白色像素=动态障碍物，黑色=背景
  bg_sub_->apply(costmap_mat_, fg_mask_, origin_x, origin_y);

  // 如果一张动态物体都没有，直接返回，不用输出障碍物
  if (fg_mask_.empty())
    return;

  cv::Mat bg_mat;
  if (publish_static_obstacles_)
  {
    //====小白注释====
    // 原图减去动态物体掩码 = 剩下静态障碍物（墙）的图片
    bg_mat = costmap_mat_ - fg_mask_;
  }


  /////////////////////////////// Blob detection /////////////////////////////////////
  //====小白注释====
  // blob_det_->detect：在动态掩码图上找一块块白色斑点
  // keypoints_：每个斑点的中心点；contours：每个斑点外圈轮廓（像素坐标！OpenCV图片坐标系，左上角原点，y向下！）
  blob_det_->detect(fg_mask_, keypoints_);
  std::vector<std::vector<cv::Point>> contours = blob_det_->getContours();


  ////////////////////////////// Tracking ////////////////////////////////////////////
  //====小白注释====
  // 把检测出来的斑点中心点，转成跟踪器需要的数据格式
  std::vector<Point_t> detected_centers(keypoints_.size());
  for (size_t i = 0; i < keypoints_.size(); i++)
  {
    detected_centers.at(i).x = keypoints_.at(i).pt.x;
    detected_centers.at(i).y = keypoints_.at(i).pt.y; //====BUG注释====！这里y是OpenCV向下的像素y，没有翻转！
    detected_centers.at(i).z = 0;
  }

  //====小白注释====
  // 跟踪器更新：匈牙利算法做物体匹配 + 卡尔曼滤波预测位置速度
  // 输入：当前帧检测到的所有中心点、轮廓；输出：tracks列表，每一条代表一个跟踪中的物体，带唯一ID
  tracker_->Update(detected_centers, contours);


  ///////////////////////////////////// Output ///////////////////////////////////////
  //====小白注释====
  // 创建输出消息容器 ObstacleArrayMsg：一堆障碍物信息装在这里
  ObstacleArrayPtr obstacles(new costmap_converter_msgs::msg::ObstacleArrayMsg);
  obstacles->header.stamp = now();
  obstacles->header.frame_id = "/map"; // 输出坐标全部是map全局坐标系

  //====小白注释====
  // 循环每一个被跟踪到的动态物体
  for (unsigned int i = 0; i < (unsigned int)tracker_->tracks.size(); ++i)
  {
    geometry_msgs::msg::Polygon polygon;

    std::vector<Point_t> contour;
    //====小白注释====
    // getContour：把OpenCV像素轮廓 → map世界米制坐标
    //====BUG重点！函数内部没有翻转y轴，轮廓上下颠倒！====
    getContour(i, contour);

    //====小白注释====
    // 把轮廓点填充进多边形消息
    for (const Point_t& pt : contour)
    {
      polygon.points.emplace_back();
      polygon.points.back().x = pt.x;
      polygon.points.back().y = pt.y;
      polygon.points.back().z = 0;
    }

    obstacles->obstacles.emplace_back();
    obstacles->obstacles.back().polygon = polygon;

    obstacles->obstacles.back().id = tracker_->tracks.at(i)->track_id; // 物体跟踪ID

    //====小白注释====
    // 根据物体速度向量，计算物体朝向yaw角
    geometry_msgs::msg::QuaternionStamped orientation;
    Point_t vel = getEstimatedVelocityOfObject(i);
    double yaw = std::atan2(vel.y, vel.x);
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    obstacles->obstacles.back().orientation = tf2::toMsg(q);

    //====小白注释====
    // 填充障碍物速度（map坐标系 m/s）
    geometry_msgs::msg::TwistWithCovariance velocities;
    velocities.twist.linear.x = vel.x;
    velocities.twist.linear.y = vel.y; //====BUG注释====y方向因为图像坐标系问题符号错误
    velocities.twist.linear.z = 0;
    velocities.twist.angular.x = 0;
    velocities.twist.angular.y = 0;
    velocities.twist.angular.z = 0;

    // 协方差直接写单位矩阵，不是真实计算出来的
    velocities.covariance = {1, 0, 0, 0, 0, 0,
                             0, 1, 0, 0, 0, 0,
                             0, 0, 1, 0, 0, 0,
                             0, 0, 0, 1, 0, 0,
                             0, 0, 0, 0, 1, 0,
                             0, 0, 0, 0, 0, 1};

    obstacles->obstacles.back().velocities = velocities;
  }

  ////////////////////////// Static obstacles ////////////////////////////
  //====小白注释====
  // 如果开启publish_static_obstacles，把墙等静态障碍物也塞进输出消息，id=-1代表静态
  if (publish_static_obstacles_)
  {
    uchar* img_data = bg_mat.data;
    int width = bg_mat.cols;
    int height = bg_mat.rows;
    int stride = bg_mat.step;

    if (stackedCostmapConversion())
    {
      // 模式1：重建一份Costmap2D，调用静态插件把静态障碍物转为多边形
      std::shared_ptr<nav2_costmap_2d::Costmap2D> static_costmap(new nav2_costmap_2d::Costmap2D(costmap_->getSizeInCellsX(),
                                                                                        costmap_->getSizeInCellsY(),
                                                                                        costmap_->getResolution(),
                                                                                        costmap_->getOriginX(),
                                                                                        costmap_->getOriginY()));
      for(int i = 0; i < height; i++)
      {
        for(int j = 0; j < width; j++)
        {
          static_costmap->setCost(j, i, img_data[i * stride + j]);
        }
      }

      setStaticCostmap(static_costmap);
      convertStaticObstacles();

      auto static_polygons = getStaticPolygons();
      for (auto it = static_polygons->begin(); it != static_polygons->end(); ++it)
      {
        obstacles->obstacles.emplace_back();
        obstacles->obstacles.back().polygon = *it;
        obstacles->obstacles.back().velocities.twist.linear.x = 0;
        obstacles->obstacles.back().velocities.twist.linear.y = 0;
        obstacles->obstacles.back().id = -1; // id=-1表示静态障碍物
      }
    }
    else
    {
      // 模式2：不转多边形，静态障碍物直接用单点表示
      for(int i = 0; i < height; i++)
      {
        for(int j = 0; j < width; j++)
        {
            uchar value = img_data[i * stride + j];
            if (value > 0)
            {
              obstacles->obstacles.emplace_back();
              geometry_msgs::msg::Point32 pt;
              pt.x = (double)j*costmap_->getResolution() + costmap_->getOriginX();
              pt.y = (double)i*costmap_->getResolution() + costmap_->getOriginY();
              obstacles->obstacles.back().polygon.points.push_back(pt);
              obstacles->obstacles.back().velocities.twist.linear.x = 0;
              obstacles->obstacles.back().velocities.twist.linear.y = 0;
              obstacles->obstacles.back().id = -1;
            }
        }
      }
    }
  }

  //====小白注释====
  // 加锁保存最终障碍物消息，外部函数getObstacles()读取结果
  updateObstacleContainer(obstacles);
}

//====小白注释====
// setCostmap2D：Nav2把代价地图指针传给插件
void CostmapToDynamicObstacles::setCostmap2D(nav2_costmap_2d::Costmap2D* costmap)
{
  if (!costmap)
    return;

  costmap_ = costmap;

  updateCostmap2D();
}

//====小白注释====
// updateCostmap2D：把Nav2代价地图原始char数组包装成OpenCV Mat图片
// ⚠️这里是浅拷贝！直接引用costmap内存，没有复制一份新内存
void CostmapToDynamicObstacles::updateCostmap2D()
{
  if (!costmap_->getMutex())
  {
    RCLCPP_ERROR(getLogger(), "Cannot update costmap since the mutex pointer is null");
    return;
  }
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());

  // 把代价地图原始内存包装成cv::Mat，浅拷贝！
  costmap_mat_ = cv::Mat(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY(), CV_8UC1,
                        costmap_->getCharMap());
}

//====小白注释====
// getObstacles：对外接口，获取算好的障碍物数组
ObstacleArrayConstPtr CostmapToDynamicObstacles::getObstacles()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return obstacles_;
}

//====小白注释====
// updateObstacleContainer：线程安全保存障碍物结果
void CostmapToDynamicObstacles::updateObstacleContainer(ObstacleArrayPtr obstacles)
{
  std::lock_guard<std::mutex> lock(mutex_);
  obstacles_ = obstacles;
}

//====小白注释====
// getEstimatedVelocityOfObject：计算障碍物真实世界速度
// 公式：真实速度 =（跟踪器输出像素速度 × 地图分辨率） + 机器人自身map坐标系速度 ego_vel_
// 加ego_vel_是为了抵消机器人自己移动造成的虚假运动
Point_t CostmapToDynamicObstacles::getEstimatedVelocityOfObject(unsigned int idx)
{
  Point_t vel = tracker_->tracks.at(idx)->getEstimatedVelocity() * costmap_->getResolution() + ego_vel_;
  return vel;
}

//====小白注释====
// odomCallback：里程计回调函数
// 接收odom消息：机器人本体坐标系下的速度，旋转转换到map全局坐标系，存到ego_vel_
void CostmapToDynamicObstacles::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  RCLCPP_INFO_ONCE(getLogger(), "CostmapToDynamicObstacles: odom received.");

  tf2::Quaternion pose;
  tf2::fromMsg(msg->pose.pose.orientation, pose);

  tf2::Vector3 twistLinear;
  twistLinear.setX(msg->twist.twist.linear.x);
  twistLinear.setY(msg->twist.twist.linear.y);
  twistLinear.setZ(msg->twist.twist.linear.z);

  //====小白注释====
  // quatRotate：把机器人本体坐标系速度，旋转到map全局坐标系
  tf2::Vector3 vel = tf2::quatRotate(pose, twistLinear);
  ego_vel_.x = vel.x();
  ego_vel_.y = vel.y();
  ego_vel_.z = vel.z();
}

//====小白注释====
// getContour：把OpenCV像素轮廓点转为map世界米坐标
//====重大BUG！没有翻转y！OpenCV图像y向下，map坐标系y向上，这里没有做y翻转====
void CostmapToDynamicObstacles::getContour(unsigned int idx, std::vector<Point_t>& contour)
{
  assert(!tracker_->tracks.empty() && idx < tracker_->tracks.size());

  contour.clear();

  std::vector<cv::Point> contour2i = tracker_->tracks.at(idx)->getLastContour();

  contour.reserve(contour2i.size());

  Point_t costmap_origin(costmap_->getOriginX(), costmap_->getOriginY(), 0);

  for (std::size_t i = 0; i < contour2i.size(); ++i)
  {
    contour.push_back((Point_t(contour2i.at(i).x, contour2i.at(i).y, 0.0)*costmap_->getResolution())
                        + costmap_origin); // Shift to /map
  }

}

//====小白注释====
// visualize调试函数：弹出OpenCV本地窗口显示图片
// ⚠️内部做了cv::flip翻转图片给人看！但是程序内部运算的数据没有翻转！
void CostmapToDynamicObstacles::visualize(const std::string& name, const cv::Mat& image)
{
  if (!image.empty())
  {
    cv::Mat im = image.clone();
    cv::flip(im, im, 0); // 这里翻转只是给弹窗看，算法逻辑不用这份翻转后的图！
    cv::imshow(name, im);
    cv::waitKey(1);
  }
}

}
