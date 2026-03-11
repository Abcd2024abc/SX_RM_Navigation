#include "small_gicp_relocalization/small_gicp_relocalization.hpp"

#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace small_gicp_relocalization
{

  SmallGicpRelocalizationNode::SmallGicpRelocalizationNode(const rclcpp::NodeOptions &options)
      : Node("small_gicp_relocalization", options),
        result_t_(Eigen::Isometry3d::Identity()),
        previous_result_t_(Eigen::Isometry3d::Identity())
  {
    this->declare_parameter("num_threads", 4);//The number of threads used for parallel calculations. Influences the performance of point cloud processing (sampling and covariance estimation below).
    this->declare_parameter("num_neighbors", 20);//在协方差估计中使用的邻居点数量。较大的值会增加计算时间，但可能提高精度。
    this->declare_parameter("global_leaf_size", 0.25);//Voxel grid downsampling dimensions (units: meters) of global map point cloud. Larger values ​​will reduce point cloud density and increase processing speed, but may reduce accuracy.
    this->declare_parameter("registered_leaf_size", 0.25);//注册点云（当前扫描点云）的体素网格下采样尺寸（单位：米）。与 `global_leaf_size` 类似，但用于当前扫描点云。
    this->declare_parameter("max_dist_sq", 1.0);//最大距离平方值，用于剔除点云匹配中的离群点。较小的值会严格限制匹配点对的距离。
    this->declare_parameter("map_frame", "map");//地图坐标系的名称。通常是全局参考坐标系。
    this->declare_parameter("odom_frame", "odom");//The name of the odometer coordinate system. It is usually a local reference coordinate system that represents the motion of the robot relative to its starting position.
    this->declare_parameter("base_frame", "");//机器人底盘坐标系的名称。机器人主体的参考点。
    this->declare_parameter("robot_base_frame", "");//机器人底盘的具体坐标系名称，用于与其他坐标系（如地图或里程计）进行转换。
    this->declare_parameter("lidar_frame", "");// 激光雷达坐标系的名称。用于表示激光雷达数据的参考点。
    this->declare_parameter("prior_pcd_file", "");// 全局地图的点云文件路径。用于加载全局地图以进行点云匹配。


    this->get_parameter("num_threads", num_threads_);
    this->get_parameter("num_neighbors", num_neighbors_);
    this->get_parameter("global_leaf_size", global_leaf_size_);
    this->get_parameter("registered_leaf_size", registered_leaf_size_);
    this->get_parameter("max_dist_sq", max_dist_sq_);
    this->get_parameter("map_frame", map_frame_);
    this->get_parameter("odom_frame", odom_frame_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("robot_base_frame", robot_base_frame_);
    this->get_parameter("lidar_frame", lidar_frame_);
    this->get_parameter("prior_pcd_file", prior_pcd_file_);
    accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    register_ = std::make_shared<
        small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    loadGlobalMap(prior_pcd_file_);

    // Downsample points and convert them into pcl::PointCloud<pcl::PointCovariance>
    target_ = small_gicp::voxelgrid_sampling_omp<
        pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
        *global_map_, global_leaf_size_);

    // Estimate covariances of points
    small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

    // Create KdTree for target
    target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
        target_, small_gicp::KdTreeBuilderOMP(num_threads_));

    pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "registered_scan", 10,
        std::bind(&SmallGicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initialpose", 10,
        std::bind(&SmallGicpRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));

    register_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), // 2 Hz
        std::bind(&SmallGicpRelocalizationNode::performRegistration, this));

    transform_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), // 20 Hz
        std::bind(&SmallGicpRelocalizationNode::publishTransform, this));
  }

void SmallGicpRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded global map with %zu points", global_map_->points.size());

  // NOTE: Transform global pcd_map (based on `lidar_odom` frame) to the `odom` frame
  Eigen::Affine3d odom_to_lidar_odom;
  while (true) {
    try {
      auto tf_stamped = tf_buffer_->lookupTransform(
        base_frame_, lidar_frame_, this->now(), rclcpp::Duration::from_seconds(1.0));
      odom_to_lidar_odom = tf2::transformToEigen(tf_stamped.transform);
      RCLCPP_INFO_STREAM(
        this->get_logger(), "odom_to_lidar_odom: translation = "
                              << odom_to_lidar_odom.translation().transpose() << ", rpy = "
                              << odom_to_lidar_odom.rotation().eulerAngles(0, 1, 2).transpose());
      break;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s Retrying...", ex.what());
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }
  pcl::transformPointCloud(*global_map_, *global_map_, odom_to_lidar_odom);
}

void SmallGicpRelocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  last_scan_time_ = msg->header.stamp;
  current_scan_frame_id_ = msg->header.frame_id;

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);
  *accumulated_cloud_ += *scan;
}

void SmallGicpRelocalizationNode::performRegistration()
{
  if (accumulated_cloud_->empty()) {
    RCLCPP_WARN(this->get_logger(), "No accumulated points to process.");
    return;
  }

  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *accumulated_cloud_, registered_leaf_size_);

  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    source_, small_gicp::KdTreeBuilderOMP(num_threads_));

  if (!source_ || !source_tree_) {
    return;
  }

  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;
  register_->optimizer.max_iterations = 10;

  auto result = register_->align(*target_, *source_, *target_tree_, previous_result_t_);

  if (result.converged) {
    result_t_ = previous_result_t_ = result.T_target_source;
  } else {
    RCLCPP_WARN(this->get_logger(), "GICP did not converge.");
  }

  accumulated_cloud_->clear();
}

void SmallGicpRelocalizationNode::publishTransform()
{
  // 如果结果矩阵为空（未初始化或无效），直接返回
  if (result_t_.matrix().isZero()) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;

  // Set the timestamp to the last scan time + 0.1 second (future time) to ensure time synchronization of TFs
  transform_stamped.header.stamp = last_scan_time_ + rclcpp::Duration::from_seconds(0.1);
  transform_stamped.header.frame_id = map_frame_;  // 设置父坐标系为地图坐标系
  transform_stamped.child_frame_id = odom_frame_; // 设置子坐标系为里程计坐标系

  // 从结果矩阵中提取平移和旋转信息
  const Eigen::Vector3d translation = result_t_.translation(); // 提取平移向量
  const Eigen::Quaterniond rotation(result_t_.rotation());     // 提取旋转四元数

  // 填充 TransformStamped 消息的平移部分
  transform_stamped.transform.translation.x = translation.x();
  transform_stamped.transform.translation.y = translation.y();
  transform_stamped.transform.translation.z = translation.z();

  // 填充 TransformStamped 消息的旋转部分
  transform_stamped.transform.rotation.x = rotation.x();
  transform_stamped.transform.rotation.y = rotation.y();
  transform_stamped.transform.rotation.z = rotation.z();
  transform_stamped.transform.rotation.w = rotation.w();

  // 通过 TF 广播器发布变换
  tf_broadcaster_->sendTransform(transform_stamped);
}

void SmallGicpRelocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // 打印接收到的初始位姿信息
  RCLCPP_INFO(
    this->get_logger(), "Received initial pose: [x: %f, y: %f, z: %f]", msg->pose.pose.position.x,
    msg->pose.pose.position.y, msg->pose.pose.position.z);

  // Create a transformation matrix from map to robot chassis
  Eigen::Isometry3d map_to_robot_base = Eigen::Isometry3d::Identity();
  map_to_robot_base.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y,
    msg->pose.pose.position.z; // 设置平移部分
  map_to_robot_base.linear() = Eigen::Quaterniond(
                                 msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                 msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
                                 .toRotationMatrix(); // 设置旋转部分

  try {
    // 查找从机器人底盘到当前扫描帧的变换
    auto transform =
      tf_buffer_->lookupTransform(robot_base_frame_, current_scan_frame_id_, tf2::TimePointZero);

    // 将 TF 变换转换为 Eigen 格式
    Eigen::Isometry3d robot_base_to_odom = tf2::transformToEigen(transform.transform);

    // Calculate the transformation from map to odometer
    Eigen::Isometry3d map_to_odom = map_to_robot_base * robot_base_to_odom;

    // Update current and previous results transformations
    previous_result_t_ = result_t_ = map_to_odom;
  } catch (tf2::TransformException & ex) {
    // 如果变换查找失败，打印警告信息
    RCLCPP_WARN(
      this->get_logger(), "Could not transform initial pose from %s to %s: %s",
      robot_base_frame_.c_str(), current_scan_frame_id_.c_str(), ex.what());
  }
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
