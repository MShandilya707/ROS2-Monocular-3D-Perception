#include "depth_to_pointcloud/depth_to_pointcloud_node.hpp"

#include <cmath>
#include <vector>

#include "rclcpp_components/register_node_macro.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <cv_bridge/cv_bridge.h>

// PCL Includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace depth_to_pointcloud
{

DepthToPointCloudNode::DepthToPointCloudNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("depth_to_pointcloud", options)
{
  // We declare ROS 2 parameters here in the constructor.
  // These parameters can be modified dynamically at runtime.
  this->declare_parameter("voxel_leaf_size", 0.05); // 5cm resolution for downsampling
  this->declare_parameter("sor_k_neighbors", 50);   // Number of neighbors for SOR
  this->declare_parameter("sor_stddev_mult", 1.0);  // Standard deviation multiplier

  RCLCPP_INFO(this->get_logger(), "DepthToPointCloudNode created (Unconfigured state).");
}

DepthToPointCloudNode::~DepthToPointCloudNode() = default;

// --- Lifecycle State Machine ---

// 1. on_configure: 
// Called when the node transitions from Unconfigured to Inactive.
// We allocate resources here, set up subscribers, and prepare publishers, but we DO NOT start 
// processing data or emitting messages yet.
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DepthToPointCloudNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Configuring...");

  // Initialize the LifecyclePublisher.
  pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);

  // Initialize the message filters subscribers.
  // We use rmw_qos_profile_sensor_data as the QoS profile for sensors like cameras.
  // 'this' refers to the rclcpp::Node base class, but message_filters needs a standard node interface.
  // rclcpp_lifecycle provides a get_node_base_interface() to satisfy this.
  depth_sub_.subscribe(this, "depth_image", rmw_qos_profile_sensor_data);
  info_sub_.subscribe(this, "camera_info", rmw_qos_profile_sensor_data);
  image_sub_.subscribe(this, "image_raw", rmw_qos_profile_sensor_data);

  // Initialize the synchronizer with our exact time policy.
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), depth_sub_, info_sub_, image_sub_);
  
  // Register the callback to be fired when a synchronized pair arrives.
  sync_->registerCallback(
    std::bind(&DepthToPointCloudNode::on_depth_image_received, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
  );

  RCLCPP_INFO(this->get_logger(), "Configured successfully. Transitioning to Inactive.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// 2. on_activate:
// Called when transitioning from Inactive to Active.
// This is where the node actually "turns on". Publishers are activated so they can send data.
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DepthToPointCloudNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Activating...");
  pc_pub_->on_activate();
  RCLCPP_INFO(this->get_logger(), "Activated successfully.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// 3. on_deactivate:
// Called when transitioning back to Inactive.
// We stop emitting data. The publishers are deactivated.
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DepthToPointCloudNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating...");
  pc_pub_->on_deactivate();
  RCLCPP_INFO(this->get_logger(), "Deactivated successfully.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// 4. on_cleanup:
// Called when transitioning back to Unconfigured. Free resources.
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DepthToPointCloudNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up...");
  pc_pub_.reset();
  sync_.reset();
  depth_sub_.unsubscribe();
  info_sub_.unsubscribe();
  image_sub_.unsubscribe();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// 5. on_shutdown:
// Called when shutting down the node entirely.
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DepthToPointCloudNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


// --- Core Computation Callback ---

void DepthToPointCloudNode::on_depth_image_received(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg)
{
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  if (depth_msg->encoding != sensor_msgs::image_encodings::TYPE_16UC1 &&
      depth_msg->encoding != sensor_msgs::image_encodings::TYPE_32FC1) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Unsupported depth image encoding: %s", depth_msg->encoding.c_str());
    return;
  }

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  double fx = info_msg->k[0];
  double cx = info_msg->k[2];
  double fy = info_msg->k[4];
  double cy = info_msg->k[5];

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  cloud->header.frame_id = depth_msg->header.frame_id;
  cloud->header.stamp = (depth_msg->header.stamp.sec * 1000000ull) + (depth_msg->header.stamp.nanosec / 1000);
  
  cloud->height = depth_msg->height;
  cloud->width = depth_msg->width;
  cloud->is_dense = false;
  cloud->points.reserve(depth_msg->height * depth_msg->width);

  bool is_float = (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1);
  const uint8_t* data_ptr = depth_msg->data.data();

  for (uint32_t v = 0; v < depth_msg->height; ++v) {
    for (uint32_t u = 0; u < depth_msg->width; ++u) {
      float depth;
      if (is_float) {
        depth = *reinterpret_cast<const float*>(&data_ptr[v * depth_msg->step + u * sizeof(float)]);
      } else {
        uint16_t depth_mm = *reinterpret_cast<const uint16_t*>(&data_ptr[v * depth_msg->step + u * sizeof(uint16_t)]);
        depth = static_cast<float>(depth_mm) * 0.001f;
      }

      if (std::isnan(depth) || std::isinf(depth) || depth <= 0.0f) {
        continue;
      }

      pcl::PointXYZRGB pt;
      pt.x = (static_cast<float>(u) - cx) * depth / fx;
      pt.y = (static_cast<float>(v) - cy) * depth / fy;
      pt.z = depth;

      cv::Vec3b color = cv_ptr->image.at<cv::Vec3b>(v, u);
      pt.b = color[0];
      pt.g = color[1];
      pt.r = color[2];

      cloud->points.push_back(pt);
    }
  }

  cloud->width = cloud->points.size();
  cloud->height = 1;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
  voxel_filter.setInputCloud(cloud);
  double leaf_size = this->get_parameter("voxel_leaf_size").as_double();
  voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_filter.filter(*cloud_downsampled);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud(cloud_downsampled);
  sor.setMeanK(this->get_parameter("sor_k_neighbors").as_int());
  sor.setStddevMulThresh(this->get_parameter("sor_stddev_mult").as_double());
  sor.filter(*cloud_filtered);

  sensor_msgs::msg::PointCloud2 output_msg;
  pcl::toROSMsg(*cloud_filtered, output_msg);
  output_msg.header = depth_msg->header;

  pc_pub_->publish(output_msg);
}

}  // namespace depth_to_pointcloud

// Register the component with class_loader.
// This allows the node to be dynamically loaded into a component container at runtime,
// enabling zero-copy transport when other nodes are in the same container.
RCLCPP_COMPONENTS_REGISTER_NODE(depth_to_pointcloud::DepthToPointCloudNode)
