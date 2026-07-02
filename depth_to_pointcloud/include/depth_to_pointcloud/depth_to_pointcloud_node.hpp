#ifndef DEPTH_TO_POINTCLOUD__DEPTH_TO_POINTCLOUD_NODE_HPP_
#define DEPTH_TO_POINTCLOUD__DEPTH_TO_POINTCLOUD_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// We use message_filters to synchronize the incoming depth image and camera info.
// Both messages need to have the same timestamp for accurate projection.
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"

namespace depth_to_pointcloud
{

/**
 * @brief Lifecycle Node that converts a 2D depth image into a 3D PointCloud2.
 * 
 * We inherit from `rclcpp_lifecycle::LifecycleNode`. This allows the node's state
 * to be managed externally (e.g., Unconfigured -> Inactive -> Active).
 * This is crucial for deterministic startup in complex robotic systems.
 */
class DepthToPointCloudNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief Constructor. We pass options to enable intra-process communication by default,
   * which is a requirement for the zero-copy transport mentioned in the brief.
   */
  explicit DepthToPointCloudNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~DepthToPointCloudNode() override;

  // --- Lifecycle Transition Callbacks ---
  // These must be implemented to handle state changes.

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  /**
   * @brief The core processing callback. This is called only when both an Image and
   * its corresponding CameraInfo arrive with the exact same timestamp.
   * 
   * @param image_msg The raw or rectified depth image.
   * @param info_msg The camera intrinsics matrix.
   */
  void on_depth_image_received(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg);

  // Lifecycle publisher for the output point cloud
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>> pc_pub_;

  // Message filter subscribers
  // Note: we use message_filters::Subscriber rather than standard rclcpp::Subscription
  // because message_filters can hold messages and pass them to a Synchronizer.
  message_filters::Subscriber<sensor_msgs::msg::Image, rclcpp_lifecycle::LifecycleNode> depth_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo, rclcpp_lifecycle::LifecycleNode> info_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image, rclcpp_lifecycle::LifecycleNode> image_sub_;

  // Synchronizer policy: ExactTime ensures we only process triplets with identical timestamps.
  using SyncPolicy = message_filters::sync_policies::ExactTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::Image>;
  
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

}  // namespace depth_to_pointcloud

#endif  // DEPTH_TO_POINTCLOUD__DEPTH_TO_POINTCLOUD_NODE_HPP_
