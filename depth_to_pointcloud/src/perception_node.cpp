#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "depth_to_pointcloud/depth_to_pointcloud_node.hpp"

int main(int argc, char ** argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // We set the NodeOptions to enable intra-process communication.
  // When this node is run as a standalone executable, it won't benefit as much from zero-copy
  // as it would in a component container with other nodes, but it's good practice to enable it.
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  // Instantiate our lifecycle node component
  auto node = std::make_shared<depth_to_pointcloud::DepthToPointCloudNode>(options);

  // Use a SingleThreadedExecutor to spin the node.
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());

  // Spin until shutdown is requested
  exec.spin();

  // Shutdown ROS 2
  rclcpp::shutdown();

  return 0;
}
