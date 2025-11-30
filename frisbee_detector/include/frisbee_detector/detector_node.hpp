#pragma once

#include <memory>
#include <optional>
#include <string>
#include <filesystem>
#include <unordered_set>

#include "frisbee_detector/detector_core.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"

namespace frisbee_detector
{

class FrisbeeDetectorNode : public rclcpp::Node
{
public:
  FrisbeeDetectorNode();

private:
  void DeclareParametersFromYaml();
  void ApplyParameterValues();
  std::optional<std::filesystem::path> ResolveParametersFilePath() const;
  void SyncCoreConfig();

  rcl_interfaces::msg::SetParametersResult ParametersCallback(
    const std::vector<rclcpp::Parameter> & params);
  void PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  std::string cloud_topic_;

  DetectorConfig config_{};
  FrisbeeDetectorCore core_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detection_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr position_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  bool last_detection_{false};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace frisbee_detector
