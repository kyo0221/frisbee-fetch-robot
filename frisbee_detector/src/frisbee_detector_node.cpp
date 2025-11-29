#include "frisbee_detector/detector_node.hpp"

#include <filesystem>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/parameter_map.hpp"
#include "rclcpp/qos.hpp"

namespace frisbee_detector
{
namespace
{
constexpr std::array<const char *, 22> kRequiredParameterKeys = {
  "cloud_topic",
  "min_range",
  "max_range",
  "min_height",
  "max_height",
  "min_azimuth",
  "max_azimuth",
  "min_cluster_points",
  "max_cluster_points",
  "min_cluster_width",
  "max_cluster_width",
  "max_angle_gap",
  "max_range_jump",
  "max_range_std",
  "max_vertical_span",
  "max_cluster_radius",
  "position_smoothing_alpha",
  "max_position_step",
  "max_tracking_velocity",
  "max_vertical_step",
  "max_vertical_velocity",
  "verbose_logging"
};

std::vector<rclcpp::Parameter> LoadParametersForNode(
  const std::filesystem::path & file_path,
  const std::string & node_name,
  const std::string & node_fqn)
{
  const auto parameter_map = rclcpp::parameter_map_from_yaml_file(file_path.string());
  const std::array<std::string, 3> search_order = {node_fqn, node_name, "/**"};

  for (const auto & key : search_order) {
    auto iter = parameter_map.find(key);
    if (iter != parameter_map.end() && !iter->second.empty()) {
      return iter->second;
    }
  }

  throw std::runtime_error(
          "No parameters found for node '" + node_name + "' in '" + file_path.string() + "'.");
}

bool IsRequiredKey(const std::string & key)
{
  return std::any_of(
    kRequiredParameterKeys.begin(), kRequiredParameterKeys.end(),
    [&](const char * required) {return key == required;});
}
}  // namespace

FrisbeeDetectorNode::FrisbeeDetectorNode()
: Node("frisbee_detector"),
  core_(config_)
{
  DeclareParametersFromYaml();
  ApplyParameterValues();

  const auto qos = rclcpp::SensorDataQoS();
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, qos,
    std::bind(&FrisbeeDetectorNode::PointCloudCallback, this, std::placeholders::_1));
  detection_pub_ = create_publisher<std_msgs::msg::Bool>("frisbee_detected", 10);
  position_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("frisbee_position", 10);

  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&FrisbeeDetectorNode::ParametersCallback, this, std::placeholders::_1));

  last_update_time_ = now();

  RCLCPP_INFO(get_logger(), "Frisbee detector listening on '%s'.", cloud_topic_.c_str());
}

void FrisbeeDetectorNode::DeclareParametersFromYaml()
{
  const auto parameter_file_path = ResolveParametersFilePath();
  if (!parameter_file_path.has_value()) {
    throw std::runtime_error("Parameter file could not be resolved.");
  }

  const auto parameters = LoadParametersForNode(
    parameter_file_path.value(), get_name(), get_fully_qualified_name());

  std::unordered_set<std::string> declared;
  declared.reserve(parameters.size());
  for (const auto & parameter : parameters) {
    this->declare_parameter(parameter.get_name(), parameter.get_parameter_value());
    declared.insert(parameter.get_name());
  }

  for (const auto & key : kRequiredParameterKeys) {
    if (declared.count(key) == 0U) {
      throw std::runtime_error("Parameter '" + std::string(key) + "' missing in configuration.");
    }
  }
}

std::optional<std::filesystem::path> FrisbeeDetectorNode::ResolveParametersFilePath() const
{
  try {
    auto share_dir = ament_index_cpp::get_package_share_directory("frisbee_detector");
    std::filesystem::path path = share_dir;
    path /= "config/frisbee_detector.yaml";
    if (std::filesystem::exists(path)) {
      return path;
    }
    RCLCPP_ERROR(
      get_logger(),
      "Parameter file not found at '%s'.",
      path.string().c_str());
    return std::nullopt;
  } catch (const ament_index_cpp::PackageNotFoundError & exc) {
    RCLCPP_ERROR(
      get_logger(),
      "Could not locate package share directory for 'frisbee_detector': %s",
      exc.what());
    return std::nullopt;
  }
}

void FrisbeeDetectorNode::ApplyParameterValues()
{
  cloud_topic_ = this->get_parameter("cloud_topic").as_string();
  config_.min_range = this->get_parameter("min_range").as_double();
  config_.max_range = this->get_parameter("max_range").as_double();
  config_.min_height = this->get_parameter("min_height").as_double();
  config_.max_height = this->get_parameter("max_height").as_double();
  config_.min_azimuth = this->get_parameter("min_azimuth").as_double();
  config_.max_azimuth = this->get_parameter("max_azimuth").as_double();
  config_.min_cluster_points = static_cast<int>(this->get_parameter("min_cluster_points").as_int());
  config_.max_cluster_points = static_cast<int>(this->get_parameter("max_cluster_points").as_int());
  config_.min_cluster_width = this->get_parameter("min_cluster_width").as_double();
  config_.max_cluster_width = this->get_parameter("max_cluster_width").as_double();
  config_.max_angle_gap = this->get_parameter("max_angle_gap").as_double();
  config_.max_range_jump = this->get_parameter("max_range_jump").as_double();
  config_.max_range_std = this->get_parameter("max_range_std").as_double();
  config_.max_vertical_span = this->get_parameter("max_vertical_span").as_double();
  config_.max_cluster_radius = this->get_parameter("max_cluster_radius").as_double();
  config_.position_smoothing_alpha = this->get_parameter("position_smoothing_alpha").as_double();
  config_.max_position_step = this->get_parameter("max_position_step").as_double();
  config_.max_tracking_velocity = this->get_parameter("max_tracking_velocity").as_double();
  config_.max_vertical_step = this->get_parameter("max_vertical_step").as_double();
  config_.max_vertical_velocity = this->get_parameter("max_vertical_velocity").as_double();
  config_.verbose_logging = this->get_parameter("verbose_logging").as_bool();
  config_.voxel_size = this->get_parameter_or<double>("voxel_size", 0.05);
  SyncCoreConfig();
}

rcl_interfaces::msg::SetParametersResult FrisbeeDetectorNode::ParametersCallback(
  const std::vector<rclcpp::Parameter> & params)
{
  DetectorConfig proposed = config_;
  const auto Failure = [](const std::string & reason) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = false;
      result.reason = reason;
      return result;
    };

  for (const auto & param : params) {
    const auto & name = param.get_name();
    try {
      if (name == "position_smoothing_alpha") {
        const double value = param.as_double();
        if (value < 0.0 || value > 1.0) {
          return Failure("position_smoothing_alpha must be within [0.0, 1.0].");
        }
        proposed.position_smoothing_alpha = value;
      } else if (
        name == "min_range" || name == "max_range" ||
        name == "min_cluster_width" || name == "max_cluster_width" ||
        name == "max_angle_gap" || name == "max_range_jump" ||
        name == "max_range_std" || name == "max_vertical_span" ||
        name == "max_cluster_radius" || name == "max_position_step" ||
        name == "max_tracking_velocity" || name == "max_vertical_step" ||
        name == "max_vertical_velocity")
      {
        const double value = param.as_double();
        if (value <= 0.0) {
          return Failure(name + " must be positive.");
        }
        if (name == "min_range") {
          proposed.min_range = value;
        } else if (name == "max_range") {
          proposed.max_range = value;
        } else if (name == "min_cluster_width") {
          proposed.min_cluster_width = value;
        } else if (name == "max_cluster_width") {
          proposed.max_cluster_width = value;
        } else if (name == "max_angle_gap") {
          proposed.max_angle_gap = value;
        } else if (name == "max_range_jump") {
          proposed.max_range_jump = value;
        } else if (name == "max_range_std") {
          proposed.max_range_std = value;
        } else if (name == "max_vertical_span") {
          proposed.max_vertical_span = value;
        } else if (name == "max_cluster_radius") {
          proposed.max_cluster_radius = value;
        } else if (name == "max_position_step") {
          proposed.max_position_step = value;
        } else if (name == "max_tracking_velocity") {
          proposed.max_tracking_velocity = value;
        } else if (name == "max_vertical_step") {
          proposed.max_vertical_step = value;
        } else if (name == "max_vertical_velocity") {
          proposed.max_vertical_velocity = value;
        }
      } else if (name == "min_height") {
        proposed.min_height = param.as_double();
      } else if (name == "max_height") {
        proposed.max_height = param.as_double();
      } else if (name == "min_azimuth") {
        proposed.min_azimuth = param.as_double();
      } else if (name == "max_azimuth") {
        proposed.max_azimuth = param.as_double();
      } else if (name == "min_cluster_points") {
        const int value = static_cast<int>(param.as_int());
        if (value < 1) {
          return Failure("min_cluster_points must be >= 1.");
        }
        proposed.min_cluster_points = value;
      } else if (name == "max_cluster_points") {
        const int value = static_cast<int>(param.as_int());
        if (value < 1) {
          return Failure("max_cluster_points must be >= 1.");
        }
        proposed.max_cluster_points = value;
      } else if (name == "voxel_size") {
        const double value = param.as_double();
        proposed.voxel_size = value < 0.0 ? 0.0 : value;
      }
    } catch (const rclcpp::ParameterTypeException & exc) {
      return Failure("Invalid type for parameter '" + name + "': " + std::string(exc.what()));
    }
  }

  if (proposed.max_range <= proposed.min_range) {
    return Failure("max_range must be greater than min_range.");
  }
  if (proposed.max_height <= proposed.min_height) {
    return Failure("max_height must be greater than min_height.");
  }
  if (proposed.max_cluster_points < proposed.min_cluster_points) {
    return Failure("max_cluster_points must be >= min_cluster_points.");
  }
  if (proposed.max_cluster_width <= proposed.min_cluster_width) {
    return Failure("max_cluster_width must be greater than min_cluster_width.");
  }
  if (proposed.max_azimuth <= proposed.min_azimuth) {
    return Failure("max_azimuth must be greater than min_azimuth.");
  }

  config_ = proposed;
  SyncCoreConfig();
  rcl_interfaces::msg::SetParametersResult success;
  success.successful = true;
  return success;
}

void FrisbeeDetectorNode::PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const auto candidate = core_.Detect(*msg);

  std_msgs::msg::Bool detection_msg;
  detection_msg.data = static_cast<bool>(candidate);
  detection_pub_->publish(detection_msg);

  if (candidate) {
    const auto stamp_time = (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
      ? now()
      : rclcpp::Time(msg->header.stamp);
    const double dt = (stamp_time - last_update_time_).seconds();
    last_update_time_ = stamp_time;

    const auto smoothed_position = core_.UpdateSmoothedPosition(candidate->position, dt);
    geometry_msgs::msg::PointStamped point_msg;
    point_msg.header = msg->header;
    point_msg.point.x = smoothed_position[0];
    point_msg.point.y = smoothed_position[1];
    point_msg.point.z = smoothed_position[2];
    position_pub_->publish(point_msg);

    if (config_.verbose_logging) {
      const double smoothed_range = std::hypot(smoothed_position[0], smoothed_position[1]);
      RCLCPP_INFO(
        get_logger(),
        "Frisbee detected at x=%.2f m, y=%.2f m, z=%.2f m "
        "(raw range=%.2f m, smoothed range=%.2f m, width=%.3f rad, points=%d, z_span=%.2f m).",
        smoothed_position[0], smoothed_position[1], smoothed_position[2],
        candidate->range, smoothed_range, candidate->width, candidate->points, candidate->z_span);
    }
  } else {
    if (last_detection_) {
      RCLCPP_INFO(get_logger(), "Frisbee lost.");
    }
    core_.ResetSmoothedPosition();
  }

  last_detection_ = static_cast<bool>(candidate);
}

void FrisbeeDetectorNode::SyncCoreConfig()
{
  core_.SetConfig(config_);
}

}  // namespace frisbee_detector

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<frisbee_detector::FrisbeeDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
