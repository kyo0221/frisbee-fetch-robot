#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/parameter_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/bool.hpp"

namespace frisbee_detector
{
namespace
{
constexpr std::array<const char *, 25> kRequiredParameterKeys = {
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
  "range_jump_ratio",
  "range_std_ratio",
  "cluster_radius_ratio",
  "position_smoothing_alpha",
  "max_position_step",
  "max_tracking_velocity",
  "max_vertical_step",
  "max_vertical_velocity",
  "verbose_logging"
};

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

struct ClusterPoint
{
  double angle;
  double range;
  double x;
  double y;
  double z;
};

struct Candidate
{
  std::array<double, 3> position;
  double range;
  double width;
  int points;
  double z_span;
  double score;
};

bool is_required_key(const std::string & key)
{
  return std::any_of(
    kRequiredParameterKeys.begin(), kRequiredParameterKeys.end(),
    [&](const char * required) {return key == required;});
}

std::vector<rclcpp::Parameter> LoadParametersForNode(
  const std::filesystem::path & file_path,
  const std::string & node_name,
  const std::string & node_fqn)
{
  const auto parameter_map = rclcpp::parameter_map_from_yaml_file(file_path.string());
  const std::array<std::string, 3> search_order = {
    node_fqn,
    node_name,
    "/**"
  };

  for (const auto & key : search_order) {
    auto iter = parameter_map.find(key);
    if (iter != parameter_map.end() && !iter->second.empty()) {
      return iter->second;
    }
  }

  throw std::runtime_error(
          "No parameters found for node '" + node_name + "' in '" + file_path.string() + "'.");
}

}  // namespace

class FrisbeeDetectorNode : public rclcpp::Node
{
public:
  FrisbeeDetectorNode()
  : Node("frisbee_detector")
  {
    DeclareParametersFromYaml();
    ApplyParameterValues();

    const auto qos = rclcpp::SensorDataQoS();
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, qos,
      std::bind(&FrisbeeDetectorNode::OnPointCloud, this, std::placeholders::_1));
    detection_pub_ = create_publisher<std_msgs::msg::Bool>("frisbee_detected", 10);
    position_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("frisbee_position", 10);

    parameter_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&FrisbeeDetectorNode::OnParametersSet, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Frisbee detector listening on '%s'.", cloud_topic_.c_str());
  }

private:
  void DeclareParametersFromYaml()
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

  std::optional<std::filesystem::path> ResolveParametersFilePath() const
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

  void ApplyParameterValues()
  {
    cloud_topic_ = this->get_parameter("cloud_topic").as_string();
    min_range_ = this->get_parameter("min_range").as_double();
    max_range_ = this->get_parameter("max_range").as_double();
    min_height_ = this->get_parameter("min_height").as_double();
    max_height_ = this->get_parameter("max_height").as_double();
    min_azimuth_ = this->get_parameter("min_azimuth").as_double();
    max_azimuth_ = this->get_parameter("max_azimuth").as_double();
    min_cluster_points_ = static_cast<int>(this->get_parameter("min_cluster_points").as_int());
    max_cluster_points_ = static_cast<int>(this->get_parameter("max_cluster_points").as_int());
    min_cluster_width_ = this->get_parameter("min_cluster_width").as_double();
    max_cluster_width_ = this->get_parameter("max_cluster_width").as_double();
    max_angle_gap_ = this->get_parameter("max_angle_gap").as_double();
    max_range_jump_ = this->get_parameter("max_range_jump").as_double();
    max_range_std_ = this->get_parameter("max_range_std").as_double();
    max_vertical_span_ = this->get_parameter("max_vertical_span").as_double();
    max_cluster_radius_ = this->get_parameter("max_cluster_radius").as_double();
    range_jump_ratio_ = this->get_parameter("range_jump_ratio").as_double();
    range_std_ratio_ = this->get_parameter("range_std_ratio").as_double();
    cluster_radius_ratio_ = this->get_parameter("cluster_radius_ratio").as_double();
    position_smoothing_alpha_ = this->get_parameter("position_smoothing_alpha").as_double();
    max_position_step_ = this->get_parameter("max_position_step").as_double();
    max_tracking_velocity_ = this->get_parameter("max_tracking_velocity").as_double();
    max_vertical_step_ = this->get_parameter("max_vertical_step").as_double();
    max_vertical_velocity_ = this->get_parameter("max_vertical_velocity").as_double();
    verbose_logging_ = this->get_parameter("verbose_logging").as_bool();
  }

  rcl_interfaces::msg::SetParametersResult OnParametersSet(
    const std::vector<rclcpp::Parameter> & params)
  {
    struct ProposedValues
    {
      double min_range;
      double max_range;
      double min_height;
      double max_height;
      double min_azimuth;
      double max_azimuth;
      int min_cluster_points;
      int max_cluster_points;
      double min_cluster_width;
      double max_cluster_width;
      double max_angle_gap;
      double max_range_jump;
      double max_range_std;
      double max_vertical_span;
      double max_cluster_radius;
      double range_jump_ratio;
      double range_std_ratio;
      double cluster_radius_ratio;
      double position_smoothing_alpha;
      double max_position_step;
      double max_tracking_velocity;
      double max_vertical_step;
      double max_vertical_velocity;
    } proposed{
      min_range_,
      max_range_,
      min_height_,
      max_height_,
      min_azimuth_,
      max_azimuth_,
      min_cluster_points_,
      max_cluster_points_,
      min_cluster_width_,
      max_cluster_width_,
      max_angle_gap_,
      max_range_jump_,
      max_range_std_,
      max_vertical_span_,
      max_cluster_radius_,
      range_jump_ratio_,
      range_std_ratio_,
      cluster_radius_ratio_,
      position_smoothing_alpha_,
      max_position_step_,
      max_tracking_velocity_,
      max_vertical_step_,
      max_vertical_velocity_
    };

    for (const auto & param : params) {
      const auto & name = param.get_name();
      try {
        if (name == "position_smoothing_alpha") {
          const double value = param.as_double();
          if (value < 0.0 || value > 1.0) {
            return InvalidResult("position_smoothing_alpha must be within [0.0, 1.0].");
          }
          proposed.position_smoothing_alpha = value;
        } else if (
          name == "min_range" || name == "max_range" ||
          name == "min_cluster_width" || name == "max_cluster_width" ||
          name == "max_angle_gap" || name == "max_range_jump" ||
          name == "max_range_std" || name == "max_vertical_span" ||
          name == "max_cluster_radius" || name == "range_jump_ratio" ||
          name == "range_std_ratio" || name == "cluster_radius_ratio" ||
          name == "max_position_step" || name == "max_tracking_velocity" ||
          name == "max_vertical_step" || name == "max_vertical_velocity")
        {
          const double value = param.as_double();
          if (value <= 0.0) {
            return InvalidResult(name + " must be positive.");
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
          } else if (name == "range_jump_ratio") {
            proposed.range_jump_ratio = value;
          } else if (name == "range_std_ratio") {
            proposed.range_std_ratio = value;
          } else if (name == "cluster_radius_ratio") {
            proposed.cluster_radius_ratio = value;
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
            return InvalidResult("min_cluster_points must be >= 1.");
          }
          proposed.min_cluster_points = value;
        } else if (name == "max_cluster_points") {
          const int value = static_cast<int>(param.as_int());
          if (value < 1) {
            return InvalidResult("max_cluster_points must be >= 1.");
          }
          proposed.max_cluster_points = value;
        }
      } catch (const rclcpp::ParameterTypeException & exc) {
        return InvalidResult(
          "Invalid type for parameter '" + name + "': " + std::string(exc.what()));
      }
    }

    if (proposed.max_range <= proposed.min_range) {
      return InvalidResult("max_range must be greater than min_range.");
    }
    if (proposed.max_height <= proposed.min_height) {
      return InvalidResult("max_height must be greater than min_height.");
    }
    if (proposed.max_cluster_points < proposed.min_cluster_points) {
      return InvalidResult("max_cluster_points must be >= min_cluster_points.");
    }
    if (proposed.max_cluster_width <= proposed.min_cluster_width) {
      return InvalidResult("max_cluster_width must be greater than min_cluster_width.");
    }
    if (proposed.max_azimuth <= proposed.min_azimuth) {
      return InvalidResult("max_azimuth must be greater than min_azimuth.");
    }

    ApplyParameterValues();
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  rcl_interfaces::msg::SetParametersResult InvalidResult(const std::string & reason) const
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    result.reason = reason;
    return result;
  }

  void OnPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const auto candidate = DetectCandidate(*msg);

    std_msgs::msg::Bool detection_msg;
    detection_msg.data = static_cast<bool>(candidate);
    detection_pub_->publish(detection_msg);

    if (candidate) {
      const auto smoothed = UpdateSmoothedPosition(candidate->position, msg->header.stamp);
      geometry_msgs::msg::PointStamped point_msg;
      point_msg.header = msg->header;
      point_msg.point.x = smoothed[0];
      point_msg.point.y = smoothed[1];
      point_msg.point.z = smoothed[2];
      position_pub_->publish(point_msg);
    } else {
      ResetSmoothedPosition();
    }

    if (static_cast<bool>(candidate) != last_detection_ ||
      (candidate && verbose_logging_))
    {
      if (candidate) {
        const auto position = smoothed_position_.value_or(candidate->position);
        const double smoothed_range = std::hypot(position[0], position[1]);
        RCLCPP_INFO(
          get_logger(),
          "Frisbee detected at x=%.2f m, y=%.2f m, z=%.2f m "
          "(raw range=%.2f m, smoothed range=%.2f m, width=%.3f rad, points=%d, z_span=%.2f m).",
          position[0], position[1], position[2],
          candidate->range, smoothed_range, candidate->width, candidate->points, candidate->z_span);
      } else {
        RCLCPP_INFO(get_logger(), "Frisbee lost.");
      }
    }

    last_detection_ = static_cast<bool>(candidate);
  }

  std::optional<Candidate> DetectCandidate(const sensor_msgs::msg::PointCloud2 & cloud) const
  {
    std::vector<ClusterPoint> points;
    points.reserve(cloud.width * cloud.height);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const float x = *iter_x;
      const float y = *iter_y;
      const float z = *iter_z;

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      const double xy_range = std::hypot(x, y);
      if (xy_range < min_range_ || xy_range > max_range_) {
        continue;
      }
      if (z < min_height_ || z > max_height_) {
        continue;
      }
      const double angle = std::atan2(y, x);
      if (!AngleWithinBounds(angle)) {
        continue;
      }
      points.push_back({angle, xy_range, x, y, z});
    }

    if (points.empty()) {
      return std::nullopt;
    }

    std::sort(points.begin(), points.end(), [](const ClusterPoint & lhs, const ClusterPoint & rhs) {
      return lhs.angle < rhs.angle;
    });
    points = UnwrapPoints(points);

    std::optional<Candidate> best_candidate;
    double best_score = -std::numeric_limits<double>::infinity();
    std::vector<ClusterPoint> current_cluster;
    current_cluster.reserve(points.size());
    std::optional<double> last_angle;
    std::optional<double> last_range;

    const auto finalize_cluster = [&]() {
        if (current_cluster.empty()) {
          return;
        }
        auto candidate = EvaluateCluster(current_cluster);
        if (candidate && candidate->score > best_score) {
          best_score = candidate->score;
          best_candidate = candidate;
        }
        current_cluster.clear();
        last_angle.reset();
        last_range.reset();
      };

    for (const auto & point : points) {
      if (current_cluster.empty()) {
        current_cluster.push_back(point);
        last_angle = point.angle;
        last_range = point.range;
        continue;
      }

      double angle_gap = point.angle - last_angle.value();
      if (angle_gap < 0.0) {
        angle_gap += kTwoPi;
      }

      double allowed_jump = max_range_jump_;
      if (last_range.has_value()) {
        const double ref_range = std::max(point.range, last_range.value());
        allowed_jump = std::max(max_range_jump_, range_jump_ratio_ * ref_range);
      }

      if (
        angle_gap <= max_angle_gap_ &&
        last_range.has_value() &&
        std::abs(point.range - last_range.value()) <= allowed_jump)
      {
        current_cluster.push_back(point);
      } else {
        finalize_cluster();
        current_cluster.push_back(point);
      }

      last_angle = point.angle;
      last_range = point.range;
    }

    finalize_cluster();
    return best_candidate;
  }

  std::optional<Candidate> EvaluateCluster(const std::vector<ClusterPoint> & cluster) const
  {
    const auto count = static_cast<int>(cluster.size());
    if (count < min_cluster_points_ || count > max_cluster_points_) {
      return std::nullopt;
    }

    const double first_angle = cluster.front().angle;
    double angular_span = cluster.back().angle - first_angle;
    if (angular_span < 0.0) {
      angular_span += kTwoPi;
    }
    if (angular_span < min_cluster_width_ || angular_span > max_cluster_width_) {
      return std::nullopt;
    }

    double sum_range = 0.0;
    for (const auto & point : cluster) {
      sum_range += point.range;
    }
    const double mean_range = sum_range / static_cast<double>(count);
    double variance = 0.0;
    for (const auto & point : cluster) {
      const double delta = point.range - mean_range;
      variance += delta * delta;
    }
    variance /= static_cast<double>(count);
    const double std_dev = std::sqrt(std::max(variance, 0.0));
    const double allowed_std = max_range_std_ + range_std_ratio_ * mean_range;
    if (std_dev > allowed_std) {
      return std::nullopt;
    }

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::lowest();
    for (const auto & point : cluster) {
      sum_x += point.x;
      sum_y += point.y;
      sum_z += point.z;
      min_z = std::min(min_z, point.z);
      max_z = std::max(max_z, point.z);
    }

    const double centroid_x = sum_x / static_cast<double>(count);
    const double centroid_y = sum_y / static_cast<double>(count);
    const double centroid_z = sum_z / static_cast<double>(count);

    const double z_span = max_z - min_z;
    const double allowed_z_span = max_vertical_span_ + 0.01 * mean_range;
    if (z_span > allowed_z_span) {
      return std::nullopt;
    }

    double max_radius = 0.0;
    for (const auto & point : cluster) {
      const double dx = point.x - centroid_x;
      const double dy = point.y - centroid_y;
      const double dz = point.z - centroid_z;
      const double radius = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (radius > max_radius) {
        max_radius = radius;
      }
    }

    const double allowed_radius = max_cluster_radius_ + cluster_radius_ratio_ * mean_range;
    if (max_radius > allowed_radius) {
      return std::nullopt;
    }

    Candidate candidate;
    candidate.position = {centroid_x, centroid_y, centroid_z};
    candidate.range = std::hypot(centroid_x, centroid_y);
    candidate.width = angular_span;
    candidate.points = count;
    candidate.z_span = z_span;
    candidate.score =
      1.0 / (std_dev + 1e-3) +
      1.0 / (angular_span + 1e-3) +
      1.0 / (z_span + 1e-3) +
      1.0 / (max_radius + 1e-3);

    return candidate;
  }

  std::vector<ClusterPoint> UnwrapPoints(const std::vector<ClusterPoint> & points) const
  {
    if (points.size() <= 1) {
      return points;
    }

    double max_gap = -1.0;
    size_t start_idx = 0;
    for (size_t i = 0; i + 1 < points.size(); ++i) {
      const double gap = points[i + 1].angle - points[i].angle;
      if (gap > max_gap) {
        max_gap = gap;
        start_idx = i + 1;
      }
    }

    const double wrap_gap = (points.front().angle + kTwoPi) - points.back().angle;
    if (wrap_gap > max_gap) {
      start_idx = 0;
    }

    std::vector<ClusterPoint> rotated;
    rotated.reserve(points.size());
    for (size_t offset = 0; offset < points.size(); ++offset) {
      const size_t idx = (start_idx + offset) % points.size();
      ClusterPoint point = points[idx];
      if (idx < start_idx) {
        point.angle += kTwoPi;
      }
      rotated.push_back(point);
    }

    return rotated;
  }

  std::array<double, 3> UpdateSmoothedPosition(
    const std::array<double, 3> & new_position,
    const builtin_interfaces::msg::Time & stamp)
  {
    rclcpp::Time current_time;
    if (stamp.sec == 0 && stamp.nanosec == 0) {
      current_time = this->get_clock()->now();
    } else {
      current_time = rclcpp::Time(stamp);
    }

    if (!smoothed_position_.has_value() || !has_last_update_time_) {
      smoothed_position_ = new_position;
      last_update_time_ = current_time;
      has_last_update_time_ = true;
      return smoothed_position_.value();
    }

    const double dt = std::max((current_time - last_update_time_).seconds(), 1e-3);
    last_update_time_ = current_time;

    const double alpha = std::clamp(position_smoothing_alpha_, 0.0, 1.0);
    std::array<double, 3> blended = {
      smoothed_position_.value()[0] * (1.0 - alpha) + new_position[0] * alpha,
      smoothed_position_.value()[1] * (1.0 - alpha) + new_position[1] * alpha,
      smoothed_position_.value()[2] * (1.0 - alpha) + new_position[2] * alpha
    };

    std::array<double, 3> delta = {
      blended[0] - smoothed_position_.value()[0],
      blended[1] - smoothed_position_.value()[1],
      blended[2] - smoothed_position_.value()[2]
    };

    const double horizontal_delta = std::hypot(delta[0], delta[1]);
    const double max_horizontal_step = std::max(max_position_step_, max_tracking_velocity_ * dt);
    if (horizontal_delta > max_horizontal_step && horizontal_delta > 0.0) {
      const double scale = max_horizontal_step / horizontal_delta;
      delta[0] *= scale;
      delta[1] *= scale;
    }

    const double vertical_delta = std::abs(delta[2]);
    const double max_vertical_step = std::max(max_vertical_step_, max_vertical_velocity_ * dt);
    if (vertical_delta > max_vertical_step) {
      delta[2] = std::copysign(max_vertical_step, delta[2]);
    }

    smoothed_position_ = {
      smoothed_position_.value()[0] + delta[0],
      smoothed_position_.value()[1] + delta[1],
      smoothed_position_.value()[2] + delta[2]
    };

    return smoothed_position_.value();
  }

  void ResetSmoothedPosition()
  {
    smoothed_position_.reset();
    has_last_update_time_ = false;
  }

  bool AngleWithinBounds(double angle) const
  {
    if (min_azimuth_ <= max_azimuth_) {
      return angle >= min_azimuth_ && angle <= max_azimuth_;
    }
    return angle >= min_azimuth_ || angle <= max_azimuth_;
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detection_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr position_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  std::string cloud_topic_;
  double min_range_{0.0};
  double max_range_{0.0};
  double min_height_{0.0};
  double max_height_{0.0};
  double min_azimuth_{0.0};
  double max_azimuth_{0.0};
  int min_cluster_points_{0};
  int max_cluster_points_{0};
  double min_cluster_width_{0.0};
  double max_cluster_width_{0.0};
  double max_angle_gap_{0.0};
  double max_range_jump_{0.0};
  double max_range_std_{0.0};
  double max_vertical_span_{0.0};
  double max_cluster_radius_{0.0};
  double range_jump_ratio_{0.0};
  double range_std_ratio_{0.0};
  double cluster_radius_ratio_{0.0};
  double position_smoothing_alpha_{0.0};
  double max_position_step_{0.0};
  double max_tracking_velocity_{0.0};
  double max_vertical_step_{0.0};
  double max_vertical_velocity_{0.0};
  bool verbose_logging_{false};

  bool last_detection_{false};
  std::optional<std::array<double, 3>> smoothed_position_;
  rclcpp::Time last_update_time_;
  bool has_last_update_time_{false};
};

}  // namespace frisbee_detector

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<frisbee_detector::FrisbeeDetectorNode>();
  try {
    rclcpp::spin(node);
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(node->get_logger(), "Frisbee detector terminated: %s", exc.what());
  }
  rclcpp::shutdown();
  return 0;
}
