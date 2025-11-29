#pragma once

#include <array>
#include <optional>
#include <vector>

#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace frisbee_detector
{

struct Candidate
{
  std::array<double, 3> position;
  double range;
  double width;
  int points;
  double z_span;
  double score;
};

struct DetectorConfig
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
  double position_smoothing_alpha;
  double max_position_step;
  double max_tracking_velocity;
  double max_vertical_step;
  double max_vertical_velocity;
  bool verbose_logging;
  double voxel_size;
};

class FrisbeeDetectorCore
{
public:
  explicit FrisbeeDetectorCore(const DetectorConfig & config);
  void SetConfig(const DetectorConfig & config) {config_ = config;}

  std::optional<Candidate> Detect(const sensor_msgs::msg::PointCloud2 & cloud) const;

  std::array<double, 3> UpdateSmoothedPosition(
    const std::array<double, 3> & new_position,
    double dt_seconds);

  void ResetSmoothedPosition();

  const std::optional<std::array<double, 3>> & smoothed_position() const {return smoothed_position_;}

private:
  struct ClusterPoint
  {
    double angle;
    double range;
    double x;
    double y;
    double z;
  };

  bool AngleWithinBounds(double angle) const;
  std::vector<ClusterPoint> CollectFilteredPoints(const sensor_msgs::msg::PointCloud2 & cloud) const;
  std::vector<ClusterPoint> UnwrapPoints(const std::vector<ClusterPoint> & points) const;
  std::optional<Candidate> EvaluateCluster(const std::vector<ClusterPoint> & cluster) const;

  DetectorConfig config_;

  std::optional<std::array<double, 3>> smoothed_position_;
};

}  // namespace frisbee_detector
