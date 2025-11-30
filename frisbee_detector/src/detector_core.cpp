#include "frisbee_detector/detector_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace frisbee_detector
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kRangeJumpRatio = 0.02;
constexpr double kRangeStdRatio = 0.015;
constexpr double kClusterRadiusRatio = 0.02;

struct VoxelKey
{
  int ix;
  int iy;
  int iz;

  bool operator==(const VoxelKey & other) const
  {
    return ix == other.ix && iy == other.iy && iz == other.iz;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const noexcept
  {
    std::size_t seed = static_cast<std::size_t>(key.ix);
    seed ^= static_cast<std::size_t>(key.iy) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= static_cast<std::size_t>(key.iz) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};
}  // namespace

FrisbeeDetectorCore::FrisbeeDetectorCore(const DetectorConfig & config)
: config_(config)
{
}

bool FrisbeeDetectorCore::AngleWithinBounds(double angle) const
{
  if (config_.min_azimuth <= config_.max_azimuth) {
    return angle >= config_.min_azimuth && angle <= config_.max_azimuth;
  }
  return angle >= config_.min_azimuth || angle <= config_.max_azimuth;
}

std::vector<FrisbeeDetectorCore::ClusterPoint> FrisbeeDetectorCore::CollectFilteredPoints(
  const sensor_msgs::msg::PointCloud2 & cloud) const
{
  std::vector<ClusterPoint> points;
  points.reserve(cloud.width * cloud.height);

  const bool use_voxel = config_.voxel_size > 0.0;
  const double voxel = config_.voxel_size;
  std::unordered_map<VoxelKey, ClusterPoint, VoxelKeyHash> voxel_map;
  if (use_voxel) {
    voxel_map.reserve(cloud.width * cloud.height / 4);
  }

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
    if (xy_range < config_.min_range || xy_range > config_.max_range) {
      continue;
    }
    if (z < config_.min_height || z > config_.max_height) {
      continue;
    }
    const double angle = std::atan2(y, x);
    if (!AngleWithinBounds(angle)) {
      continue;
    }

    ClusterPoint point{angle, xy_range, static_cast<double>(x), static_cast<double>(y), static_cast<double>(z)};
    if (!use_voxel) {
      points.push_back(point);
      continue;
    }

    VoxelKey key{
      static_cast<int>(std::floor(point.x / voxel)),
      static_cast<int>(std::floor(point.y / voxel)),
      static_cast<int>(std::floor(point.z / voxel))
    };

    auto inserted = voxel_map.emplace(key, point);
    if (!inserted.second) {
      // keep closer point
      if (point.range < inserted.first->second.range) {
        inserted.first->second = point;
      }
    }
  }

  if (use_voxel) {
    points.reserve(voxel_map.size());
    for (const auto & entry : voxel_map) {
      points.push_back(entry.second);
    }
  }

  std::sort(points.begin(), points.end(), [](const ClusterPoint & lhs, const ClusterPoint & rhs) {
    return lhs.angle < rhs.angle;
  });
  return points;
}

std::optional<Candidate> FrisbeeDetectorCore::Detect(const sensor_msgs::msg::PointCloud2 & cloud) const
{
  auto points = CollectFilteredPoints(cloud);
  if (points.empty()) {
    return std::nullopt;
  }

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

    double allowed_jump = config_.max_range_jump;
    if (last_range.has_value()) {
      const double ref_range = std::max(point.range, last_range.value());
      allowed_jump = std::max(config_.max_range_jump, kRangeJumpRatio * ref_range);
    }

    if (
      angle_gap <= config_.max_angle_gap &&
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

std::optional<Candidate> FrisbeeDetectorCore::EvaluateCluster(
  const std::vector<ClusterPoint> & cluster) const
{
  const auto count = static_cast<int>(cluster.size());
  if (count < config_.min_cluster_points || count > config_.max_cluster_points) {
    return std::nullopt;
  }

  const double first_angle = cluster.front().angle;
  double angular_span = cluster.back().angle - first_angle;
  if (angular_span < 0.0) {
    angular_span += kTwoPi;
  }
  if (angular_span < config_.min_cluster_width || angular_span > config_.max_cluster_width) {
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
  const double allowed_std = config_.max_range_std + kRangeStdRatio * mean_range;
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
  const double allowed_z_span = config_.max_vertical_span + 0.01 * mean_range;
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

  const double allowed_radius = config_.max_cluster_radius + kClusterRadiusRatio * mean_range;
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

std::vector<FrisbeeDetectorCore::ClusterPoint> FrisbeeDetectorCore::UnwrapPoints(
  const std::vector<ClusterPoint> & points) const
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

std::array<double, 3> FrisbeeDetectorCore::UpdateSmoothedPosition(
  const std::array<double, 3> & new_position,
  double dt_seconds)
{
  if (!smoothed_position_.has_value()) {
    smoothed_position_ = new_position;
    return smoothed_position_.value();
  }

  const double dt = std::max(dt_seconds, 1e-3);
  const double alpha = std::clamp(config_.position_smoothing_alpha, 0.0, 1.0);

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
  const double max_horizontal_step =
    std::max(config_.max_position_step, config_.max_tracking_velocity * dt);
  if (horizontal_delta > max_horizontal_step && horizontal_delta > 0.0) {
    const double scale = max_horizontal_step / horizontal_delta;
    delta[0] *= scale;
    delta[1] *= scale;
  }

  const double vertical_delta = std::abs(delta[2]);
  const double max_vertical_step =
    std::max(config_.max_vertical_step, config_.max_vertical_velocity * dt);
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

void FrisbeeDetectorCore::ResetSmoothedPosition()
{
  smoothed_position_.reset();
}

}  // namespace frisbee_detector
