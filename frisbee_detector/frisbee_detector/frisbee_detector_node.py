import math
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import PointStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool

ClusterPoint = Tuple[float, float, float, float, float]


class FrisbeeDetectorNode(Node):
    """Rule-based detector for a frisbee using 3D LiDAR point clouds."""

    def __init__(self) -> None:
        super().__init__('frisbee_detector')

        # Declare configurable parameters with sensible defaults.
        self.declare_parameter('cloud_topic', '/livox/lidar')
        self.declare_parameter('min_range', 1.0)
        self.declare_parameter('max_range', 5.0)
        self.declare_parameter('min_height', 0.3)
        self.declare_parameter('max_height', 2.1)
        self.declare_parameter('min_azimuth', -math.pi / 6.0)
        self.declare_parameter('max_azimuth', math.pi / 6.0)
        self.declare_parameter('min_cluster_points', 4)
        self.declare_parameter('max_cluster_points', 140)
        self.declare_parameter('min_cluster_width', 0.01)  # radians
        self.declare_parameter('max_cluster_width', 0.35)  # radians
        self.declare_parameter('max_angle_gap', 0.07)  # radians between consecutive points
        self.declare_parameter('max_range_jump', 0.35)  # meters difference between neighbouring points
        self.declare_parameter('max_range_std', 0.12)  # meters
        self.declare_parameter('max_vertical_span', 0.18)  # meters
        self.declare_parameter('max_cluster_radius', 0.35)  # meters (3D radius from centroid)
        self.declare_parameter('range_jump_ratio', 0.02)  # scales allowed range jump with distance
        self.declare_parameter('range_std_ratio', 0.015)  # scales allowed radial std with distance
        self.declare_parameter('cluster_radius_ratio', 0.02)  # scales allowed radius with distance
        self.declare_parameter('position_smoothing_alpha', 0.7)
        self.declare_parameter('max_position_step', 3.5)  # meters per update (horizontal plane)
        self.declare_parameter('max_tracking_velocity', 18.0)  # meters per second (horizontal plane)
        self.declare_parameter('max_vertical_step', 2.5)  # meters per update
        self.declare_parameter('max_vertical_velocity', 15.0)  # meters per second
        self.declare_parameter('verbose_logging', False)

        self._apply_parameter_values()

        cloud_topic = self.get_parameter('cloud_topic').value
        self._cloud_sub = self.create_subscription(
            PointCloud2,
            cloud_topic,
            self._on_pointcloud,
            qos_profile_sensor_data,
        )
        self._detection_pub = self.create_publisher(Bool, 'frisbee_detected', 10)
        self._position_pub = self.create_publisher(PointStamped, 'frisbee_position', 10)

        self._last_detection = False
        self._smoothed_position: Optional[Tuple[float, float, float]] = None
        self._last_update_time: Optional[Time] = None

        self.add_on_set_parameters_callback(self._on_parameters_set)

        self.get_logger().info(f"Frisbee detector listening on '{cloud_topic}'.")

    def _apply_parameter_values(self) -> None:
        self.cloud_topic = str(self.get_parameter('cloud_topic').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.min_height = float(self.get_parameter('min_height').value)
        self.max_height = float(self.get_parameter('max_height').value)
        self.min_azimuth = float(self.get_parameter('min_azimuth').value)
        self.max_azimuth = float(self.get_parameter('max_azimuth').value)
        self.min_cluster_points = int(self.get_parameter('min_cluster_points').value)
        self.max_cluster_points = int(self.get_parameter('max_cluster_points').value)
        self.min_cluster_width = float(self.get_parameter('min_cluster_width').value)
        self.max_cluster_width = float(self.get_parameter('max_cluster_width').value)
        self.max_angle_gap = float(self.get_parameter('max_angle_gap').value)
        self.max_range_jump = float(self.get_parameter('max_range_jump').value)
        self.max_range_std = float(self.get_parameter('max_range_std').value)
        self.max_vertical_span = float(self.get_parameter('max_vertical_span').value)
        self.max_cluster_radius = float(self.get_parameter('max_cluster_radius').value)
        self.range_jump_ratio = float(self.get_parameter('range_jump_ratio').value)
        self.range_std_ratio = float(self.get_parameter('range_std_ratio').value)
        self.cluster_radius_ratio = float(self.get_parameter('cluster_radius_ratio').value)
        self.position_smoothing_alpha = float(self.get_parameter('position_smoothing_alpha').value)
        self.max_position_step = float(self.get_parameter('max_position_step').value)
        self.max_tracking_velocity = float(self.get_parameter('max_tracking_velocity').value)
        self.max_vertical_step = float(self.get_parameter('max_vertical_step').value)
        self.max_vertical_velocity = float(self.get_parameter('max_vertical_velocity').value)
        self.verbose_logging = bool(self.get_parameter('verbose_logging').value)

    def _on_parameters_set(self, params) -> SetParametersResult:
        proposed = {
            'min_range': self.min_range,
            'max_range': self.max_range,
            'min_height': self.min_height,
            'max_height': self.max_height,
            'min_azimuth': self.min_azimuth,
            'max_azimuth': self.max_azimuth,
            'min_cluster_points': self.min_cluster_points,
            'max_cluster_points': self.max_cluster_points,
            'min_cluster_width': self.min_cluster_width,
            'max_cluster_width': self.max_cluster_width,
            'max_angle_gap': self.max_angle_gap,
            'max_range_jump': self.max_range_jump,
            'max_range_std': self.max_range_std,
            'max_vertical_span': self.max_vertical_span,
            'max_cluster_radius': self.max_cluster_radius,
            'range_jump_ratio': self.range_jump_ratio,
            'range_std_ratio': self.range_std_ratio,
            'cluster_radius_ratio': self.cluster_radius_ratio,
            'position_smoothing_alpha': self.position_smoothing_alpha,
            'max_position_step': self.max_position_step,
            'max_tracking_velocity': self.max_tracking_velocity,
            'max_vertical_step': self.max_vertical_step,
            'max_vertical_velocity': self.max_vertical_velocity,
        }

        alpha_params = {'position_smoothing_alpha'}
        positive_float_params = {
            'min_range',
            'max_range',
            'min_cluster_width',
            'max_cluster_width',
            'max_angle_gap',
            'max_range_jump',
            'max_range_std',
            'max_vertical_span',
            'max_cluster_radius',
            'range_jump_ratio',
            'range_std_ratio',
            'cluster_radius_ratio',
            'max_position_step',
            'max_tracking_velocity',
            'max_vertical_step',
            'max_vertical_velocity',
        }
        height_params = {'min_height', 'max_height'}
        azimuth_params = {'min_azimuth', 'max_azimuth'}

        for param in params:
            name = param.name
            value = param.value

            if name not in proposed:
                continue

            if name in alpha_params:
                value = float(value)
                if not (0.0 <= value <= 1.0):
                    return SetParametersResult(successful=False, reason=f"{name} must be within [0.0, 1.0].")
            elif name in positive_float_params:
                value = float(value)
                if value <= 0.0:
                    return SetParametersResult(successful=False, reason=f"{name} must be positive.")
            elif name in height_params:
                value = float(value)
            elif name in azimuth_params:
                value = float(value)
            elif name in ('min_cluster_points', 'max_cluster_points'):
                value = int(value)
                if value < 1:
                    return SetParametersResult(successful=False, reason=f"{name} must be >= 1.")

            proposed[name] = value

        if proposed['max_range'] <= proposed['min_range']:
            return SetParametersResult(successful=False, reason="max_range must be greater than min_range.")
        if proposed['max_height'] <= proposed['min_height']:
            return SetParametersResult(successful=False, reason="max_height must be greater than min_height.")
        if proposed['max_cluster_points'] < proposed['min_cluster_points']:
            return SetParametersResult(successful=False, reason="max_cluster_points must be >= min_cluster_points.")
        if proposed['max_cluster_width'] <= proposed['min_cluster_width']:
            return SetParametersResult(successful=False, reason="max_cluster_width must be greater than min_cluster_width.")
        if proposed['max_azimuth'] <= proposed['min_azimuth']:
            return SetParametersResult(successful=False, reason="max_azimuth must be greater than min_azimuth.")

        self._apply_parameter_values()
        return SetParametersResult(successful=True)

    def _on_pointcloud(self, cloud: PointCloud2) -> None:
        candidate = self._detect_candidate(cloud)

        detected = candidate is not None
        detection_msg = Bool()
        detection_msg.data = detected
        self._detection_pub.publish(detection_msg)

        if detected and candidate:
            smoothed_position = self._update_smoothed_position(candidate['position'], cloud.header.stamp)
            position_msg = PointStamped()
            position_msg.header = cloud.header
            position_msg.point.x = smoothed_position[0]
            position_msg.point.y = smoothed_position[1]
            position_msg.point.z = smoothed_position[2]
            self._position_pub.publish(position_msg)
        else:
            self._reset_smoothed_position()

        if detected != self._last_detection or (detected and self.verbose_logging):
            if detected and candidate:
                smoothed_position = self._smoothed_position if self._smoothed_position else candidate['position']
                smoothed_range = math.hypot(smoothed_position[0], smoothed_position[1])
                info = (
                    f"Frisbee detected at x={smoothed_position[0]:.2f} m, "
                    f"y={smoothed_position[1]:.2f} m, "
                    f"z={smoothed_position[2]:.2f} m "
                    f"(raw range={candidate['range']:.2f} m, smoothed range={smoothed_range:.2f} m, "
                    f"width={candidate['width']:.3f} rad, "
                    f"points={candidate['points']}, "
                    f"z_span={candidate['z_span']:.2f} m)."
                )
                self.get_logger().info(info)
            else:
                self.get_logger().info("Frisbee lost.")

        self._last_detection = detected

    def _detect_candidate(self, cloud: PointCloud2) -> Optional[dict]:
        points: List[ClusterPoint] = []
        for x, y, z in point_cloud2.read_points(cloud, field_names=('x', 'y', 'z'), skip_nans=True):
            xy_range = math.hypot(x, y)
            if xy_range < self.min_range or xy_range > self.max_range:
                continue
            if z < self.min_height or z > self.max_height:
                continue
            angle = math.atan2(y, x)
            if not self._angle_within_bounds(angle):
                continue
            points.append((angle, xy_range, x, y, z))

        if not points:
            return None

        points.sort(key=lambda item: item[0])
        points = self._unwrap_points(points)

        best_candidate = None
        best_score = -math.inf
        current_cluster: List[ClusterPoint] = []
        last_angle: Optional[float] = None
        last_range: Optional[float] = None

        def finalize_cluster() -> None:
            nonlocal current_cluster, best_candidate, best_score, last_angle, last_range
            if not current_cluster:
                return
            candidate = self._evaluate_cluster(current_cluster)
            if candidate and candidate['score'] > best_score:
                best_candidate = candidate
                best_score = candidate['score']
            current_cluster = []
            last_angle = None
            last_range = None

        for angle, rng, x, y, z in points:
            if not current_cluster:
                current_cluster = [(angle, rng, x, y, z)]
                last_angle = angle
                last_range = rng
                continue

            angle_gap = angle - (last_angle if last_angle is not None else angle)
            if angle_gap < 0.0:
                angle_gap += 2.0 * math.pi

            allowed_jump = self.max_range_jump
            if last_range is not None:
                ref_range = max(rng, last_range)
                allowed_jump = max(self.max_range_jump, self.range_jump_ratio * ref_range)

            if angle_gap <= self.max_angle_gap and last_range is not None and abs(rng - last_range) <= allowed_jump:
                current_cluster.append((angle, rng, x, y, z))
            else:
                finalize_cluster()
                current_cluster = [(angle, rng, x, y, z)]
            last_angle = angle
            last_range = rng

        finalize_cluster()
        return best_candidate

    def _evaluate_cluster(self, cluster: Sequence[ClusterPoint]) -> Optional[dict]:
        count = len(cluster)
        if count < self.min_cluster_points or count > self.max_cluster_points:
            return None

        angles = [item[0] for item in cluster]
        angular_span = angles[-1] - angles[0]
        if angular_span < 0.0:
            angular_span += 2.0 * math.pi

        if angular_span < self.min_cluster_width or angular_span > self.max_cluster_width:
            return None

        ranges = [item[1] for item in cluster]
        mean_range = sum(ranges) / float(count)
        variance = sum((r - mean_range) ** 2 for r in ranges) / float(count)
        std_dev = math.sqrt(max(variance, 0.0))
        allowed_std = self.max_range_std + self.range_std_ratio * mean_range
        if std_dev > allowed_std:
            return None

        xs = [item[2] for item in cluster]
        ys = [item[3] for item in cluster]
        zs = [item[4] for item in cluster]

        z_span = max(zs) - min(zs)
        allowed_z_span = self.max_vertical_span + 0.01 * mean_range
        if z_span > allowed_z_span:
            return None

        centroid_x = sum(xs) / float(count)
        centroid_y = sum(ys) / float(count)
        centroid_z = sum(zs) / float(count)

        max_radius = 0.0
        for x, y, z in zip(xs, ys, zs):
            radius = math.sqrt(
                (x - centroid_x) ** 2 +
                (y - centroid_y) ** 2 +
                (z - centroid_z) ** 2
            )
            max_radius = max(max_radius, radius)

        allowed_radius = self.max_cluster_radius + self.cluster_radius_ratio * mean_range
        if max_radius > allowed_radius:
            return None

        centroid_range = math.hypot(centroid_x, centroid_y)

        # Heuristic score favouring compact, low-variance clusters.
        score = (
            1.0 / (std_dev + 1e-3)
            + 1.0 / (angular_span + 1e-3)
            + 1.0 / (z_span + 1e-3)
            + 1.0 / (max_radius + 1e-3)
        )

        return {
            'position': (centroid_x, centroid_y, centroid_z),
            'range': centroid_range,
            'width': angular_span,
            'points': count,
            'z_span': z_span,
            'score': score,
        }

    def _unwrap_points(self, points: Sequence[ClusterPoint]) -> List[ClusterPoint]:
        if len(points) <= 1:
            return list(points)

        angles = [p[0] for p in points]
        max_gap = -1.0
        start_idx = 0
        two_pi = 2.0 * math.pi

        for i in range(len(points) - 1):
            gap = angles[i + 1] - angles[i]
            if gap > max_gap:
                max_gap = gap
                start_idx = i + 1

        wrap_gap = (angles[0] + two_pi) - angles[-1]
        if wrap_gap > max_gap:
            start_idx = 0

        rotated: List[ClusterPoint] = []
        total = len(points)
        for offset in range(total):
            idx = (start_idx + offset) % total
            angle, rng, x, y, z = points[idx]
            if idx < start_idx:
                angle += two_pi
            rotated.append((angle, rng, x, y, z))

        return rotated

    def _update_smoothed_position(
        self,
        new_position: Tuple[float, float, float],
        stamp,
    ) -> Tuple[float, float, float]:
        current_time = Time.from_msg(stamp) if stamp.sec != 0 or stamp.nanosec != 0 else self.get_clock().now()

        if self._smoothed_position is None or self._last_update_time is None:
            self._smoothed_position = new_position
            self._last_update_time = current_time
            return new_position

        dt = (current_time - self._last_update_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 1e-3

        old_x, old_y, old_z = self._smoothed_position
        alpha = max(min(self.position_smoothing_alpha, 1.0), 0.0)
        blended = (
            old_x * (1.0 - alpha) + new_position[0] * alpha,
            old_y * (1.0 - alpha) + new_position[1] * alpha,
            old_z * (1.0 - alpha) + new_position[2] * alpha,
        )

        delta_vec = (
            blended[0] - old_x,
            blended[1] - old_y,
            blended[2] - old_z,
        )
        horizontal_delta = math.hypot(delta_vec[0], delta_vec[1])
        max_horizontal_step = max(self.max_position_step, self.max_tracking_velocity * dt)
        if horizontal_delta > max_horizontal_step and horizontal_delta > 0.0:
            scale = max_horizontal_step / horizontal_delta
            delta_vec = (
                delta_vec[0] * scale,
                delta_vec[1] * scale,
                delta_vec[2],
            )

        vertical_delta = abs(delta_vec[2])
        max_vertical_step = max(self.max_vertical_step, self.max_vertical_velocity * dt)
        if vertical_delta > max_vertical_step:
            delta_vec = (
                delta_vec[0],
                delta_vec[1],
                math.copysign(max_vertical_step, delta_vec[2]),
            )

        self._smoothed_position = (
            old_x + delta_vec[0],
            old_y + delta_vec[1],
            old_z + delta_vec[2],
        )
        self._last_update_time = current_time
        return self._smoothed_position

    def _reset_smoothed_position(self) -> None:
        self._smoothed_position = None
        self._last_update_time = None

    def _angle_within_bounds(self, angle: float) -> bool:
        if self.min_azimuth <= self.max_azimuth:
            return self.min_azimuth <= angle <= self.max_azimuth
        return angle >= self.min_azimuth or angle <= self.max_azimuth


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrisbeeDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
