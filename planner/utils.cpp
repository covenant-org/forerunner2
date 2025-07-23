#include "utils.hpp"

namespace SimplePlanner {
void pathToMsg(const std::vector<std::shared_ptr<PathNode>> &path,
               nav_msgs::msg::Path &msg, std::string frame_id,
               rclcpp::Time stamp, Eigen::Vector3d &goal, tf2::Transform &t) {
  msg.poses.clear();
  for (auto node = path.begin(); node != path.end(); node++) {
    auto next_node = std::next(node);
    Eigen::Vector3d pos_next =
        next_node == path.end() ? goal : (*next_node)->coords;

    geometry_msgs::msg::PoseStamped pose;
    float currYaw = atan2((pos_next(1) - (*node)->coords(1)),
                          (pos_next(0) - (*node)->coords(0)));
    float qz = sin(currYaw / 2.0);
    float qw = cos(currYaw / 2.0);
    tf2::Quaternion tf2_quat(0.0, 0.0, qz, qw);
    tf2::Vector3 tf2_vec((*node)->coords(0), (*node)->coords(1),
                         (*node)->coords(2));
    tf2_quat = t * tf2_quat;
    tf2_vec = t * tf2_vec;
    pose.header.frame_id = frame_id;
    pose.header.stamp = stamp;
    pose.pose.position.x = tf2_vec.x();
    pose.pose.position.y = tf2_vec.y();
    pose.pose.position.z = tf2_vec.z();
    pose.pose.orientation.x = tf2_quat.x();
    pose.pose.orientation.y = tf2_quat.y();
    pose.pose.orientation.z = tf2_quat.z();
    pose.pose.orientation.w = tf2_quat.w();
    msg.poses.push_back(pose);
  }
}

void transform_path(nav_msgs::msg::Path &path, const tf2::Transform &transform,
                    std::string frame_id, rclcpp::Time stamp) {
  for (geometry_msgs::msg::PoseStamped &pose : path.poses) {
    tf2::Vector3 tf2_vec(pose.pose.position.x, pose.pose.position.y,
                         pose.pose.position.z);
    tf2::Quaternion tf2_quat(pose.pose.orientation.x, pose.pose.orientation.y,
                             pose.pose.orientation.z, pose.pose.orientation.w);
    tf2_vec = transform * tf2_vec;
    tf2_quat = transform * tf2_quat;
    pose.header.frame_id = frame_id;
    pose.header.stamp = stamp;
    pose.pose.position.x = tf2_vec.x();
    pose.pose.position.y = tf2_vec.y();
    pose.pose.position.z = tf2_vec.z();
    pose.pose.orientation.x = tf2_quat.x();
    pose.pose.orientation.y = tf2_quat.y();
    pose.pose.orientation.z = tf2_quat.z();
    pose.pose.orientation.w = tf2_quat.w();
  }
}

visualization_msgs::msg::Marker
create_marker(const pcl::PointXYZ &point, unsigned int id, rclcpp::Time stamp,
              float scale, std::optional<std::string> frame_id) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id.value_or("map");
  marker.header.stamp = stamp;
  marker.ns = id;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = point.x;
  marker.pose.position.y = point.y;
  marker.pose.position.z = point.z;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;

  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 0.5;
  marker.lifetime = rclcpp::Duration::max();

  return marker;
}

visualization_msgs::msg::Marker
create_marker(Eigen::Vector3d &point, unsigned int id, rclcpp::Time stamp,
              float scale, std::optional<std::string> frame_id) {
  pcl::PointXYZ point3d(point(0), point(1), point(2));
  return create_marker(point3d, id, stamp, scale, frame_id);
}

bool eq(const PathNode &a, const PathNode &b) { return a.coords == b.coords; }
bool eq(const PathNode &a, const std::vector<double> &b) {
  return a.coords(0) == b[0] && a.coords(1) == b[1] && a.coords(2) == b[2];
}
bool eq(const pcl::PointXYZ &a, const pcl::PointXYZ &b) {
  return a.x == b.x && a.y == b.y && a.z == b.z;
}
bool eq(const pcl::PointXYZ &a, const Eigen::Vector3d &b) {
  return a.x == b(0) && a.y == b(1) && a.z == b(2);
}

tf2::Transform fromMsgToTF(geometry_msgs::msg::TransformStamped &t) {
  tf2::Quaternion tf_quat(t.transform.rotation.x, t.transform.rotation.y,
                          t.transform.rotation.z, t.transform.rotation.w);
  tf2::Vector3 tf_vec(t.transform.translation.x, t.transform.translation.y,
                      t.transform.translation.z);
  return tf2::Transform(tf_quat, tf_vec);
}

bool isEqualDouble(double a, double b) { return fabs(a - b) < 1e-6; }

bool isInsideBox(const Eigen::Vector3d &point, const Eigen::Vector3d &min,
                 const Eigen::Vector3d &max) {
  Eigen::Vector3d dist = min - max;
  Eigen::Vector3d p2(max(0), min(1), min(2));
  Eigen::Vector3d p3(min(0), max(1), min(2));
  Eigen::Vector3d p4(min(0), min(1), max(2));
  Eigen::Vector3d u(dist(0), 0, 0);
  Eigen::Vector3d v(0, dist(1), 0);
  Eigen::Vector3d w(0, 0, dist(2));
  double udot = u.dot(point);
  double vdot = v.dot(point);
  double wdot = w.dot(point);
  return u.dot(p2) < udot && udot < u.dot(min) && v.dot(p3) < vdot &&
         vdot < v.dot(min) && w.dot(p4) < wdot && wdot < w.dot(min);
}

bool rayIntersects(const Ray &ray, const Eigen::Vector3d &min,
                   const Eigen::Vector3d &max) {
  double tmin = (min(0) - ray.origin(0)) * ray.inv_dir(0);
  double tmax = (max(0) - ray.origin(0)) * ray.inv_dir(0);

  if (tmin > tmax)
    std::swap(tmin, tmax);

  double tymin = (min(1) - ray.origin(1)) * ray.inv_dir(1);
  double tymax = (max(1) - ray.origin(1)) * ray.inv_dir(1);

  if (tymin > tymax)
    std::swap(tymin, tymax);

  if ((tmin > tymax) || (tymin > tmax))
    return false;

  if (tymin > tmin)
    tmin = tymin;
  if (tymax < tmax)
    tmax = tymax;

  double tzmin = (min(2) - ray.origin(2)) * ray.inv_dir(2);
  double tzmax = (max(2) - ray.origin(2)) * ray.inv_dir(2);

  if (tzmin > tzmax)
    std::swap(tzmin, tzmax);

  if ((tmin > tzmax) || (tzmin > tmax))
    return false;

  if (tzmin > tmin)
    tmin = tzmin;
  if (tzmax < tmax)
    tmax = tzmax;
  if (tmin < 0)
    return false;

  return true;
}

bool boxIntersects(Eigen::Vector3d minA, Eigen::Vector3d maxA,
                   Eigen::Vector3d minB, Eigen::Vector3d maxB) {
  return minA(0) <= maxB(0) && minB(0) <= maxA(0) && minA(1) <= maxB(1) &&
         minB(1) <= maxA(1) && minA(2) <= maxB(2) && minB(2) <= maxA(2);
}

double distanceToBox(Eigen::Vector3d point, Eigen::Vector3d min,
                     Eigen::Vector3d max) {
  double dx = std::max({min(0) - point(0), 0.0, point(0) - max(0)});
  double dy = std::max({min(1) - point(1), 0.0, point(1) - max(1)});
  double dz = std::max({min(2) - point(2), 0.0, point(2) - max(2)});
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

} // namespace SimplePlanner

