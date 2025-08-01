#include "utils.hpp"
#include <Eigen/src/Geometry/Quaternion.h>
#include <capnp_schemas/visualization_msgs.capnp.h>

namespace SimplePlanner {
void pathToMsg(const std::vector<std::shared_ptr<PathNode>> &path,
               Path::Builder &msg, Eigen::Vector3d &goal, Eigen::Affine3d &t) {
  auto poses = msg.initPoses(path.size());
  for (size_t i = 0; i < path.size(); ++i) {
    auto node = path[i];
    auto pose = poses[i];

    pose.getPose().initOrientation();
    pose.getPose().initPosition();

    Eigen::Vector3d pos_next =
        (i + 1 < path.size()) ? path[i + 1]->coords : goal;

    float currYaw =
        atan2(pos_next.y() - node->coords.y(), pos_next.x() - node->coords.x());
    float qz = sin(currYaw / 2.0);
    float qw = cos(currYaw / 2.0);

    Eigen::Quaterniond quat(qw, 0.0, 0.0, qz);
    Eigen::Vector3d translation(node->coords.x(), node->coords.y(),
                                     node->coords.z());

    quat = t.rotation() * quat;
    translation = t * translation;

    pose.getPose().getPosition().setX(node->coords.x());
    pose.getPose().getPosition().setY(node->coords.y());
    pose.getPose().getPosition().setZ(node->coords.z());

    pose.getPose().getOrientation().setX(0.0);
    pose.getPose().getOrientation().setY(0.0);
    pose.getPose().getOrientation().setZ(qz);
    pose.getPose().getOrientation().setW(qw);
  }
}
//
// void transform_path(nav_msgs::msg::Path &path, const tf2::Transform
// &transform,
//                     std::string frame_id, rclcpp::Time stamp) {
//   for (geometry_msgs::msg::PoseStamped &pose : path.poses) {
//     tf2::Vector3 tf2_vec(pose.pose.position.x, pose.pose.position.y,
//                          pose.pose.position.z);
//     tf2::Quaternion tf2_quat(pose.pose.orientation.x,
//     pose.pose.orientation.y,
//                              pose.pose.orientation.z,
//                              pose.pose.orientation.w);
//     tf2_vec = transform * tf2_vec;
//     tf2_quat = transform * tf2_quat;
//     pose.header.frame_id = frame_id;
//     pose.header.stamp = stamp;
//     pose.pose.position.x = tf2_vec.x();
//     pose.pose.position.y = tf2_vec.y();
//     pose.pose.position.z = tf2_vec.z();
//     pose.pose.orientation.x = tf2_quat.x();
//     pose.pose.orientation.y = tf2_quat.y();
//     pose.pose.orientation.z = tf2_quat.z();
//     pose.pose.orientation.w = tf2_quat.w();
//   }
// }

void create_marker(Marker::Builder &marker, const pcl::PointXYZ &point,
                   float scale) {
  marker.setShape(MarkerType::CUBE);

  marker.initColor();
  marker.initHeader();
  marker.initPose();
  marker.initScale();
  marker.getPose().initOrientation();
  marker.getPose().initPosition();

  marker.getPose().getPosition().setX(point.x);
  marker.getPose().getPosition().setY(point.y);
  marker.getPose().getPosition().setZ(point.z);

  marker.getPose().getOrientation().setX(0.0);
  marker.getPose().getOrientation().setY(0.0);
  marker.getPose().getOrientation().setZ(0.0);
  marker.getPose().getOrientation().setW(1.0);

  marker.getScale().setX(scale);
  marker.getScale().setY(scale);
  marker.getScale().setZ(scale);

  marker.getColor().setR(0.5);
  marker.getColor().setG(0.5);
  marker.getColor().setB(0.5);
  marker.getColor().setA(0.5);
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

// tf2::Transform fromMsgToTF(geometry_msgs::msg::TransformStamped &t) {
//   tf2::Quaternion tf_quat(t.transform.rotation.x, t.transform.rotation.y,
//                           t.transform.rotation.z, t.transform.rotation.w);
//   tf2::Vector3 tf_vec(t.transform.translation.x, t.transform.translation.y,
//                       t.transform.translation.z);
//   return tf2::Transform(tf_quat, tf_vec);
// }

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

  if (tmin > tmax) std::swap(tmin, tmax);

  double tymin = (min(1) - ray.origin(1)) * ray.inv_dir(1);
  double tymax = (max(1) - ray.origin(1)) * ray.inv_dir(1);

  if (tymin > tymax) std::swap(tymin, tymax);

  if ((tmin > tymax) || (tymin > tmax)) return false;

  if (tymin > tmin) tmin = tymin;
  if (tymax < tmax) tmax = tymax;

  double tzmin = (min(2) - ray.origin(2)) * ray.inv_dir(2);
  double tzmax = (max(2) - ray.origin(2)) * ray.inv_dir(2);

  if (tzmin > tzmax) std::swap(tzmin, tzmax);

  if ((tmin > tzmax) || (tzmin > tmax)) return false;

  if (tzmin > tmin) tmin = tzmin;
  if (tzmax < tmax) tmax = tzmax;
  if (tmin < 0) return false;

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

}  // namespace SimplePlanner
