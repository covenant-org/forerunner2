#ifndef UTILS_HPP
#define UTILS_HPP

#include "logger.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <capnp_schemas/geometry_msgs.capnp.h>
#include <capnp_schemas/nav_msgs.capnp.h>
#include <capnp_schemas/planner.capnp.h>
#include <capnp_schemas/visualization_msgs.capnp.h>
#include <cmath>
#include <optional>
#include <pcl/octree/octree_search.h>
#include <pcl/point_types.h>

namespace SimplePlanner {
// using ReplanAction = planner_interfaces::action::Replan;
// using ReplanGoalHandle = rclcpp_action::ServerGoalHandle<ReplanAction>;
using Box = std::pair<Eigen::Vector3d, Eigen::Vector3d>;
using PCLOctree = pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>;

class AlgorithmConfig {
 public:
  double max_distance;
  double resolution;
  double min_distance;
  double safe_distance;
  double preferred_distance;
};

class PathNode {
 public:
  PathNode(double x, double y, double z) : coords(x, y, z), cost(0) {}
  PathNode(Eigen::Vector3d coords) : coords(coords), cost(0) {}
  void clear() {
    cost = 0;
    heuristic = 0;
    score = 0;
    previous_position = nullptr;
    isObstacle = false;
  }
  ~PathNode(){};

  bool operator==(const PathNode &a) { return coords == a.coords; }

  Eigen::Vector3d coords;
  double cost;
  double heuristic;
  double score;
  size_t index;
  size_t internal_layer_index;
  size_t layer;
  std::vector<std::pair<std::shared_ptr<PathNode>, double>> children;
  std::shared_ptr<PathNode> previous_position;
  bool isObstacle;
};
using PathNodeVector = std::vector<std::shared_ptr<PathNode>>;

enum RequestType { START, REPLAN };

class ReplanRequest {
 public:
  size_t path_id;
  std::vector<Pose::Reader> path;
  size_t current_index;
};

class PlanRequest {
 public:
  RequestType type;
  Eigen::Vector3d goal;
  void *metadata;
  std::optional<ReplanRequest> body;
};

class PlanResponse {
 public:
  size_t path_id;
  PlanRequest request;
  PathNodeVector path;
};

class Algorithm {
 public:
  virtual ~Algorithm() = default;
  virtual void init(AlgorithmConfig, std::unique_ptr<Core::Logger>) = 0;
  virtual void update_config(AlgorithmConfig) = 0;
  virtual void update_obstacles(std::vector<Box>) = 0;
  virtual void update_octree(std::vector<pcl::PointXYZ>) = 0;
  virtual void enqueue(PlanRequest) = 0;
  virtual void run(std::function<void(const PlanRequest &)>,
                   std::function<void(PlanResponse)>) = 0;
  virtual void stop() = 0;
  virtual Path::Reader get_current_trajectory() = 0;
  virtual void update_current_trajectory(Path::Reader) = 0;
  virtual std::vector<std::vector<std::shared_ptr<PathNode>>> get_layers() = 0;
};

void pathToMsg(const std::vector<std::shared_ptr<PathNode>> &,
               Path::Builder &, Eigen::Vector3d &, Eigen::Affine3d &);
//
// void transform_path(Path::Reader &path, const tf2::Transform &transform,
//                     std::string frame_id, rclcpp::Time stamp);

void create_marker(Marker::Builder &marker, const pcl::PointXYZ &point,
                   float scale = 0.3);

bool eq(const PathNode &a, const PathNode &b);
bool eq(const PathNode &a, const std::vector<double> &b);
bool eq(const pcl::PointXYZ &a, const pcl::PointXYZ &b);
bool eq(const pcl::PointXYZ &a, const Eigen::Vector3d &b);

// tf2::Transform fromMsgToTF(TransformStamped &t);
bool isEqualDouble(double a, double b);

bool isInsideBox(const Eigen::Vector3d &point, const Eigen::Vector3d &min,
                 const Eigen::Vector3d &max);

class Ray {
 public:
  ~Ray(){};
  Ray(Eigen::Vector3d origin, Eigen::Vector3d dir)
      : origin(origin), dir(dir), inv_dir(1 / dir(0), 1 / dir(1), 1 / dir(2)){};

  Eigen::Vector3d origin;
  Eigen::Vector3d dir;
  Eigen::Vector3d inv_dir;
};

bool rayIntersects(const Ray &ray, const Eigen::Vector3d &min,
                   const Eigen::Vector3d &max);

bool boxIntersects(Eigen::Vector3d minA, Eigen::Vector3d maxA,
                   Eigen::Vector3d minB, Eigen::Vector3d maxB);

double distanceToBox(Eigen::Vector3d point, Eigen::Vector3d min,
                     Eigen::Vector3d max);

}  // namespace SimplePlanner

#endif
