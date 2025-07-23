#ifndef THETA_STAR_HPP
#define THETA_STAR_H

#include "kdtree.hpp"
#include "utils.hpp"
#include "planner/profiler.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/server.hpp>

namespace SimplePlanner {
#define INF 1e15
#define EPS 1e-6

class ThetaStar : public SimplePlanner::Algorithm {
private:
  SimplePlanner::Logger logger;
  AlgorithmConfig config;
  std::vector<pcl::PointXYZ> octree_points;
  std::vector<Box> obstacles;

  Eigen::Vector3d real_goal;
  pcl::PointXYZ local_goal;
  std::string original_cloud_frame_id;
  rclcpp::Time last_cloud_stamp;
  std::shared_ptr<PathNode> last_local_goal;
  size_t path_sequence;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  bool recived_goal;
  bool empty_cloud;

  std::mutex planning_mutex;
  std::thread planner_thread;
  std::mutex queue_mutex;
  std::condition_variable queue_cv;
  std::queue<PlanRequest> queue;

  std::vector<std::shared_ptr<PathNode>> last_path;
  nav_msgs::msg::Path current_trajectory;

  std::shared_ptr<PathNode> graph_head;
  tf2::Transform last_base_to_map;
  std::unique_ptr<Kdtree::KdTree> kdtree;
  Kdtree::KdNodeVector kdtree_nodes;
  std::vector<std::vector<std::shared_ptr<PathNode>>> layers;
  std::vector<std::shared_ptr<PathNode>> touched_nodes;

  bool planning;
  bool stop_thread;

  void generate_travel_graph();
  void add_octree_costs();
  void add_costs();
  void clear_nodes();
  PathNodeVector recover_path(const std::shared_ptr<PathNode> &);
  bool line_of_sight(std::shared_ptr<PathNode>, std::shared_ptr<PathNode>);
  PathNodeVector plan();
  bool ready_to_plan();
  void find_local_goal();
  void add_heuristics();
  void add_synthetic_costs();

public:
  void init(AlgorithmConfig, Logger);
  void run(std::function<void(const PlanRequest &)>,
           std::function<void(PlanResponse)>);
  void update_obstacles(std::vector<Box>);
  void update_octree(std::vector<pcl::PointXYZ>);
  void update_config(AlgorithmConfig);
  void enqueue(PlanRequest);
  nav_msgs::msg::Path get_current_trajectory();
  void update_current_trajectory(nav_msgs::msg::Path);
  std::vector<std::vector<std::shared_ptr<PathNode>>> get_layers();
  void stop();
  explicit ThetaStar();
  ~ThetaStar();
};

} // namespace SimplePlanner
#endif

