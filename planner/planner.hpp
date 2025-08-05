#include "action.hpp"
#include "argument_parser.hpp"
#include "message.hpp"
#include "publisher.hpp"
#include "subscriber.hpp"
#include "utils.hpp"
#include "vertex.hpp"
#include <Eigen/Dense>
#include <capnp_schemas/generics.capnp.h>
#include <capnp_schemas/mavlink.capnp.h>
#include <capnp_schemas/planner.capnp.h>
#include <capnp_schemas/zed.capnp.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/octree/octree_search.h>
#include <thread>

#ifndef PLANNER_HPP
#define PLANNER_HPP

class Planner : Core::Vertex {
 private:
  struct DronePose {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
  } _drone_pose;
  size_t _path_sequence = 0;
  // cloudpoint
  std::shared_ptr<Core::Subscriber<PointCloud>> _cloud_sub;
  std::shared_ptr<Core::ActionServer<Position, GenericResponse>>
      _goal_action_server;
  std::shared_ptr<Core::Subscriber<Odometry>> _odometry_sub;
  std::shared_ptr<Core::Publisher<MarkerArray>> _octree_pub;
  std::shared_ptr<Core::Publisher<MarkerArray>> _octree_layers_pub;
  std::shared_ptr<Core::Publisher<Path>> _path_pub;
  std::shared_ptr<Core::Publisher<Path>> _local_path_pub;
  std::shared_ptr<Core::ActionServer<ReplanRequest, GenericResponse>>
      _planner_server;
  Odometry::Reader _last_odometry;
  pcl::io::OctreePointCloudCompression<pcl::PointXYZ> *_point_cloud_decoder;

  // algorithm
  std::shared_ptr<SimplePlanner::PCLOctree> _octree_search;
  std::thread _planner_thread;
  std::thread _publish_visualization_thread;
  SimplePlanner::Algorithm *_algorithm;
  pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;
  Eigen::Vector3d _goal_msg;
  bool _received_goal = false;
  bool _planning = false;

  void goal_server_cb(const Core::IncomingMessage<Position> &,
                      GenericResponse::Builder &);
  void planner_server_cb(const Core::IncomingMessage<ReplanRequest> &,
                         GenericResponse::Builder &);
  void cloud_point_cb(const Core::IncomingMessage<PointCloud> &);
  void odometry_cb(const Core::IncomingMessage<Odometry> &);
  void executing_request_cb(const SimplePlanner::PlanRequest &);
  void result_cb(SimplePlanner::PlanResponse);
  void run_planner(ReplanRequest::Start::Reader &);
  std::vector<pcl::PointXYZ> recover_octree_points();
  void publish_visualization();
  void publish_octree();
  void publish_layers();

 public:
  Planner(Core::ArgumentParser, SimplePlanner::Algorithm *);
  ~Planner();
  void run();
  void stop();
};

#endif
