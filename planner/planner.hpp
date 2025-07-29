#include "action.hpp"
#include "argument_parser.hpp"
#include "message.hpp"
#include "publisher.hpp"
#include "subscriber.hpp"
#include "utils.hpp"
#include "vertex.hpp"
#include <Eigen/Dense>
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
  // cloudpoint
  std::shared_ptr<Core::Subscriber<PointCloud>> _cloud_sub;
  std::shared_ptr<Core::Subscriber<Position>> _goal_sub;
  std::shared_ptr<Core::Subscriber<Odometry>> _odometry_sub;
  std::shared_ptr<Core::Publisher<MarkerArray>> _octree_pub;
  Odometry::Reader _last_odometry;
  pcl::io::OctreePointCloudCompression<pcl::PointXYZ> *_point_cloud_decoder;

  // algorithm
  std::shared_ptr<SimplePlanner::PCLOctree> _octree_search;
  std::thread _planner_thread;
  std::thread _publish_visualization_thread;
  SimplePlanner::Algorithm *_algorithm;
  pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;

  void cloud_point_cb(const Core::IncomingMessage<PointCloud> &);
  void goal_cb(const Core::IncomingMessage<Position> &);
  void odometry_cb(const Core::IncomingMessage<Odometry> &);
  void executing_request_cb(const SimplePlanner::PlanRequest &);
  void result_cb(SimplePlanner::PlanResponse);
  std::vector<pcl::PointXYZ> recover_octree_points();
  void publish_visualization();
  void publish_octree();

 public:
  Planner(Core::ArgumentParser, SimplePlanner::Algorithm *);
  ~Planner();
  void run();
  void stop();
};

#endif
