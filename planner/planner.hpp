#include "action.hpp"
#include "message.hpp"
#include "publisher.hpp"
#include "subscriber.hpp"
#include "utils.hpp"
#include "vertex.hpp"
#include <Eigen/Dense>
#include <capnp_schemas/mavlink.capnp.h>
#include <capnp_schemas/zed.capnp.h>
#include <capnp_schemas/planner.capnp.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <thread>

#ifndef PLANNER_HPP
#define PLANNER_HPP

class Planner : Core::Vertex {
 private:
  // cloudpoint
  std::shared_ptr<Core::Subscriber<PointCloud>> _cloud_sub;
  std::shared_ptr<Core::Subscriber<Position>> _goal_sub;
  pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> *_point_cloud_decoder;

  // algorithm
  std::thread planner_thread;
  std::shared_ptr<SimplePlanner::Algorithm> _algorithm;

 public:
  Planner(int, char **, std::shared_ptr<SimplePlanner::Algorithm> algorithm);
  void cloud_point_cb(const Core::IncomingMessage<PointCloud> &);
  void goal_cb(const Core::IncomingMessage<Position> &);

  void executing_request_cb(const PlanRequest &);
  void result_cb(PlanResponse);

  void run();
};

#endif
