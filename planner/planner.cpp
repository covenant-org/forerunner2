#include "message.hpp"
#include "planner.hpp"
#include "utils.hpp"
#include <capnp_schemas/controller.capnp.h>
#include <capnp_schemas/generics.capnp.h>
#include <memory>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/impl/point_types.hpp>

Planner::Planner(int argc, char **argv,
                 std::shared_ptr<SimplePlanner::Algorithm> algorithm)
    : Core::Vertex(argc, argv) {
  this->_point_cloud_decoder =
      new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>();

  this->_cloud_sub = this->create_subscriber<PointCloud>(
      "point_cloud",
      std::bind(&Planner::cloud_point_cb, this, std::placeholders::_1));

  this->_goal_sub = this->create_subscriber<Position>(
      "input_goal", std::bind(&Planner::goal_cb, this, std::placeholders::_1));

  SimplePlanner::AlgorithmConfig config;
  // TODO: update with parsed args
  config.resolution = 0.5;
  config.min_distance = 0.5;
  config.safe_distance = 1.0;
  config.preferred_distance = 3.0;
  config.max_distance = 5.0;

  this->_algorithm = algorithm;
  this->_algorithm->init(config);
  this->planner_thread = std::thread([&] {
    this->_algorithm->run(
        std::bind(&Planner::executing_request_cb, this, std::placeholders::_1),
        std::bind(&Planner::result_cb, this, std::placeholders::_1), );
  });
}

/*
 * Called when a request was dequeued and is being executed
 * */
void Planner::executing_request_cb(const PlanRequest &request) {
  if (request.type == RequestType::START) {
    return;
  }
  RequestMetadata *metadata = (RequestMetadata *)request.metadata;
  std::shared_ptr<ReplanGoalHandle> goal_handle = metadata->goal_handle;
  if (!goal_handle->is_executing()) {
    goal_handle->execute();
  }
}

void Planner::cloud_point_cb(const Core::IncomingMessage<PointCloud> &msg) {
  auto data_reader = msg.content.getData();
  auto width = msg.content.getWidth();
  auto height = msg.content.getHeight();

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGBA>());
  std::stringstream buffer(
      std::string((char *)data_reader.begin(), data_reader.size()));
  try {
    _point_cloud_decoder->decodePointCloud(buffer, cloud);
  } catch (const std::exception &e) {
    this->_logger.error("error while decoding: %s", e.what());
    return;
  }
}

void Planner::goal_cb(const Core::IncomingMessage<Position> &msg) {
  this->_logger.info("goal received");
  auto content = msg.content;
  Eigen::Vector3d goal(content.getX(), content.getY(), content.getZ());
}

void Planner::run() {
  while (true) {
  }
}

int main(int argc, char **argv) {
  std::shared_ptr<Planner> planner = std::make_shared<Planner>(argc, argv);
  planner->run();
  return 0;
}
