#include "argument_parser.hpp"
#include "message.hpp"
#include "planner.hpp"
#include "theta-star.hpp"
#include "utils.hpp"
#include <Eigen/src/Geometry/Transform.h>
#include <capnp_schemas/controller.capnp.h>
#include <capnp_schemas/generics.capnp.h>
#include <memory>
#include <pcl/common/transforms.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/impl/point_types.hpp>

Planner::Planner(Core::ArgumentParser parser,
                 SimplePlanner::Algorithm *algorithm)
    : Core::Vertex(parser) {
  this->_point_cloud_decoder =
      new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>();

  this->_cloud_sub = this->create_subscriber<PointCloud>(
      "point_cloud",
      std::bind(&Planner::cloud_point_cb, this, std::placeholders::_1));

  this->_goal_sub = this->create_subscriber<Position>(
      "input_goal", std::bind(&Planner::goal_cb, this, std::placeholders::_1));

  this->_odometry_sub = this->create_subscriber<Odometry>(
      "input_goal",
      std::bind(&Planner::odometry_cb, this, std::placeholders::_1));

  SimplePlanner::AlgorithmConfig config;
  config.resolution = this->get_argument<float>("--resolution");
  config.min_distance = this->get_argument<float>("--min-distance");
  config.max_distance = this->get_argument<float>("--max-distance");
  config.safe_distance = this->get_argument<float>("--safe-distance");
  config.preferred_distance = this->get_argument<float>("--preferred-distance");

  this->_algorithm = algorithm;
  this->_algorithm->init(config, std::make_unique<Core::Logger>());
  this->_planner_thread = std::thread([&] {
    this->_algorithm->run(
        std::bind(&Planner::executing_request_cb, this, std::placeholders::_1),
        std::bind(&Planner::result_cb, this, std::placeholders::_1));
  });
}

/*
 * Called when a request was dequeued and is being executed
 * */
void Planner::executing_request_cb(const SimplePlanner::PlanRequest &request) {
  if (request.type == SimplePlanner::RequestType::START) {
    return;
  }

  // RequestMetadata *metadata = (RequestMetadata *)request.metadata;
  // std::shared_ptr<ReplanGoalHandle> goal_handle = metadata->goal_handle;
  // if (!goal_handle->is_executing()) {
  //   goal_handle->execute();
  // }
}

void Planner::cloud_point_cb(const Core::IncomingMessage<PointCloud> &msg) {
  this->_logger.info("cloud point received");
  auto data_reader = msg.content.getData();
  auto width = msg.content.getWidth();
  auto height = msg.content.getHeight();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  std::stringstream buffer(
      std::string((char *)data_reader.begin(), data_reader.size()));
  try {
    _point_cloud_decoder->decodePointCloud(buffer, cloud);
  } catch (const std::exception &e) {
    this->_logger.error("error while decoding: %s", e.what());
    return;
  }

  if (cloud->points.size() == 0) {
    this->_logger.info("empty cloud");
    this->_algorithm->update_octree(std::vector<pcl::PointXYZ>());
    return;
  }

  // Maybe implement a buffer to be able to extract at a specific time
  // also include the header in the mavlink pub
  Eigen::Affine3d eigen_transform(Eigen::Isometry3d(
      Eigen::Translation3d(_last_odometry.getPosition().getX(),
                           _last_odometry.getPosition().getY(),
                           _last_odometry.getPosition().getZ()) *
      Eigen::Quaterniond(
          _last_odometry.getQ().getW(), _last_odometry.getQ().getX(),
          _last_odometry.getQ().getY(), _last_odometry.getQ().getZ())));
  pcl::transformPointCloud(*cloud, *cloud, eigen_transform);

  if (cloud->points.size() == 0) {
    this->_logger.info("empty cloud");
    this->_algorithm->update_octree(std::vector<pcl::PointXYZ>());
    return;
  }

  this->_octree_search = std::make_shared<SimplePlanner::PCLOctree>(
      this->get_argument<float>("--resolution"));
  this->_octree_search->setInputCloud(cloud);
  this->_octree_search->addPointsFromInputCloud();

  if (this->_octree_search->getLeafCount() == 0) {
    this->_logger.info("empty octree");
    this->_algorithm->update_octree(std::vector<pcl::PointXYZ>());
    return;
  }

  this->_cloud = cloud;
  this->_algorithm->update_octree(this->recover_octree_points());
}

std::vector<pcl::PointXYZ> Planner::recover_octree_points() {
  std::vector<int> pointIdxSearch;
  std::vector<float> pointSquaredDistance;
  float max_distance = this->get_argument<float>("--max-distance");
  pcl::PointXYZ searchPoint(0, 0, 0);
  this->_octree_search->radiusSearch(searchPoint, max_distance, pointIdxSearch,
                                     pointSquaredDistance);
  if (pointIdxSearch.size() == 0) {
    return std::vector<pcl::PointXYZ>();
  }
  std::vector<pcl::PointXYZ> octree_points;
  octree_points.reserve(pointIdxSearch.size());
  for (size_t i = 0; i < pointIdxSearch.size(); i++) {
    octree_points.push_back(this->_cloud->points[pointIdxSearch[i]]);
  }
  return octree_points;
}

void Planner::goal_cb(const Core::IncomingMessage<Position> &msg) {
  this->_logger.info("goal received");
  auto content = msg.content;
  Eigen::Vector3d goal(content.getX(), content.getY(), content.getZ());
}

void Planner::odometry_cb(const Core::IncomingMessage<Odometry> &msg) {
  _last_odometry = std::move(msg.content);
}

void Planner::run() {
  while (true) {
  }
}

int main(int argc, char **argv) {
  Core::BaseArgumentParser parser(argc, argv);
  parser.add_argument("--resolution")
      .default_value(0.5)
      .help("spacing between points (number of overall points generated)")
      .scan<'f', float>();
  parser.add_argument("--min-distance")
      .default_value(0.5)
      .help("min distance to account from the cloud point")
      .scan<'f', float>();
  parser.add_argument("--safe-distance")
      .default_value(1.0)
      .help("min distance from the drone to any other object around")
      .scan<'f', float>();
  parser.add_argument("--preferred-distance")
      .default_value(3.0)
      .help("Ideal distance from the drone to any other object around")
      .scan<'f', float>();
  parser.add_argument("--max-distance")
      .default_value(5.0)
      .help("max distance to account from the cloud point")
      .scan<'f', float>();

  SimplePlanner::ThetaStar algorithm;
  std::shared_ptr<Planner> planner =
      std::make_shared<Planner>(parser, &algorithm);
  planner->run();
  return 0;
}
