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
    : Core::Vertex(parser), _algorithm(algorithm) {
  this->_point_cloud_decoder =
      new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>();

  this->_octree_pub = this->create_publisher<MarkerArray>("octree");

  this->_path_pub = this->create_publisher<Path>("planned_path");

  this->_cloud_sub = this->create_subscriber<PointCloud>(
      "point_cloud",
      std::bind(&Planner::cloud_point_cb, this, std::placeholders::_1));

  this->_goal_sub = this->create_subscriber<Position>(
      "input_goal", std::bind(&Planner::goal_cb, this, std::placeholders::_1));

  this->_odometry_sub = this->create_subscriber<Odometry>(
      "odometry",
      std::bind(&Planner::odometry_cb, this, std::placeholders::_1));

  SimplePlanner::AlgorithmConfig config;
  config.resolution = this->get_argument<double>("--resolution");
  config.min_distance = this->get_argument<double>("--min-distance");
  config.max_distance = this->get_argument<double>("--max-distance");
  config.safe_distance = this->get_argument<double>("--safe-distance");
  config.preferred_distance =
      this->get_argument<double>("--preferred-distance");

  this->_algorithm->init(config, std::make_unique<Core::Logger>());
  this->_planner_thread = std::thread([&] {
    this->_algorithm->run(
        std::bind(&Planner::executing_request_cb, this, std::placeholders::_1),
        std::bind(&Planner::result_cb, this, std::placeholders::_1));
  });

  this->_publish_visualization_thread =
      std::thread([&] { this->publish_visualization(); });

  this->_logger.info("Planner initialized");
}

Planner::~Planner() { this->stop(); }

void Planner::stop() {
  this->_algorithm->stop();
  if (this->_planner_thread.joinable()) {
    this->_planner_thread.join();
  }
  // TODO: properly stop the publisher
  delete this->_point_cloud_decoder;
}

void Planner::publish_visualization() {
  bool should_publish_octree = this->get_argument<bool>("--publish-octree");

  while (true) {
    if (should_publish_octree) {
      this->publish_octree();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
}

void Planner::publish_octree() {
  if (this->_octree_search == nullptr || this->_cloud == nullptr ||
      this->_octree_search->getLeafCount() == 0) {
    return;
  }

  std::vector<int> pointIdxSearch;
  std::vector<float> pointSquaredDistance;
  double radius = this->get_argument<double>("--max-distance");
  double resolution = this->get_argument<double>("--resolution");
  pcl::PointXYZ searchPoint(_drone_pose.position.x(), _drone_pose.position.y(),
                            _drone_pose.position.z());
  this->_octree_search->radiusSearch(searchPoint, radius, pointIdxSearch,
                                     pointSquaredDistance);

  if (pointIdxSearch.size() > 0) {
    auto msg = this->_octree_pub->new_msg();
    auto markers = msg.content.initMarkers(pointIdxSearch.size());
    for (size_t i = 0; i < pointIdxSearch.size(); i++) {
      auto marker = markers[i];
      auto point = (*this->_cloud)[pointIdxSearch[i]];
      SimplePlanner::create_marker(marker, point, resolution);
      marker.getColor().setR(0.0);
      marker.getColor().setG(0.0);
      marker.getColor().setB(1.0);
      marker.getColor().setA(1.0);
      marker.setNs("octree");
    }
    msg.publish();
  }
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
  this->_logger.debug("cloud point received");
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
      Eigen::Translation3d(_drone_pose.position.x(), _drone_pose.position.y(),
                           _drone_pose.position.z()) *
      Eigen::Quaterniond(
          _drone_pose.orientation.w(), _drone_pose.orientation.x(),
          _drone_pose.orientation.y(), _drone_pose.orientation.z())));
  pcl::transformPointCloud(*cloud, *cloud, eigen_transform);

  if (cloud->points.size() == 0) {
    this->_logger.info("empty cloud");
    this->_algorithm->update_octree(std::vector<pcl::PointXYZ>());
    return;
  }

  this->_octree_search = std::make_shared<SimplePlanner::PCLOctree>(
      this->get_argument<double>("--resolution"));
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
  double max_distance = this->get_argument<double>("--max-distance");
  pcl::PointXYZ searchPoint(_drone_pose.position.x(), _drone_pose.position.y(),
                            _drone_pose.position.z());
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
  Eigen::Vector3d current_goal(_goal_msg);

  _goal_msg.x() = msg.content.getX();
  _goal_msg.y() = msg.content.getY();
  _goal_msg.z() = msg.content.getZ();

  SimplePlanner::PlanRequest request;
  request.type = SimplePlanner::RequestType::START;
  request.goal << _goal_msg.x(), _goal_msg.y(), _goal_msg.z();

  this->_algorithm->enqueue(std::move(request));
}

void Planner::result_cb(SimplePlanner::PlanResponse response) {
  if (response.request.type == SimplePlanner::RequestType::START) {
    // If request is a start request just recover path and do transformations
    this->_logger.info("Received start request");
    this->_path_sequence = response.path_id;

    auto msg = this->_path_pub->new_msg();
    SimplePlanner::pathToMsg(response.path, msg.content, _goal_msg);
    msg.publish();
    return;
  }

  if (this->_path_sequence != response.path_id) {
    auto msg = this->_path_pub->new_msg();
    SimplePlanner::pathToMsg(response.path, msg.content, _goal_msg);
    this->_path_sequence = response.path_id;
    msg.publish();
  }
}

void Planner::odometry_cb(const Core::IncomingMessage<Odometry> &msg) {
  _drone_pose.position.x() = msg.content.getPosition().getX();
  _drone_pose.position.y() = msg.content.getPosition().getY();
  _drone_pose.position.z() = msg.content.getPosition().getZ();
  _drone_pose.orientation.x() = msg.content.getQ().getX();
  _drone_pose.orientation.y() = msg.content.getQ().getY();
  _drone_pose.orientation.z() = msg.content.getQ().getZ();
  _drone_pose.orientation.w() = msg.content.getQ().getW();

  this->_logger.debug("drone position: (%f, %f, %f)", _drone_pose.position.x(),
                      _drone_pose.position.y(), _drone_pose.position.z());
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
      .scan<'g', double>();
  parser.add_argument("--min-distance")
      .default_value(0.5)
      .help("min distance to account from the cloud point")
      .scan<'g', double>();
  parser.add_argument("--safe-distance")
      .default_value(1.0)
      .help("min distance from the drone to any other object around")
      .scan<'g', double>();
  parser.add_argument("--preferred-distance")
      .default_value(3.0)
      .help("Ideal distance from the drone to any other object around")
      .scan<'g', double>();
  parser.add_argument("--max-distance")
      .default_value(5.0)
      .help("max distance to account from the cloud point")
      .scan<'g', double>();
  parser.add_argument("--publish-octree")
      .help("publish the octree as a visualization marker")
      .flag();

  SimplePlanner::ThetaStar algorithm;
  std::shared_ptr<Planner> planner =
      std::make_shared<Planner>(parser, &algorithm);
  planner->run();
  return 0;
}
