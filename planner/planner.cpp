#include "argument_parser.hpp"
#include "message.hpp"
#include "planner.hpp"
#include "theta-star.hpp"
#include "utils.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Transform.h>
#include <capnp_schemas/controller.capnp.h>
#include <capnp_schemas/generics.capnp.h>
#include <capnp_schemas/planner.capnp.h>
#include <cmath>
#include <eigen3/Eigen/src/Core/Matrix.h>
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
  this->_octree_layers_pub =
      this->create_publisher<MarkerArray>("octree_layers");

  this->_path_pub = this->create_publisher<Path>("planned_path");
  this->_goal_pub = this->create_publisher<Position>("goal");

  this->_cloud_sub = this->create_subscriber<PointCloud>(
      "point_cloud",
      std::bind(&Planner::cloud_point_cb, this, std::placeholders::_1));

  this->_goal_action_server =
      this->create_action_server<Position, GenericResponse>(
          "input_goal",
          std::bind(&Planner::goal_server_cb, this, std::placeholders::_1,
                    std::placeholders::_2));

  this->_odometry_sub = this->create_subscriber<Odometry>(
      "odometry",
      std::bind(&Planner::odometry_cb, this, std::placeholders::_1));
  this->_planner_server =
      this->create_action_server<ReplanRequest, GenericResponse>(
          "planner", std::bind(&Planner::planner_server_cb, this,
                               std::placeholders::_1, std::placeholders::_2));

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

void Planner::planner_server_cb(const Core::IncomingMessage<ReplanRequest> &req,
                                GenericResponse::Builder &res) {
  this->_logger.info("got new request: %zu", req.content.which());
  switch (req.content.which()) {
    case ReplanRequest::START: {
      auto start = req.content.getStart();
      auto pos = start.getPose().getPose().getPosition();
      this->_logger.info("received goal request: %f, %f, %f", pos.getX(),
                         pos.getY(), pos.getZ());

      Eigen::Vector3d absolute_goal(_goal_msg.x(), _goal_msg.y(),
                                    _goal_msg.z());
      Eigen::Vector3d current_pose(pos.getX(), pos.getY(), pos.getZ());

      if ((current_pose - absolute_goal).norm() < 1.0) {
        this->_received_goal = false;
        res.setCode(422);
        res.setMessage("goal is too close");
      } else {
        this->run_planner(start);
        res.setCode(200);
        res.setMessage("ok");
      }
      break;
    }
  }
}

void Planner::run_planner(ReplanRequest::Start::Reader &msg) {
  SimplePlanner::AlgorithmConfig config;
  config.resolution = this->get_argument<double>("--resolution");
  config.min_distance = this->get_argument<double>("--min-distance");
  config.max_distance = this->get_argument<double>("--max-distance");
  config.safe_distance = this->get_argument<double>("--safe-distance");
  config.preferred_distance =
      this->get_argument<double>("--preferred-distance");

  Eigen::Affine3d drone_transform(Eigen::Isometry3d(
      Eigen::Translation3d(_drone_pose.position.x(), _drone_pose.position.y(),
                           _drone_pose.position.z()) *
      Eigen::Quaterniond(
          _drone_pose.orientation.w(), _drone_pose.orientation.x(),
          _drone_pose.orientation.y(), _drone_pose.orientation.z())));

  auto pos = msg.getPose().getPose().getPosition();
  SimplePlanner::PlanRequest request;
  request.type = SimplePlanner::RequestType::REPLAN;
  request.metadata = (void *)new RequestMetadata(drone_transform);

  Eigen::Vector3d goal(_goal_msg.x(), _goal_msg.y(), _goal_msg.z());
  // auto transformed_goal = drone_transform * goal;
  request.goal << goal.x(), goal.y(), goal.z();

  SimplePlanner::ReplanRequest replan;
  replan.current_index = msg.getCurrentPathIndex();
  replan.path_id = msg.getPathSequence();
  request.body = std::make_optional<SimplePlanner::ReplanRequest>(replan);
  ::capnp::MallocMessageBuilder message;
  for (const auto &pose :
       this->_algorithm->get_current_trajectory().getPoses()) {
    auto pos = pose.getPose().getPosition();
    Eigen::Vector3d point(pos.getX(), pos.getY(), pos.getZ());
    // auto transformed_point = drone_transform * point;
    auto transformed_point = point;

    Pose::Builder p = message.initRoot<Pose>();
    p.initPosition();
    p.getPosition().setX(transformed_point.x());
    p.getPosition().setY(transformed_point.y());
    p.getPosition().setZ(transformed_point.z());
    request.body.value().path.push_back(p.asReader());
  }
  this->_algorithm->enqueue(std::move(request));
}

void Planner::publish_visualization() {
  bool should_publish_octree = this->get_argument<bool>("--publish-octree");
  bool should_publish_layers = this->get_argument<bool>("--publish-layers");

  while (true) {
    if (should_publish_octree) {
      this->publish_octree();
    }
    if (should_publish_layers) {
      this->publish_layers();
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
  pcl::PointXYZ searchPoint(0, 0, 0);
  this->_octree_search->radiusSearch(searchPoint, radius, pointIdxSearch,
                                     pointSquaredDistance);

  if (pointIdxSearch.size() > 0) {
    auto msg = this->_octree_pub->new_msg();
    auto markers = msg.content.initMarkers(pointIdxSearch.size());
    for (size_t i = 0; i < pointIdxSearch.size(); i++) {
      auto marker = markers[i];
      auto point = (*this->_cloud)[pointIdxSearch[i]];
      SimplePlanner::create_marker(marker, point);
      marker.getColor().setR(0.8);
      marker.getColor().setG(0.0);
      marker.getColor().setB(0.0);
      marker.getColor().setA(0.75);
      marker.setNs("octree");
    }
    msg.publish();
  }
}

void Planner::publish_layers() {
  size_t index = 0;
  size_t total = 0;
  auto layers = this->_algorithm->get_layers();
  size_t layer_index = 0;
  for (auto &layer : layers) {
    total += layer.size();
  }
  auto msg = this->_octree_layers_pub->new_msg();
  auto markers = msg.content.initMarkers(total);
  for (auto &layer : layers) {
    for (auto &node : layer) {
      auto marker = markers[index];
      auto color = marker.getColor();
      SimplePlanner::create_marker(
          marker,
          pcl::PointXYZ(node->coords.x(), node->coords.y(), node->coords.z()));
      double distance = 1 / node->cost;
      if (!std::isnormal(distance) || node->cost <= EPS) {
        distance = INF;
      }

      color.setA(0.3);
      color.setR((1 - distance / 10.0) * (1 - distance / 10.0));
      color.setG(std::sqrt(std::sqrt(distance / 10.0)));
      color.setB(1.0);
      index++;
    }
    layer_index++;
  }
  msg.publish();
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
  // In previous implementation this was against the frame of the drone
  Eigen::Affine3d camera_to_drone = Eigen::Affine3d::Identity();
  // defined in the model from the PX4 repo for the x500_depth model.sdf
  camera_to_drone.translation() = Eigen::Vector3d(0.12, 0.03, 0.242);

  // Eigen::Affine3d drone_ned(Eigen::Isometry3d(
  //     Eigen::Translation3d(_drone_pose.position.x(), _drone_pose.position.y(),
  //                          _drone_pose.position.z()) *
  //     Eigen::Quaterniond(
  //         _drone_pose.orientation.w(), _drone_pose.orientation.x(),
  //         _drone_pose.orientation.y(), _drone_pose.orientation.z())));
  // Eigen::Affine3d camera_to_ned = drone_ned * camera_to_drone;
  pcl::transformPointCloud(*cloud, *cloud, camera_to_drone);

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

void Planner::goal_server_cb(const Core::IncomingMessage<Position> &msg,
                             GenericResponse::Builder &res) {
  this->_logger.info("goal received");
  auto content = msg.content;
  Eigen::Vector3d goal(content.getX(), content.getY(), content.getZ());
  Eigen::Vector3d current_goal(_goal_msg);

  _goal_msg.x() = msg.content.getX();
  _goal_msg.y() = msg.content.getY();
  _goal_msg.z() = msg.content.getZ();

  auto *drone_transform = new Eigen::Affine3d(
      Eigen::Translation3d(_drone_pose.position.x(), _drone_pose.position.y(),
                           _drone_pose.position.z()) *
      Eigen::Quaterniond(
          _drone_pose.orientation.w(), _drone_pose.orientation.x(),
          _drone_pose.orientation.y(), _drone_pose.orientation.z()));
  auto goal_msg = this->_goal_pub->new_msg();
  auto transformed_goal = *drone_transform * goal;
  goal_msg.content.setX(transformed_goal.x());
  goal_msg.content.setY(transformed_goal.y());
  goal_msg.content.setZ(transformed_goal.z());
  goal_msg.publish();

  SimplePlanner::PlanRequest request;
  request.type = SimplePlanner::RequestType::START;
  request.goal << goal.x(), goal.y(), goal.z();
  request.metadata = static_cast<void *>(drone_transform);

  this->_algorithm->enqueue(std::move(request));

  res.setCode(200);
  res.setMessage("ok");
}

void Planner::result_cb(SimplePlanner::PlanResponse response) {
  this->_logger.info("received result from planner");
  if (response.request.type == SimplePlanner::RequestType::START) {
    // If request is a start request just recover path and do transformations
    this->_logger.info("received start request");
    this->_path_sequence = response.path_id;

    Eigen::Affine3d *saved_transform =
        static_cast<Eigen::Affine3d *>(response.request.metadata);
    auto msg = this->_path_pub->new_msg();
    // convert to NED system at the time the plan was requested (t)
    SimplePlanner::pathToMsg(response.path, msg.content, _goal_msg,
                             *saved_transform);
    this->_algorithm->update_current_trajectory(msg.content.asReader());

    auto current_transform = Eigen::Affine3d(Eigen::Isometry3d(
        Eigen::Translation3d(_drone_pose.position.x(), _drone_pose.position.y(),
                             _drone_pose.position.z()) *
        Eigen::Quaterniond(
            _drone_pose.orientation.w(), _drone_pose.orientation.x(),
            _drone_pose.orientation.y(), _drone_pose.orientation.z())));
    auto relative_transform = saved_transform->inverse() * current_transform;

    // update with latest values
    SimplePlanner::transform_path(msg.content, relative_transform);
    msg.publish();
    delete saved_transform;
    this->_logger.info("published new message with %zu nodes",
                       msg.content.getPoses().size());
    return;
  }

  RequestMetadata *metadata = (RequestMetadata *)response.request.metadata;
  Eigen::Affine3d saved_transform = metadata->transform;
  auto current_transform = Eigen::Affine3d(Eigen::Isometry3d(
      Eigen::Translation3d(_drone_pose.position.x(), _drone_pose.position.y(),
                           _drone_pose.position.z()) *
      Eigen::Quaterniond(
          _drone_pose.orientation.w(), _drone_pose.orientation.x(),
          _drone_pose.orientation.y(), _drone_pose.orientation.z())));

  auto msg = this->_path_pub->new_msg();
  if (this->_path_sequence != response.path_id) {
    SimplePlanner::pathToMsg(response.path, msg.content, _goal_msg,
                             saved_transform);
    this->_path_sequence = response.path_id;
  } else {
    auto poses = this->_algorithm->get_current_trajectory().getPoses();
    msg.content.setPoses(poses);
  }

  auto relative_transform = current_transform * saved_transform.inverse();
  SimplePlanner::transform_path(msg.content, relative_transform);
  this->_path_sequence = response.path_id;
  msg.publish();
  this->_logger.info("published new message with %zu nodes (replan)",
                     msg.content.getPoses().size());
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
  parser.add_argument("--publish-layers")
      .help("publish the algorithm layers")
      .flag();

  SimplePlanner::ThetaStar algorithm;
  std::shared_ptr<Planner> planner =
      std::make_shared<Planner>(parser, &algorithm);
  planner->run();
  return 0;
}
