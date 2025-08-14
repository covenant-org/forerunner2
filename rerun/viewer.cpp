#include "message.hpp"
#include "rerun/archetypes/arrows3d.hpp"
#include "rerun/archetypes/asset3d.hpp"
#include "rerun/archetypes/boxes3d.hpp"
#include "rerun/archetypes/instance_poses3d.hpp"
#include "rerun/components/fill_mode.hpp"
#include "rerun/components/rotation_quat.hpp"
#include "utils.hpp"
#include "viewer.hpp"
#include <Eigen/src/Geometry/Quaternion.h>
#include <capnp_schemas/geometry_msgs.capnp.h>
#include <cmath>
#include <exception>
#include <iostream>
#include <pcl/impl/point_types.hpp>
#include <rerun.hpp>
#include <rerun/recording_stream.hpp>
#include <vector>

Demo::Demo(Core::ArgumentParser args) : Core::Vertex(args) {
  this->_point_cloud_decoder =
      new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>();
  this->_map_decoder =
      new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>();

  this->_rec = std::make_shared<rerun::RecordingStream>("Forerunner v2");
  this->_rec->spawn().exit_on_failure();

  this->_sub = this->create_subscriber<PointCloud>(
      "point_cloud",
      std::bind(&Demo::point_cloud_cb, this, std::placeholders::_1));
  this->_map_sub = this->create_subscriber<PointCloud>(
      "map", std::bind(&Demo::map_cloud_cb, this, std::placeholders::_1));
  this->_goal_sub = this->create_subscriber<Position>(
      "goal", std::bind(&Demo::goal_cb, this, std::placeholders::_1));
  this->_mic_sub = this->create_subscriber<StereoMic>(
      "mic", std::bind(&Demo::mic_cb, this, std::placeholders::_1));
  this->_odom_sub = this->create_subscriber<Odometry>(
      "odometry", std::bind(&Demo::odom_cb, this, std::placeholders::_1));
  this->_octree_sub = this->create_subscriber<MarkerArray>(
      "octree", std::bind(&Demo::octree_cb, this, std::placeholders::_1));
  this->_octree_layers_sub = this->create_subscriber<MarkerArray>(
      "octree_layers",
      std::bind(&Demo::octree_layers_cb, this, std::placeholders::_1));
  this->_planned_path_sub = this->create_subscriber<Path>(
      "planned_path",
      std::bind(&Demo::planned_path_cb, this, std::placeholders::_1));
}

rerun::Color Demo::distance_to_color(float distance) {
  // Create a color gradient based on distance
  // Blue (close) -> Green -> Yellow -> Red (far)

  // Normalize distance (adjust these values based on your scene)
  float min_dist = 0.5f;   // 0.5 meters
  float max_dist = 10.0f;  // 10 meters

  float normalized =
      std::clamp((distance - min_dist) / (max_dist - min_dist), 0.0f, 1.0f);

  if (normalized < 0.33f) {
    // Blue to Green
    float t = normalized / 0.33f;
    return rerun::Color(0, static_cast<uint8_t>(255 * t),
                        static_cast<uint8_t>(255 * (1 - t)));
  } else if (normalized < 0.66f) {
    // Green to Yellow
    float t = (normalized - 0.33f) / 0.33f;
    return rerun::Color(static_cast<uint8_t>(255 * t), 255, 0);
  } else {
    // Yellow to Red
    float t = (normalized - 0.66f) / 0.34f;
    return rerun::Color(255, static_cast<uint8_t>(255 * (1 - t)), 0);
  }
}

void Demo::render_path(const Core::IncomingMessage<Path> &msg,
                       const std::string &points_name,
                       const std::string &path_arrows) {
  auto poses = msg.content.getPoses();

  std::vector<rerun::Position3D> points;
  std::vector<rerun::Vec3D> vectors;
  std::vector<rerun::Position3D> origins;
  std::vector<rerun::Color> colors;

  // Extract path points
  for (auto pose : poses) {
    auto pos = pose.getPose().getPosition();
    points.emplace_back(pos.getX(), pos.getY(), -pos.getZ());
  }

  // Create direction vectors between consecutive points
  for (size_t i = 0; i < poses.size() - 1; ++i) {
    auto current_pos = poses[i].getPose().getPosition();
    auto next_pos = poses[i + 1].getPose().getPosition();

    // Calculate direction vector
    float dx = next_pos.getX() - current_pos.getX();
    float dy = next_pos.getY() - current_pos.getY();
    float dz = next_pos.getZ() - current_pos.getZ();

    // Normalize and scale the vector
    float length = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (length > 0.001f) {
      float scale =
          std::min(length * 0.8f, 0.3f);  // Scale arrow but cap max size
      dx = (dx / length) * scale;
      dy = (dy / length) * scale;
      dz = (dz / length) * scale;

      origins.emplace_back(current_pos.getX(), current_pos.getY(),
                           -current_pos.getZ());
      vectors.emplace_back(dx, dy, -dz);

      // Color gradient from start (blue) to end (red)
      float t = static_cast<float>(i) / (poses.size() - 1);
      colors.emplace_back(static_cast<uint8_t>(255 * t),       // Red increases
                          0,                                   // No green
                          static_cast<uint8_t>(255 * (1 - t))  // Blue decreases
      );
    }
  }

  // Log the path as points
  this->_rec->log(
      points_name,
      rerun::Points3D(points).with_colors(colors).with_radii({0.05f}));

  // Log direction arrows
  if (!vectors.empty()) {
    this->_rec->log(path_arrows, rerun::Arrows3D::from_vectors(vectors)
                                     .with_origins(origins)
                                     .with_colors(colors)
                                     .with_radii({0.02f}));
  }
}

void Demo::planned_path_cb(const Core::IncomingMessage<Path> &msg) {
  this->_logger.info("planned_path_cb was called");
  this->render_path(msg, "path/points", "path/arrows");
}

void Demo::odom_cb(const Core::IncomingMessage<Odometry> &msg) {
  auto content = msg.content;
  auto q = msg.content.getQ();
  auto position = content.getPosition();
  this->_rec->log("world/drone",
                  rerun::InstancePoses3D()
                      .with_translations({{position.getX(), position.getY(),
                                           -position.getZ()}})
                      .with_quaternions({rerun::components::RotationQuat(
                          {q.getX(), q.getY(), q.getZ(), q.getW()})}));
}

void Demo::goal_cb(const Core::IncomingMessage<Position> &msg) {
  auto content = msg.content;
  this->_rec->log("goal/position",
                  rerun::Boxes3D::from_centers_and_sizes(
                      {{content.getX(), content.getY(), -content.getZ()}},
                      {{0.3, 0.3, 0.3}}));
  this->_rec->log("goal/coords_x",
                  rerun::Scalars(static_cast<double>(content.getX())));
  this->_rec->log("goal/coords_y",
                  rerun::Scalars(static_cast<double>(content.getY())));
  this->_rec->log("goal/coords_z",
                  rerun::Scalars(static_cast<double>(content.getZ())));
}

void Demo::octree_cb(const Core::IncomingMessage<MarkerArray> &msg) {
  std::vector<rerun::components::PoseTranslation3D> centers;
  std::vector<rerun::HalfSize3D> sizes;
  std::vector<rerun::Color> colors;

  auto markers = msg.content.getMarkers();
  for (auto marker : markers) {
    auto position = marker.getPose().getPosition();
    auto color = marker.getColor();
    auto scale = marker.getScale();

    centers.emplace_back(position.getX(), position.getY(), position.getZ());
    sizes.emplace_back(scale.getX() / 2, scale.getY() / 2, scale.getZ() / 2);
    colors.emplace_back(color.getR() * 255, color.getG() * 255,
                        color.getB() * 255, color.getA() * 255);
  }
  this->_rec->log("octree/markers",
                  rerun::Boxes3D::from_centers_and_half_sizes(centers, sizes)
                      .with_quaternions({
                          rerun::Quaternion::IDENTITY,
                      })
                      .with_fill_mode(rerun::components::FillMode::Solid)
                      .with_colors(colors));

  // Log statistics
  this->_rec->log("stats/octree_count",
                  rerun::Scalars(static_cast<double>(markers.size())));
  this->_rec->log(
      "stats/octree_dimensions",
      rerun::TextLog("Octree markers: " + std::to_string(markers.size())));
}

void Demo::octree_layers_cb(const Core::IncomingMessage<MarkerArray> &msg) {
  std::vector<rerun::components::PoseTranslation3D> centers;
  std::vector<rerun::HalfSize3D> sizes;
  std::vector<rerun::Color> colors;

  auto markers = msg.content.getMarkers();
  for (auto marker : markers) {
    auto position = marker.getPose().getPosition();
    auto color = marker.getColor();
    auto scale = marker.getScale();

    centers.emplace_back(position.getX(), position.getY(), position.getZ());
    sizes.emplace_back(scale.getX() / 2, scale.getY() / 2, scale.getZ() / 2);
    colors.emplace_back(color.getR() * 255, color.getG() * 255,
                        color.getB() * 255, color.getA() * 255);
  }
  this->_rec->log("octree_layers/markers",
                  rerun::Boxes3D::from_centers_and_half_sizes(centers, sizes)
                      .with_quaternions({
                          rerun::Quaternion::IDENTITY,
                      })
                      .with_fill_mode(rerun::components::FillMode::Solid)
                      .with_colors(colors));

  // Log statistics
  // this->_rec->log("stats/octree_layers_count",
  //                 rerun::Scalars(static_cast<double>(markers.size())));
  this->_rec->log(
      "stats/octree_layers_dimensions",
      rerun::TextLog("Octree markers: " + std::to_string(markers.size())));
}

void Demo::mic_cb(const Core::IncomingMessage<StereoMic> &msg) {
  this->_rec->log("mic/left",
                  rerun::Scalars(static_cast<double>(msg.content.getLeft())));
  this->_rec->log("mic/right",
                  rerun::Scalars(static_cast<double>(msg.content.getRight())));
}

void Demo::point_cloud_cb(const Core::IncomingMessage<PointCloud> &msg) {
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
    _logger.warn("Error while decoding cloudpoint: %s", e.what());
    return;
  }

  size_t num_points = cloud->points.size();
  if (num_points == 0) return;

  std::vector<rerun::Position3D> positions;
  std::vector<rerun::Color> colors;

  positions.reserve(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    auto point = cloud->points[i];
    float x = point.x;
    float y = point.y;
    float z = point.z;
    positions.emplace_back(x, y, z);

    // Color based on distance for better visualization
    float distance = sqrt(x * x + y * y + z * z);
    auto color = distance_to_color(distance);
    colors.push_back(color);
  }
  // Log to Rerun
  this->_rec->log("world/camera/depth/points",
                  rerun::Points3D(positions).with_colors(colors));

  // Log statistics
  this->_rec->log("stats/camera/depth/point_count",
                  rerun::Scalars(static_cast<double>(positions.size())));
  this->_rec->log("stats/camera/depth/total_received",
                  rerun::Scalars(static_cast<double>(num_points)));
  this->_rec->log("stats/camera/depth/image_dimensions",
                  rerun::TextLog("Dimensions: " + std::to_string(width) + "x" +
                                 std::to_string(height)));
}

void Demo::map_cloud_cb(const Core::IncomingMessage<PointCloud> &msg) {
  auto data_reader = msg.content.getData();
  auto width = msg.content.getWidth();
  auto height = msg.content.getHeight();

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGBA>());
  std::stringstream buffer(
      std::string((char *)data_reader.begin(), data_reader.size()));
  try {
    _map_decoder->decodePointCloud(buffer, cloud);
  } catch (const std::exception &e) {
    _logger.warn("Error while decoding map: %s", e.what());
    return;
  }

  size_t num_points = cloud->points.size();
  _logger.debug("Decoded point cloud with %d points", num_points);
  if (num_points == 0) return;

  std::vector<rerun::Position3D> positions;
  std::vector<rerun::Color> colors;

  positions.reserve(num_points);
  colors.reserve(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    size_t idx = i * 4;
    auto point = cloud->points[i];
    float x = point.x;
    float y = point.y;
    float z = point.z;
    positions.emplace_back(x, y, z);
    colors.emplace_back(rerun::Color(point.r, point.g, point.b));
  }
  // Log to Rerun
  this->_rec->log("world/map/points",
                  rerun::Points3D(positions).with_colors(colors));

  // Log statistics
  this->_rec->log("stats/map/point_count",
                  rerun::Scalars(static_cast<double>(positions.size())));
  this->_rec->log("stats/map/total_received",
                  rerun::Scalars(static_cast<double>(num_points)));
  this->_rec->log("stats/map/image_dimensions",
                  rerun::TextLog("Dimensions: " + std::to_string(width) + "x" +
                                 std::to_string(height)));
}

void Demo::run() {
  this->_logger.info("Running");
  auto file_path = this->get_argument("--drone-model");
  this->_logger.debug("Drone model path: %s", file_path.c_str());
  this->_rec->log_static(
      "world", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);  // Set an up-axis
  this->_rec->log("world/drone",
                  rerun::Asset3D::from_file_path(file_path)
                      .value_or_throw());  // Set an up-axis
  while (true) sleep(1);
}

int main(int argc, char **argv) {
  Core::BaseArgumentParser args(argc, argv);
  auto root = Core::find_root();
  if (root.empty()) {
    root = ".";
  }
  if (root[root.size() - 1] != '/') {
    root.push_back('/');
  }
  std::string default_model_path = root + "rerun/assets/X500.glb";
  std::cout << default_model_path << std::endl;
  args.add_argument("--drone-model")
      .default_value(default_model_path)
      .help("stl file to use for rendering the drone");

  auto demo = Demo(args);
  demo.run();
  return 0;
}
