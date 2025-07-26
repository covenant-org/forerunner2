#include "demo.hpp"
#include "message.hpp"
#include "rerun/archetypes/arrows3d.hpp"
#include "rerun/archetypes/boxes3d.hpp"
#include <array>
#include <exception>
#include <iostream>
#include <pcl/impl/point_types.hpp>
#include <rerun.hpp>
#include <rerun/recording_stream.hpp>
#include <vector>

Demo::Demo(Core::ArgumentParser args) : Core::Vertex(args) {
  this->_point_cloud_decoder =
      new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>();
  this->_rec = std::make_shared<rerun::RecordingStream>("rerun_demo");
  this->_rec->spawn().exit_on_failure();

  this->_sub = this->create_subscriber<PointCloud>(
      "point_cloud",
      std::bind(&Demo::point_cloud_cb, this, std::placeholders::_1));
  this->_mic_sub = this->create_subscriber<StereoMic>(
      "mic", std::bind(&Demo::mic_cb, this, std::placeholders::_1));
  this->_odom_sub = this->create_subscriber<Odometry>(
      "odometry", std::bind(&Demo::odom_cb, this, std::placeholders::_1));
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

void Demo::odom_cb(const Core::IncomingMessage<Odometry> &msg) {
  auto content = msg.content;
  auto position = content.getPosition();
  auto velocity = content.getVelocity();
  this->_rec->log("drone/position",
                  rerun::Boxes3D::from_centers_and_sizes(
                      {{position.getX(), position.getY(), position.getZ()}},
                      {{0.3, 0.3, 0.3}}));
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

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  std::stringstream buffer(
      std::string((char *)data_reader.begin(), data_reader.size()));
  try {
    _point_cloud_decoder->decodePointCloud(buffer, cloud);
  } catch (const std::exception &e) {
    std::cerr << "Error while decoding: " << e.what() << std::endl;
    return;
  }

  size_t num_points = cloud->points.size();

  std::vector<rerun::Position3D> positions;
  std::vector<rerun::Color> colors;

  positions.reserve(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    size_t idx = i * 4;
    auto point = cloud->points[i];
    float x = point.x;
    float y = point.y;
    float z = point.z;
    // float_data[idx + 3] is typically confidence or unused

    // Filter out invalid points (NaN or infinite values)
    positions.emplace_back(x, y, z);

    // Color based on distance for better visualization
    float distance = sqrt(x * x + y * y + z * z);
    auto color = distance_to_color(distance);
    colors.push_back(color);
  }
  // Log to Rerun
  if (!positions.empty()) {
    this->_rec->log("pointcloud/points",
                    rerun::Points3D(positions).with_colors(colors));

    // Log statistics
    this->_rec->log("stats/point_count",
                    rerun::Scalars(static_cast<double>(positions.size())));
    this->_rec->log("stats/total_received",
                    rerun::Scalars(static_cast<double>(num_points)));
    this->_rec->log("stats/image_dimensions",
                    rerun::TextLog("Dimensions: " + std::to_string(width) +
                                   "x" + std::to_string(height)));
  }
}

void Demo::run() {
  this->_logger.info("Running");
  while (true) sleep(1);
}

int main(int argc, char **argv) {
  Core::BaseArgumentParser args(argc, argv);
  auto demo = Demo(args);
  demo.run();
  return 0;
}
