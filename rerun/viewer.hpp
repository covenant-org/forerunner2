#ifndef VIEWER_HPP
#define VIEWER_HPP

#include "argument_parser.hpp"
#include "message.hpp"
#include "rerun/archetypes/asset3d.hpp"
#include "subscriber.hpp"
#include "vertex.hpp"
#include <capnp_schemas/detection_messages.capnp.h>
#include <capnp_schemas/mavlink.capnp.h>
#include <capnp_schemas/nav_msgs.capnp.h>
#include <capnp_schemas/sensors.capnp.h>
#include <capnp_schemas/visualization_msgs.capnp.h>
#include <capnp_schemas/zed.capnp.h>
#include <capnp_schemas/geometry_msgs.capnp.h>
#include <memory>
#include <optional>
#include <string>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <rerun.hpp>
#include <rerun/recording_stream.hpp>

class Viewer : public Core::Vertex {
private:
  Core::ArgumentParser _args;
  Core::Logger _logger;
  
  std::shared_ptr<Core::Subscriber<PointCloud>> _sub;
  std::shared_ptr<Core::Subscriber<PointCloud>> _map_sub;
  std::shared_ptr<Core::Subscriber<PointCloudChunk>> _map_chunk_sub;
  std::shared_ptr<Core::Subscriber<Point>> _goal_sub;
  std::shared_ptr<Core::Subscriber<StereoMic>> _mic_sub;
//   std::shared_ptr<Core::Subscriber<Odometry>> _odom_sub;
  std::shared_ptr<Core::Subscriber<MarkerArray>> _octree_sub;
  std::shared_ptr<Core::Subscriber<MarkerArray>> _octree_layers_sub;
  std::shared_ptr<Core::Subscriber<Path>> _planned_path_sub;
  std::shared_ptr<Core::Subscriber<Path>> _local_planned_path_sub;
  std::shared_ptr<Core::Subscriber<Point>> _person_reco_path_sub;
  std::shared_ptr<Core::Subscriber<DetectionImage>> _detection_images_sub;
  std::shared_ptr<Core::Subscriber<Image>> _zed_image_sub;
//   std::shared_ptr<Core::Subscriber<Route>> _route_path;
  std::shared_ptr<rerun::RecordingStream> _rec;
  pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> *_point_cloud_decoder;
  pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>
      *_map_point_cloud_decoder;
  rerun::archetypes::Asset3D _drone_model;
  rerun::Quaternion _drone_quat;
  std::string build_recording_path(
      const std::string &user_input,
      const std::optional<std::string> &recording_type = std::nullopt);

 public:
  const Core::ArgumentParser& get_args() const { return _args; }
  Core::Logger* get_logger() { return &_logger; }

  template <typename T = bool>
  T public_get_argument(std::string_view arg_name) const {
    return this->get_argument<T>(arg_name);
  }

  template <typename T, typename K>
  std::shared_ptr<Core::ActionClient<T, K>> public_create_action_client(const std::string& topic) {
    return this->create_action_client<T, K>(topic);
  }

  template <typename T>
  std::shared_ptr<Core::Subscriber<T>> public_create_subscriber(const std::string& topic,
    std::function<void(const Core::IncomingMessage<T>&)> cb) {
    return this->create_subscriber<T>(topic, cb);
  }
  void goal_cb(const Core::IncomingMessage<Point> &);
  void point_cloud_cb(const Core::IncomingMessage<PointCloud> &);
  void map_cloud_cb(const Core::IncomingMessage<PointCloud> &);
  void mic_cb(const Core::IncomingMessage<StereoMic> &);
//   void odom_cb(const Core::IncomingMessage<Odometry> &);
  void octree_cb(const Core::IncomingMessage<MarkerArray> &);
  void octree_layers_cb(const Core::IncomingMessage<MarkerArray> &);
  void planned_path_cb(const Core::IncomingMessage<Path> &);
  void local_planned_path_cb(const Core::IncomingMessage<Path> &);
  void person_reco_path_cb(const Core::IncomingMessage<Point> &);

  std::shared_ptr<rerun::RecordingStream> get_recording_stream() const { return _rec; }
  void detection_image_cb(const Core::IncomingMessage<DetectionImage> &);
  void zed_image_cb(const Core::IncomingMessage<Image> &);
//   void route_path_cb(const Core::IncomingMessage<Route> &);
  void render_path(const Core::IncomingMessage<Path> &, const std::string &,
                   const std::string &);
  void log_map(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
               std::string index);
  void map_cloud_chunk_cb(const Core::IncomingMessage<PointCloudChunk> &msg);
  rerun::Color distance_to_color(float distance);
  Viewer(Core::ArgumentParser);
  void run() override;
};

#endif  // VIEWER_HPP