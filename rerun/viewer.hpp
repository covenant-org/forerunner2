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
  
  std::shared_ptr<rerun::RecordingStream> _rec;
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

  std::shared_ptr<rerun::RecordingStream> get_recording_stream() const { return _rec; }
  void log_map(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
               std::string index);
  void map_cloud_chunk_cb(const Core::IncomingMessage<PointCloudChunk> &msg);
  rerun::Color distance_to_color(float distance);
  Viewer(Core::ArgumentParser);
  void run() override;
};

#endif  // VIEWER_HPP