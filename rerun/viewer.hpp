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
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <rerun.hpp>
#include <rerun/recording_stream.hpp>

#ifndef DEMO_HPP
#define DEMO_HPP

class Viewer : Core::Vertex {
 private:
  std::shared_ptr<Core::Subscriber<PointCloud>> _sub;
  std::shared_ptr<Core::Subscriber<PointCloud>> _map_sub;
  std::shared_ptr<Core::Subscriber<PointCloudChunk>> _map_chunk_sub;
  std::shared_ptr<Core::Subscriber<Position>> _goal_sub;
  std::shared_ptr<Core::Subscriber<StereoMic>> _mic_sub;
  std::shared_ptr<Core::Subscriber<Odometry>> _odom_sub;
  std::shared_ptr<Core::Subscriber<MarkerArray>> _octree_sub;
  std::shared_ptr<Core::Subscriber<MarkerArray>> _octree_layers_sub;
  std::shared_ptr<Core::Subscriber<Path>> _planned_path_sub;
  std::shared_ptr<Core::Subscriber<Path>> _local_planned_path_sub;
  std::shared_ptr<Core::Subscriber<Point>> _person_reco_path_sub;
  std::shared_ptr<Core::Subscriber<DetectionImage>> _detection_images_sub;
  std::shared_ptr<Core::Subscriber<ImageData>> _zed_image_sub;
  std::shared_ptr<rerun::RecordingStream> _rec;
  pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> *_point_cloud_decoder;
  pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>
      *_map_point_cloud_decoder;
  rerun::archetypes::Asset3D _drone_model;
  rerun::Quaternion _drone_quat;

 public:
  void goal_cb(const Core::IncomingMessage<Position> &);
  void point_cloud_cb(const Core::IncomingMessage<PointCloud> &);
  void map_cloud_cb(const Core::IncomingMessage<PointCloud> &);
  void mic_cb(const Core::IncomingMessage<StereoMic> &);
  void odom_cb(const Core::IncomingMessage<Odometry> &);
  void octree_cb(const Core::IncomingMessage<MarkerArray> &);
  void octree_layers_cb(const Core::IncomingMessage<MarkerArray> &);
  void planned_path_cb(const Core::IncomingMessage<Path> &);
  void local_planned_path_cb(const Core::IncomingMessage<Path> &);
  void person_reco_path_cb(const Core::IncomingMessage<Point> &);
  void detection_image_cb(const Core::IncomingMessage<DetectionImage> &);
  void zed_image_cb(const Core::IncomingMessage<ImageData> &);
  void render_path(const Core::IncomingMessage<Path> &, const std::string &,
                   const std::string &);
  void log_map(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
               std::string index);
  void map_cloud_chunk_cb(const Core::IncomingMessage<PointCloudChunk> &msg);
  rerun::Color distance_to_color(float distance);
  Viewer(Core::ArgumentParser);
  void run();
};

#endif  // DEMO_HPP
