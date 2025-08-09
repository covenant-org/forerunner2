#include "argument_parser.hpp"
#include "message.hpp"
#include "subscriber.hpp"
#include "vertex.hpp"
#include <capnp_schemas/mavlink.capnp.h>
#include <capnp_schemas/nav_msgs.capnp.h>
#include <capnp_schemas/sensors.capnp.h>
#include <capnp_schemas/visualization_msgs.capnp.h>
#include <capnp_schemas/zed.capnp.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/impl/point_types.hpp>
#include <rerun.hpp>
#include <rerun/recording_stream.hpp>

#ifndef DEMO_HPP
#define DEMO_HPP

class Demo : Core::Vertex {
 private:
  std::shared_ptr<Core::Subscriber<PointCloud>> _sub;
  std::shared_ptr<Core::Subscriber<PointCloud>> _map_sub;
  std::shared_ptr<Core::Subscriber<StereoMic>> _mic_sub;
  std::shared_ptr<Core::Subscriber<Odometry>> _odom_sub;
  std::shared_ptr<Core::Subscriber<MarkerArray>> _octree_sub;
  std::shared_ptr<Core::Subscriber<MarkerArray>> _octree_layers_sub;
  std::shared_ptr<Core::Subscriber<Path>> _planned_path_sub;
  std::shared_ptr<Core::Subscriber<Path>> _local_planned_path_sub;
  std::shared_ptr<rerun::RecordingStream> _rec;
  pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> *_point_cloud_decoder;
  pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> *_map_decoder;

 public:
  void point_cloud_cb(const Core::IncomingMessage<PointCloud> &);
  void map_cloud_cb(const Core::IncomingMessage<PointCloud> &);
  void mic_cb(const Core::IncomingMessage<StereoMic> &);
  void odom_cb(const Core::IncomingMessage<Odometry> &);
  void octree_cb(const Core::IncomingMessage<MarkerArray> &);
  void octree_layers_cb(const Core::IncomingMessage<MarkerArray> &);
  void planned_path_cb(const Core::IncomingMessage<Path> &);
  void local_planned_path_cb(const Core::IncomingMessage<Path> &);
  void render_path(const Core::IncomingMessage<Path> &, const std::string &,
                   const std::string &);
  rerun::Color distance_to_color(float distance);
  Demo(Core::ArgumentParser);
  void run();
};

#endif  // DEMO_HPP
