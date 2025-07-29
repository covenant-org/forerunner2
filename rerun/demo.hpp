#include "argument_parser.hpp"
#include "message.hpp"
#include "subscriber.hpp"
#include "vertex.hpp"
#include <capnp_schemas/mavlink.capnp.h>
#include <capnp_schemas/visualization_msgs.capnp.h>
#include <capnp_schemas/sensors.capnp.h>
#include <capnp_schemas/zed.capnp.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <rerun.hpp>
#include <rerun/recording_stream.hpp>

#ifndef DEMO_HPP
#define DEMO_HPP

class Demo : Core::Vertex {
 private:
  std::shared_ptr<Core::Subscriber<PointCloud>> _sub;
  std::shared_ptr<Core::Subscriber<StereoMic>> _mic_sub;
  std::shared_ptr<Core::Subscriber<Odometry>> _odom_sub;
  std::shared_ptr<Core::Subscriber<MarkerArray>> _octree_sub;
  std::shared_ptr<rerun::RecordingStream> _rec;
  pcl::io::OctreePointCloudCompression<pcl::PointXYZ> *_point_cloud_decoder;

 public:
  void point_cloud_cb(const Core::IncomingMessage<PointCloud> &);
  void mic_cb(const Core::IncomingMessage<StereoMic> &);
  void odom_cb(const Core::IncomingMessage<Odometry> &);
  void octree_cb(const Core::IncomingMessage<MarkerArray> &);
  rerun::Color distance_to_color(float distance);
  Demo(Core::ArgumentParser);
  void run();
};

#endif  // DEMO_HPP
