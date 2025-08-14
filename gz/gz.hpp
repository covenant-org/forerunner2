#include "argument_parser.hpp"
#include "publisher.hpp"
#include "vertex.hpp"
#include <atomic>
#include <capnp_schemas/sensors.capnp.h>
#include <capnp_schemas/zed.capnp.h>
#include <gz/msgs.hh>
#include <gz/msgs/pointcloud.pb.h>
#include <gz/transport.hh>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/io/pcd_io.h>

#ifndef GZ_HPP
#define GZ_HPP

class GZ : public Core::Vertex {
 private:
  std::shared_ptr<Core::Publisher<PointCloud>> _camera_pc_publisher, _lidar_pc_publisher;
  std::shared_ptr<gz::transport::Node> _gz_node;
  pcl::io::OctreePointCloudCompression<pcl::PointXYZ>* _cloud_encoder;
  std::shared_ptr<Core::Publisher<StereoMic>> _mic_publisher;
  std::atomic<double> _last_lmic_value;
  std::atomic<double> _last_rmic_value;
  void publish_mic();
  bool is_processing = false;
  // Store last clouds separately
  pcl::PointCloud<pcl::PointXYZ>::Ptr _last_camera_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr _last_lidar_cloud;

 public:
  GZ(Core::ArgumentParser);
  void on_lmic_cb(const gz::msgs::Double&, const gz::transport::MessageInfo&);
  void on_rmic_cb(const gz::msgs::Double&, const gz::transport::MessageInfo&);
  void on_camera_cb(const gz::msgs::PointCloudPacked&);
  void on_lidar_cb(const gz::msgs::PointCloudPacked&);
  void run();
};

#endif
