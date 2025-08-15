#include "argument_parser.hpp"
#include "gz.hpp"
#include "logger.hpp"
#include <cmath>
#include <gz/msgs/pointcloud.pb.h>
#include <gz/transport/MessageInfo.hh>
#include <gz/transport/Node.hh>
#include <pcl/common/point_tests.h>
#include <pcl/compression/compression_profiles.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

GZ::GZ(Core::ArgumentParser args) : Core::Vertex(args) {
  _camera_pc_publisher = this->create_publisher<PointCloud>("point_cloud");
  _lidar_pc_publisher = this->create_publisher<PointCloud>("lidar");
  _mic_publisher = this->create_publisher<StereoMic>("mic");

  _gz_node = std::make_shared<gz::transport::Node>();
  if (!_gz_node->Subscribe("/depth_camera/points", &GZ::on_camera_cb, this)) {
    throw std::runtime_error("Failed to subscribe to point cloud topic");
  }
  // Subscribe to lidar point cloud topic
  if (!_gz_node->Subscribe("/world/imav_indoor/model/x500_lidar_down_0/link/lidar_sensor_link/sensor/lidar/scan/points",
                           &GZ::on_lidar_cb, this)) {
    throw std::runtime_error("Failed to subscribe to lidar point cloud topic");
  }

  if (!_gz_node->Subscribe("/model/x500_depth_mic_0/sensor/mic_1/detection",
                           &GZ::on_lmic_cb, this)) {
    throw std::runtime_error("Failed to subscribe to left mic");
  }

  if (!_gz_node->Subscribe("/model/x500_depth_mic_0/sensor/mic_2/detection",
                           &GZ::on_rmic_cb, this)) {
    throw std::runtime_error("Failed to subscribe to left mic");
  }

  pcl::io::compression_Profiles_e compression_profile =
      pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
  _cloud_encoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>(
      compression_profile, false);
}

void GZ::publish_mic() {
  auto out = this->_mic_publisher->new_msg();
  out.content.setLeft(this->_last_lmic_value);
  out.content.setRight(this->_last_rmic_value);
  out.publish();
}

void GZ::on_lmic_cb(const gz::msgs::Double &msg,
                    const gz::transport::MessageInfo &info) {
  this->_last_lmic_value = msg.data();
  this->_logger.debug("Received left mic");
  this->publish_mic();
}

void GZ::on_rmic_cb(const gz::msgs::Double &msg,
                    const gz::transport::MessageInfo &info) {
  this->_last_rmic_value = msg.data();
  this->_logger.debug("Received right mic");
  this->publish_mic();
}

void GZ::run() {
  auto topics = this->_gz_node->SubscribedTopics();
  this->_logger.info("Subscribed to topics");
  for (const std::string &topic : topics) {
    this->_logger.info("  %s", topic.c_str());
  }
  this->_logger.info("Running");
  gz::transport::waitForShutdown();
}

// Shared point cloud processing for both topics
void process_pointcloud(const gz::msgs::PointCloudPacked &pt, pcl::PointCloud<pcl::PointXYZ>::Ptr &out_cloud) {
  out_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
  out_cloud->width = pt.width();
  out_cloud->height = pt.height();
  out_cloud->is_dense = pt.is_dense();
  out_cloud->points.resize(out_cloud->width * out_cloud->height);
  // manually convert each point, in pcl each is represented by 16 bytes and in
  // gazebo 24 bytes
  // can run: gz topic -e -t /depth_camera/points
  // field { name: "x" datatype: FLOAT32 count: 1 }
  // field { name: "y" offset: 4 datatype: FLOAT32 count: 1 }
  // field { name: "z" offset: 8 datatype: FLOAT32 count: 1 }
  // field { name: "rgb" offset: 16 datatype: FLOAT32 count: 1 }
  // height: 480 width: 640 point_step: 24
  // Shared point cloud processing for both topics
  const uint8_t *data_ptr = reinterpret_cast<const uint8_t *>(pt.data().data());
  for (size_t i = 0; i < out_cloud->points.size(); ++i) {
    size_t point_offset = i * pt.point_step();
    float x, y, z;
    memcpy(&x, data_ptr + point_offset + 0, sizeof(float));
    memcpy(&y, data_ptr + point_offset + 4, sizeof(float));
    memcpy(&z, data_ptr + point_offset + 8, sizeof(float));
    if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
      out_cloud->points[i].x = x;
      out_cloud->points[i].y = y;
      out_cloud->points[i].z = z;
    }
  }
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(out_cloud);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(0.0f, 10.0f);
  pass.filter(*out_cloud);
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(out_cloud);
  voxel_filter.setLeafSize(0.5f, 0.5f, 0.5f);
  voxel_filter.filter(*out_cloud);
}

void GZ::on_camera_cb(const gz::msgs::PointCloudPacked &pt) {
  if (is_processing) return;
  is_processing = true;
  process_pointcloud(pt, _last_camera_cloud);
  auto msg = this->_camera_pc_publisher->new_msg();
  msg.content.setHeight(pt.height());
  msg.content.setWidth(pt.width());
  std::stringstream encoded_cloud;
  _cloud_encoder->encodePointCloud(_last_camera_cloud, encoded_cloud);
  auto buffer = encoded_cloud.str();
  msg.content.initData(buffer.size());
  msg.content.setSize(buffer.size());
  auto reader = ::capnp::Data::Reader((unsigned char *)buffer.data(), buffer.size());
  msg.content.setData(reader);
  msg.publish();
  is_processing = false;
}

void GZ::on_lidar_cb(const gz::msgs::PointCloudPacked &pt) {
  if (is_processing) return;
  is_processing = true;
  process_pointcloud(pt, _last_lidar_cloud);
  auto msg = this->_lidar_pc_publisher->new_msg();
  msg.content.setHeight(pt.height());
  msg.content.setWidth(pt.width());
  std::stringstream encoded_cloud;
  _cloud_encoder->encodePointCloud(_last_lidar_cloud, encoded_cloud);
  auto buffer = encoded_cloud.str();
  msg.content.initData(buffer.size());
  msg.content.setSize(buffer.size());
  auto reader = ::capnp::Data::Reader((unsigned char *)buffer.data(), buffer.size());
  msg.content.setData(reader);
  msg.publish();
  is_processing = false;
}

int main(int argc, char **argv) {
  Core::BaseArgumentParser args(argc, argv);
  auto gz = std::make_shared<GZ>(args);
  gz->run();
  return 0;
}
