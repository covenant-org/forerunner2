#include "argument_parser.hpp"
#include "mapping.hpp"
#include "message.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Constants.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>
#include <capnp_schemas/zed.capnp.h>
#include <capnp_schemas/nav_msgs.capnp.h>
#include <capnp_schemas/sensors.capnp.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>

Mapping::Mapping(Core::ArgumentParser args)
    : Vertex(args), _cloud(new pcl::PointCloud<pcl::PointXYZRGBA>) {
  this->_odom_sub = this->create_subscriber<Odometry>(
      "odometry", std::bind(&Mapping::odom_cb, this, std::placeholders::_1));
  this->_cloud_sub = this->create_subscriber<PointCloud>(
      "point_cloud",
      std::bind(&Mapping::stitch_cloud, this, std::placeholders::_1));
  this->_full_cloud_publisher = this->create_publisher<PointCloud>("map");
  pcl::io::compression_Profiles_e compressionProfile =
      pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
  _cloud_encoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>(
      compressionProfile, false);
  this->_point_cloud_decoder =
      new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>();
}

void Mapping::odom_cb(const Core::IncomingMessage<Odometry>& odom) {
  // Extract pose (with covariance) and then pose
  auto pose_with_cov = odom.content.getPose();
  auto pose = pose_with_cov.getPose();
  auto position = pose.getPosition();
  this->_position = Eigen::Vector3d(position.getX(), -position.getY(), -position.getZ());

  // Extract orientation quaternion
  auto orientation = pose.getOrientation();
  double qw = orientation.getW();
  double qx = orientation.getX();
  double qy = orientation.getY();
  double qz = orientation.getZ();
  this->_orientation = Eigen::Quaterniond(qw, qx, -qy, -qz);
}

void Mapping::stitch_cloud(const Core::IncomingMessage<PointCloud>& msg) {
  auto data_reader = msg.content.getData();

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGBA>());
  std::stringstream buffer(
      std::string((char*)data_reader.begin(), data_reader.size()));

  try {
    _point_cloud_decoder->decodePointCloud(buffer, cloud);
  } catch (const std::exception& e) {
    _logger.warn("Error while decoding cloudpoint: %s", e.what());
    return;
  }
  this->_logger.debug("Decoded cloud point");

  auto height = cloud->height;
  if (height < _cloud->height) {
    height = _cloud->height;
  }

  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.translate(this->_position);
  transform.rotate(this->_orientation);
  pcl::transformPointCloud(*cloud, *cloud, transform);

  *this->_cloud += *cloud;

  pcl::VoxelGrid<pcl::PointXYZRGBA> sor2;
  sor2.setInputCloud(_cloud);
  sor2.setLeafSize(0.4f, 0.4f, 0.4f);
  sor2.filter(*_cloud);

  std::stringstream encoded_cloud;
  _cloud_encoder->encodePointCloud(_cloud, encoded_cloud);

  auto out = this->_full_cloud_publisher->new_msg();
  auto output_buffer = encoded_cloud.str();
  out.content.initData(output_buffer.size());
  out.content.setSize(output_buffer.size());
  out.content.setWidth(_cloud->width);
  out.content.setHeight(_cloud->height);
  auto reader = ::capnp::Data::Reader((unsigned char*)output_buffer.data(),
                                      output_buffer.size());
  out.content.setData(reader);
  out.publish();
}

int main(int argc, char** argv) {
  Core::BaseArgumentParser arguments(argc, argv);
  Mapping mapping(arguments);
  mapping.run();
}
