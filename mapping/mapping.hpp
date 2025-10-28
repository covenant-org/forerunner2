#include "message.hpp"
#include "vertex.hpp"
#include <Eigen/Dense>
#include <capnp_schemas/mavlink.capnp.h>
#include <capnp_schemas/zed.capnp.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/octree/octree_pointcloud.h>
class Mapping : public Core::Vertex {
 private:
  std::shared_ptr<Core::Subscriber<PointCloud>> _cloud_sub;
  std::shared_ptr<Core::Publisher<PointCloud>> _full_cloud_publisher;
  pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> *_cloud_encoder;
  std::shared_ptr<Core::Subscriber<Odometry>> _odom_sub;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _cloud;
  Eigen::Vector3d _position;
  Eigen::Quaterniond _orientation;
  pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> *_point_cloud_decoder;

 public:
  void stitch_cloud(const Core::IncomingMessage<PointCloud> &);
  void odom_cb(const Core::IncomingMessage<Odometry> &);
  Mapping(Core::ArgumentParser);
};
