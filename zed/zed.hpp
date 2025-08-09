#include "argument_parser.hpp"
#include "publisher.hpp"
#include "vertex.hpp"
#include <capnp_schemas/zed.capnp.h>
#include <memory>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sl/Camera.hpp>

#ifndef ZED_HPP
#define ZED_HPP

using namespace sl;

class Zed : Core::Vertex {
 private:
  Camera _camera;
  std::shared_ptr<Core::Publisher<PointCloud>> _cloud_point_pub;
  std::shared_ptr<Core::Publisher<PointCloud>> _map_pub;
  pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* _cloud_encoder;
  pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* _map_encoder;
  pcl::visualization::CloudViewer viewer;

 public:
  Zed(const Core::ArgumentParser&);
  void run();
};

#endif
