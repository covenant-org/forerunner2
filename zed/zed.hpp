#include "argument_parser.hpp"
#include "publisher.hpp"
#include "vertex.hpp"
#include <capnp_schemas/zed.capnp.h>
#include <chrono>
#include <memory>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/io/pcd_io.h>
#include <sl/Camera.hpp>

#ifndef ZED_HPP
#define ZED_HPP

class Zed : Core::Vertex {
 private:
  sl::Camera _camera;
  std::shared_ptr<Core::Publisher<PointCloud>> _cloud_point_pub;
  std::shared_ptr<Core::Publisher<PointCloudChunk>> _map_pub;
  pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* _cloud_encoder;
  std::chrono::steady_clock::time_point _last_published_frame;

 public:
  Zed(const Core::ArgumentParser&);
  void run();
};

#endif
