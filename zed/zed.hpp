#include "argument_parser.hpp"
#include "publisher.hpp"
#include "vertex.hpp"
#include <capnp_schemas/detection_messages.capnp.h>
#include <capnp_schemas/zed.capnp.h>
#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>  // OpenCV header
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
  std::shared_ptr<Core::Publisher<ImageData>> _img_publisher;
  pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* _cloud_encoder;
  std::chrono::steady_clock::time_point _last_published_frame;

 public:
  Zed(const Core::ArgumentParser&);
  void publish_zed_image(const cv::Mat& image);
  void run();
};

#endif
