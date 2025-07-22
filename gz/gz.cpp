#include "gz.hpp"
#include <gz/msgs/pointcloud.pb.h>
#include <iostream>

GZ::GZ(int argc, char **argv) : Core::Vertex(argc, argv) {
  _point_cloud_publisher =
      std::make_shared<Core::Publisher<PointCloud>>("point_cloud");

  _gz_node = std::make_shared<gz::transport::Node>();
  if (!_gz_node->Subscribe("/depth_camera/points", &GZ::on_point_cb, this)) {
    throw std::runtime_error("Failed to subscribe to point cloud topic");
  }
}

void GZ::on_point_cb(const gz::msgs::PointCloud &pt,
                     const gz::transport::MessageInfo &info) {
  std::cout << "Received point cloud data." << std::endl;
}

void GZ::run() {
  std::cout << "running" << std::endl;
  while (true) sleep(0.1);
}

int main(int argc, char **argv) {
  auto gz = std::make_shared<GZ>(argc, argv);
  gz->run();
  return 0;
}
