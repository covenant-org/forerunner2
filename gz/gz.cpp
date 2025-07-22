#include "gz.hpp"
#include <gz/transport/MessageInfo.hh>
#include <gz/transport/Node.hh>
#include <iostream>

GZ::GZ(int argc, char **argv) : Core::Vertex(argc, argv) {
  _point_cloud_publisher =
      std::make_shared<Core::Publisher<PointCloud>>("point_cloud");

  _gz_node = std::make_shared<gz::transport::Node>();
  if (!_gz_node->Subscribe("/depth_camera/points", &GZ::on_point_cb, this)) {
    throw std::runtime_error("Failed to subscribe to point cloud topic");
  }

  if (!_gz_node->Subscribe("/model/x500_depth_mic_0/sensor/mic_1/detection",
                           &GZ::on_lmic_cb, this)) {
    throw std::runtime_error("Failed to subscribe to left mic");
  }

  if (!_gz_node->Subscribe("/model/x500_depth_mic_0/sensor/mic_2/detection",
                           &GZ::on_rmic_cb, this)) {
    throw std::runtime_error("Failed to subscribe to left mic");
  }
}

void GZ::on_point_cb(const gz::msgs::PointCloudPacked &pt,
                     const gz::transport::MessageInfo &info) {
  std::cout << "Received point cloud data." << info.Topic() << std::endl;
}

void GZ::on_lmic_cb(const gz::msgs::Double &db,
                    const gz::transport::MessageInfo &info) {
  std::cout << "Received mic 1: " << db.data() << std::endl;
}

void GZ::on_rmic_cb(const gz::msgs::Double &db,
                    const gz::transport::MessageInfo &info) {
  std::cout << "Received mic 2: " << db.data() << std::endl;
}

void GZ::run() {
  std::cout << "running" << std::endl;
  auto topics = this->_gz_node->SubscribedTopics();
  for (const std::string &topic : topics) {
    std::cout << topic << std::endl;
  }
  gz::transport::waitForShutdown();
}

int main(int argc, char **argv) {
  auto gz = std::make_shared<GZ>(argc, argv);
  gz->run();
  return 0;
}
