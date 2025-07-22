#include "publisher.hpp"
#include "vertex.hpp"
#include <capnp_schemas/zed.capnp.h>
#include <gz/msgs.hh>
#include <gz/transport.hh>

#ifndef GZ_HPP
#define GZ_HPP

class GZ : public Core::Vertex {
 private:
  std::shared_ptr<Core::Publisher<PointCloud>> _point_cloud_publisher;
  std::shared_ptr<gz::transport::Node> _gz_node;

 public:
  GZ(int, char**);
  void on_point_cb(const gz::msgs::PointCloudPacked&,
                   const gz::transport::MessageInfo&);
  void on_lmic_cb(const gz::msgs::Double&, const gz::transport::MessageInfo&);
  void on_rmic_cb(const gz::msgs::Double&, const gz::transport::MessageInfo&);
  void run();
};

#endif
