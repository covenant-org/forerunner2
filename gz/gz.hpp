#include "publisher.hpp"
#include "vertex.hpp"
#include <gz/msgs/pointcloud.pb.h>
#include <gz/transport.hh>
// #include <capnp_schemas/zed.capnp.h>

#ifndef GZ_HPP
#define GZ_HPP

class GZ : public Core::Vertex {
 private:
  //   std::shared_ptr<Core::Publisher<PointCloud>> _point_cloud_publisher;
  std::shared_ptr<gz::transport::Node> _gz_node;

 public:
  GZ(int, char**);
  void on_point_cb(const gz::msgs::PointCloud&,
                   const gz::transport::MessageInfo&);
  void run();
};

#endif
