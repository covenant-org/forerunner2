#include "argument_parser.hpp"
#include "vertex.hpp"
#include <capnp_schemas/sensors.capnp.h>
#include <capnp_schemas/zed.capnp.h>
#include <gz/msgs.hh>
#include <gz/msgs/pointcloud.pb.h>
#include <gz/transport.hh>

#ifndef GZ_SUSPEND_HPP
#define GZ_SUSPEND_HPP

class GZSuspend : public Core::Vertex {
 private:
  std::shared_ptr<gz::transport::Node> _gz_node;

 public:
  GZSuspend(Core::ArgumentParser);
  void run();
};

#endif
