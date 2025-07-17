#include "publisher.hpp"
#include "vertex.hpp"
#include <capnp_schemas/zed.capnp.h>
#include <memory>
#include <sl/Camera.hpp>

#ifndef ZED_HPP
#define ZED_HPP

using namespace sl;

class Zed : Core::Vertex {
 private:
  Camera _camera;
  std::shared_ptr<Core::Publisher<PointCloud>> _cloud_point_pub;

 public:
  Zed(int, char**);
  void run();
};

#endif
