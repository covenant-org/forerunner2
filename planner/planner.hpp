#include "vertex.hpp"
#include <capnp_schemas/mavlink.capnp.h>

#ifndef PLANNER_HPP
#define PLANNER_HPP

class Planner : Core::Vertex {
 private:
  std::shared_ptr<Core::Publisher<Takeoff>> _takeoff_publisher;
  std::shared_ptr<Core::Publisher<Land>> _land_publisher;

 public:
  Planner(int, char**);

  void run();
};

#endif
