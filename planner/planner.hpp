#include "message.hpp"
#include "publisher.hpp"
#include "subscriber.hpp"
#include "vertex.hpp"
#include <Eigen/Dense>
#include <capnp_schemas/mavlink.capnp.h>
// #include <pcl-1.15/pcl/point_cloud.h>

#ifndef PLANNER_HPP
#define PLANNER_HPP

class Planner : Core::Vertex {
 private:
  std::shared_ptr<Core::Publisher<Takeoff>> _takeoff_publisher;
  std::shared_ptr<Core::Publisher<Land>> _land_publisher;
  std::shared_ptr<Core::Publisher<StartOffboard>> _start_offboard_publisher;
  std::shared_ptr<Core::Publisher<StopOffboard>> _stop_offboard_publisher;
  std::shared_ptr<Core::Subscriber<HomePosition>> _home_position_sub;

  // TODO: this goes in controller
  Eigen::Vector3d _home_position;

 public:
  Planner(int, char**);

  void home_position_cb(const Core::IncomingMessage<HomePosition>&);

  void run();
};

#endif
