#include "argument_parser.hpp"
#include "capnp_schemas/geometry_msgs.capnp.h"
#include "crazyflie_cpp/Crazyflie.h"
#include "message.hpp"
#include "subscriber.hpp"
#include "vertex.hpp"
#include <memory>

#ifndef GZ_HPP
#define GZ_HPP

class CrazyBridge : public Core::Vertex {
 private:
  Crazyflie crf;
  std::shared_ptr<Core::Subscriber<PoseStamped>> _pose_subscriber;
  void cb(const char*);

 public:
  void pose_callback(const Core::IncomingMessage<PoseStamped>&);
  CrazyBridge(Core::ArgumentParser);
  void run();
};

#endif
