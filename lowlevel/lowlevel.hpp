#include "action.hpp"
#include "argument_parser.hpp"
#include "capnp_schemas/mavlink.capnp.h"
#include "message.hpp"
#include "subscriber.hpp"
#include "vertex.hpp"
#include <Eigen/Dense>
#include <capnp_schemas/generics.capnp.h>
#include <memory>

class LowLevel : public Core::Vertex {
 private:
  std::shared_ptr<Core::Subscriber<Odometry>> _odom_sub;
  std::shared_ptr<Core::ActionClient<MissionCommand, GenericResponse>>
      _mavlink_client;
  Eigen::Quaterniond q;
  Eigen::Vector3d pos;
  void _odom_cb(const Core::IncomingMessage<Odometry>&);

 public:
  explicit LowLevel(Core::ArgumentParser);
  void run();
};
