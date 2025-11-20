#include "action.hpp"
#include "argument_parser.hpp"
#include "capnp_schemas/mavlink.capnp.h"
#include "message.hpp"
#include "publisher.hpp"
#include "subscriber.hpp"
#include "vertex.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Geometry/Quaternion.h>
#include <capnp_schemas/controller.capnp.h>
#include <capnp_schemas/generics.capnp.h>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <memory>

class LowLevel : public Core::Vertex {
 private:
  std::shared_ptr<Core::Subscriber<Odometry>> _odom_sub;
  std::shared_ptr<Core::Publisher<Position>> _goal_pub;
  std::shared_ptr<Core::Publisher<ControlMetrics>> _metrics_pub;
  Eigen::Quaterniond _q;
  Eigen::Quaterniond _qd;
  Eigen::Quaterniond _qe;
  std::shared_ptr<Core::ActionClient<Command, GenericResponse>> _mavlink_client;
  Eigen::Vector3d _pos;
  Eigen::Vector3d _linear_velocity;
  Eigen::Vector3d _angular_velocity;
  Eigen::Vector3d _g;
  Eigen::Vector3d _f;
  Eigen::Matrix3d _kdt;
  Eigen::Matrix3d _kpt;
  Eigen::Matrix3d _J;
  Eigen::Matrix3d _kpr;
  Eigen::Matrix3d _kdr;
  double _m;
  void _odom_cb(const Core::IncomingMessage<Odometry>&);
  Eigen::Quaterniond get_q_desired(Eigen::Vector3d fu, double yaw);
  Eigen::Quaterniond get_q_error(Eigen::Quaterniond);
  Eigen::Vector3d get_f_desired(Eigen::Vector3d);
  Eigen::Vector3d get_torque_input(Eigen::Vector3d fu, double yaw);

 public:
  explicit LowLevel(Core::ArgumentParser);
  void run();
};
