#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "message.hpp"
#include "publisher.hpp"
#include "subscriber.hpp"
#include "vertex.hpp"
#include <Eigen/Dense>
#include <capnp_schemas/controller.capnp.h>
#include <capnp_schemas/generics.capnp.h>
#include <capnp_schemas/mavlink.capnp.h>
#include <capnp_schemas/nav_msgs.capnp.h>
#include <capnp_schemas/planner.capnp.h>
#include <chrono>
#include <cstdint>

using namespace std::chrono_literals;

class Controller : public Core::Vertex {
 public:
  Controller(Core::ArgumentParser parser);
  ~Controller() {};
  void control();
  void run() override;
  void smooth_path();

 private:
  std::shared_ptr<Core::Publisher<PoseStamped>> _reached_goal_pub;
  std::shared_ptr<Core::Publisher<Path>> _smoothed_path_pub;
  std::shared_ptr<Core::Subscriber<HomePosition>> _home_pos_sub;
  std::shared_ptr<Core::Subscriber<Path>> _path_sub;
  std::shared_ptr<Core::Subscriber<Odometry>> _odometry_sub;
  std::shared_ptr<Core::Subscriber<Telemetry>> _telemetry_sub;
  std::shared_ptr<Core::Subscriber<Position>> _goal_sub;
  std::shared_ptr<Core::ActionClient<Command, GenericResponse>>
      _controller_client;
  std::shared_ptr<Core::ActionClient<MissionCommand, GenericResponse>>
      _mission_client;
  std::shared_ptr<Core::ActionClient<ReplanRequest, GenericResponse>>
      _planner_client;
  std::thread _control_thread;

  Path::Reader _path;
  float _heading;
  Eigen::Vector3f _position;
  Eigen::Quaternionf _quat;
  bool recived_path;
  bool waiting_for_path;
  bool waiting_reponse;
  size_t _path_sequence;
  bool recorded_initial_position;
  int index;
  int sent_point;
  char debauncer;

  void planned_path_cb(const Core::IncomingMessage<Path> &);
  void path_cb(const Core::IncomingMessage<Path> &);
  void goal_cb(const Core::IncomingMessage<Position> &);
  void local_pose_cb(const Core::IncomingMessage<PoseStamped> &);
  void odometry_cb(const Core::IncomingMessage<Odometry> &);
  void telemetry_cb(const Core::IncomingMessage<Telemetry> &);
  void publish_trajectory_setpoint(PoseStamped::Reader &pose);

  Eigen::Vector3f _vehicle_initial_position;
  Eigen::Vector3f _last_path_start_position;
  Eigen::Affine3f _initial_transform;
  Eigen::Vector3f current_position;
  Eigen::Vector3f temp_goal;
  Eigen::Vector3f home_position;
  Eigen::Quaterniond temp_orientation;
  Eigen::Vector3f _goal_target;
  Eigen::Vector3f _local_pose;
  bool _original_local_pose_stored;
};
//
#endif
