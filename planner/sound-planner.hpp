#ifndef SOUND_PLANNER_HPP
#define SOUND_PLANNER_HPP

#include "Eigen/Dense"
#include "argument_parser.hpp"
#include "vertex.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <capnp_schemas/controller.capnp.h>
#include <capnp_schemas/generics.capnp.h>
#include <capnp_schemas/mavlink.capnp.h>
#include <capnp_schemas/sensors.capnp.h>

class SoundPlanner : public Core::Vertex {
 private:
  std::shared_ptr<Core::Subscriber<Odometry>> _odmetry_sub;
  std::shared_ptr<Core::Subscriber<Telemetry>> _telemetry_sub;
  std::shared_ptr<Core::Subscriber<StereoMic>> _mic_sub;
  std::shared_ptr<Core::Subscriber<Altitude>> _altitude_sub;
  std::shared_ptr<Core::ActionClient<Command, GenericResponse>> _command_client;

  int _telemetry_count;
  bool _is_armed;
  bool _is_in_air;
  bool _arm_requested;
  bool _takeoff_requested;
  float _lmic;
  float _rmic;
  float altitude;
  float target_altitude;
  float calc_mic_diff();
  Eigen::Quaternionf quart;
  Eigen::Vector3f position;
  Eigen::Vector3f goal;

 public:
  explicit SoundPlanner(Core::ArgumentParser);
  void run();
  void odometry_cb(const Core::IncomingMessage<Odometry>&);
  void telemetry_cb(const Core::IncomingMessage<Telemetry>&);
  void mic_cb(const Core::IncomingMessage<StereoMic>&);
  void altitude_cb(const Core::IncomingMessage<Altitude>&);
};

#endif
