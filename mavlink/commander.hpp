
#ifndef COMMANDER_HPP
#define COMMANDER_HPP

#include "message.hpp"
#include "publisher.hpp"
#include "subscriber.hpp"
#include "vertex.hpp"
#include <capnp_schemas/mavlink.capnp.h>
#include <capnp_schemas/nav_msgs.capnp.h>
#include <capnp_schemas/controller.capnp.h>
#include <capnp_schemas/generics.capnp.h>
#include <iterator>

struct GeoPosition {
  double lat = 0.0;
  double lon = 0.0;
  double alt = 0.0;
};

struct WaypointData {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
};

struct GlobalWaypointData {
  GeoPosition pos;
  double yaw = 0.0;
};

struct SimpleMissionItem {
  double latitude;
  double longitude;
  float altitude;
  float speed;
  bool is_fly_through;
  std::string camera_action = "NONE";
  float loiter_time = 0.0f;
  float gimbal_pitch = 0.0f;
  float gimbal_yaw = 0.0f;
  float camera_photo_interval = 0.0f;
};

class Commander : public Core::Vertex {
 private:
  std::shared_ptr<Core::ActionClient<MissionCommand, GenericResponse>> _mission_client;
  std::shared_ptr<Core::ActionClient<Command, GenericResponse>> _controller_client;
  std::shared_ptr<Core::Subscriber<Odometry>> _odometry_subscriber;

  WaypointData _last_local;
  GlobalWaypointData _last_global;
  
  // Current odometry data from subscriber
  float _current_x = 0.0f;
  float _current_y = 0.0f;
  float _current_z = 0.0f;
  float _current_heading = 0.0f;
  bool _has_odometry = false;

  // Helper method for loading missions from file
  std::vector<SimpleMissionItem> load_mission_from_file(const std::string& filename);

 public:
  Commander(Core::ArgumentParser);
  void run() override;
  
  // Helper method for handling mission commands (for both interactive and CLI mode)
  void handle_mission_command(const std::vector<std::string>& args);

 private:
};

#endif
