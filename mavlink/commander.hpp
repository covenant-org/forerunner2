
#ifndef COMMANDER_HPP
#define COMMANDER_HPP

#include "message.hpp"
#include "publisher.hpp"
#include "subscriber.hpp"
#include "vertex.hpp"
#include <capnp_schemas/mavlink.capnp.h>
#include <capnp_schemas/controller.capnp.h>
#include <capnp_schemas/generics.capnp.h>
#include <mutex>

struct GeoPosition {
  double lat = 0.0;
  double lon = 0.0;
  double alt = 0.0;
};

struct HomePositionData {
  GeoPosition pos;
  bool set = false;
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

class Commander : public Core::Vertex {
 private:
  std::shared_ptr<Core::ActionClient<MissionCommand, GenericResponse>> _mission_client;
  std::shared_ptr<Core::ActionClient<Command, GenericResponse>> _controller_client;
  std::shared_ptr<Core::Subscriber<HomePosition>> _home_subscriber;
  std::mutex _home_mutex;

  HomePositionData _home;
  WaypointData _last_local;
  GlobalWaypointData _last_global;

 public:
  Commander(Core::ArgumentParser);
  void run() override;
};

#endif
