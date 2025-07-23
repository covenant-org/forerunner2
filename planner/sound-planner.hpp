#ifndef SOUND_PLANNER_HPP
#define SOUND_PLANNER_HPP

#include "vertex.hpp"

class SoundPlanner : public Core::Vertex() {
 private:
  std::shared_ptr<Core::Subscriber<Odometry>> _odmetry_sub;
  std::shared_ptr<Core::ActionClient<Comand, GenericResponse>> _command_client;

 public:
  explicit SoundPlanner(int argc, char** argv);
  void run();
  void odometry_cb(const Core::IncomingMessage<Odometry>&);
};

#endif
