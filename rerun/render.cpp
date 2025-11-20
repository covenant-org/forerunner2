
#include "render.hpp"
#include <iostream>

int main(int argc, char** argv) {
    Core::BaseArgumentParser args(argc, argv);

  std::shared_ptr<rerun::RecordingStream> stream_;
  stream_ = std::make_shared<rerun::RecordingStream>("renderer_test");
  
  RegistryExample::Renderer app(args, nullptr, nullptr);
  app.run();
  return 0;
}