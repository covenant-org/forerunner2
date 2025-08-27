#include "ftp_bridge.hpp"

FtpBridge::FtpBridge(const Core::ArgumentParser& args) {}

int main(int argc, char** argv) {
  Core::ArgumentParser args(argc, argv);
  FtpBridge bridge(args);
  bridge.run();
}
