#include "ftp_bridge.hpp"
#include <fstream>
#include <string>

FtpBridge::FtpBridge(const Core::ArgumentParser& args) : Core::Vertex(args) {
  if (auto uri = args.present("--mavlink-uri")) {
  } else {
    this->_ftp_config_sub = this->create_subscriber<KeyValue>(
        "config/ftp",
        std::bind(&FtpBridge::config_cb, this, std::placeholders::_1));
    this->_map_sub = nullptr;
  }
}

void FtpBridge::config_cb(const Core::IncomingMessage<KeyValue>& msg) {
  auto key = msg.content.getKey();
  if (key != "FTP_DIR") return;
  this->_ftp_folder = msg.content.getValue();
  if (this->_map_sub == nullptr) {
    this->_map_sub = this->create_subscriber<PointCloudChunk>(
        "map", std::bind(&FtpBridge::map_cb, this, std::placeholders::_1));
  }
}

void FtpBridge::map_cb(const Core::IncomingMessage<PointCloudChunk>& msg) {
  auto id = msg.content.getId();
  std::ofstream cloudFile(this->_ftp_folder + "/" + std::to_string(id));
  cloudFile << msg.buffer;
  cloudFile.close();
}

int main(int argc, char** argv) {
  Core::ArgumentParser args(argc, argv);
  args.add_argument("--mavlink-uri");
  FtpBridge bridge(args);
  bridge.run();
}
