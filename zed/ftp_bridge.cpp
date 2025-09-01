#include "ftp_bridge.hpp"
#include <connection_result.h>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

FtpBridge::FtpBridge(const Core::ArgumentParser& args)
    : Core::Vertex(args),
      _mavsdk(
          mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation}) {
  this->_map_sub = nullptr;
  if (auto uri = args.present("--mavlink-uri")) {
    this->_map_pub = this->create_publisher<PointCloudChunk>("map");
    mavsdk::ConnectionResult connres =
        this->_mavsdk.add_any_connection(uri.value());
    if (connres != mavsdk::ConnectionResult::Success) {
      this->_logger.error("Error connecting to %s with code: %d", uri.value(),
                          connres);
      throw std::runtime_error("Cannot connect to mavlink uri");
    }
    this->_system = this->_mavsdk.first_autopilot(3.0);
    if (!this->_system) {
      this->_logger.error("Cloud not find an autopilot in the address %s",
                          uri.value());
      throw std::runtime_error("Cannot connect to mavlink uri");
    }
    this->_ftp_client = std::make_shared<mavsdk::Ftp>(this->_system.value());
    return;
  }
  this->_ftp_config_sub = this->create_subscriber<KeyValue>(
      "config/ftp",
      std::bind(&FtpBridge::config_cb, this, std::placeholders::_1));
}

void FtpBridge::poll_files() {
  auto list = this->_ftp_client->list_directory("/");
  for (const auto& item : list.second.files) {
    this->_logger.debug("%s", item.c_str());
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
  auto id = msg.content.getIndex();
  std::ofstream cloudFile(this->_ftp_folder + "/" + std::to_string(id));
  cloudFile << msg.buffer;
  cloudFile.close();
}

void FtpBridge::run() {
  if (auto uri = this->_args.present("--mablinks-uri")) {
    while (true) {
      this->poll_files();
      sleep(1);
    }
    return;
  }
  return Core::Vertex::run();
}

int main(int argc, char** argv) {
  Core::ArgumentParser args(argc, argv);
  args.add_argument("--mavlink-uri");
  FtpBridge bridge(args);
  bridge.run();
}
