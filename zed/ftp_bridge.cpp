#include "ftp_bridge.hpp"
#include "message.hpp"
#include <capnp/message.h>
#include <component_type.h>
#include <connection_result.h>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <plugins/ftp/ftp.h>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <vehicle.h>

#define BUFFER_INITIAL_SIZE 100000

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
    this->_system = this->_mavsdk.first_autopilot(3);
    this->_system->get()->subscribe_component_discovered_id(
        [&](mavsdk::ComponentType type, unsigned char id) {
          if (type == mavsdk::ComponentType::CompanionComputer) {
            this->_tmp_folder = "/tmp/zed.XXXXXX";
            if (mkdtemp(this->_tmp_folder.data()) == nullptr) {
              _logger.error("Could not create directory %s",
                            this->_ftp_folder.c_str());
              return;
            }
            this->_ftp_client =
                std::make_shared<mavsdk::Ftp>(this->_system.value());
            this->_ftp_client->set_target_compid(id);
          }
        });
    if (!this->_system) {
      this->_logger.error("Cloud not find an autopilot in the address %s",
                          uri.value().c_str());
      throw std::runtime_error("Cannot connect to mavlink uri");
    }
    return;
  }
  this->_ftp_config_sub = this->create_subscriber<KeyValue>(
      "config/ftp",
      std::bind(&FtpBridge::config_cb, this, std::placeholders::_1));
}

void FtpBridge::poll_files() {
  if (!this->_ftp_client) return;
  auto list = this->_ftp_client->list_directory("/");
  if (list.first != mavsdk::Ftp::Result::Success) {
    _logger.error("Error while listing FTP directory");
    return;
  }
  for (const auto& item : list.second.files) {
    lock.busy();
    this->_logger.debug("%s", item.c_str());
    std::string local_path = this->_tmp_folder;
    this->_ftp_client->download_async(
        item, local_path, false,
        [=](mavsdk::Ftp::Result res, mavsdk::Ftp::ProgressData progress) {
          switch (res) {
            case mavsdk::Ftp::Result::Busy:
              _logger.debug("Download of %s in progress", item.c_str());
              break;
            case mavsdk::Ftp::Result::Next:
              _logger.debug("Downloading %s: %d / %d", item.c_str(),
                            progress.bytes_transferred, progress.total_bytes);
              break;
            case mavsdk::Ftp::Result::FileDoesNotExist:
              _logger.error("File %s does not exists", item.c_str());
              break;
            case mavsdk::Ftp::Result::Success: {
              _logger.info("Downloaded file %s ", item.c_str());
              std::ifstream file(_tmp_folder + "/" + item,
                                 std::ifstream::in | std::ifstream::binary);
              size_t total_bytes = 0;
              std::string input(BUFFER_INITIAL_SIZE, '\0');
              while (!file.eof()) {
                if (total_bytes >= input.size()) {
                  input.resize(input.size() + BUFFER_INITIAL_SIZE);
                }
                file.read(input.data(), input.size());
                total_bytes += file.gcount();
              }
              this->_map_pub->_dangerously_raw_send(input.c_str(), total_bytes);
              break;
            }
            default:
              _logger.error("Ftp Error while downloading %s", item.c_str());
          }
          lock.free();
        });
  }
  lock.join();
}

void FtpBridge::config_cb(const Core::IncomingMessage<KeyValue>& msg) {
  auto key = msg.content.getKey();
  if (key != "FTP_DIR") return;
  this->_ftp_folder = msg.content.getValue();
  this->_logger.debug("Found ftp folder at %s", _ftp_folder.c_str());
  if (this->_map_sub == nullptr) {
    this->_logger.debug("Created map subscriber");
    this->_map_sub = this->create_subscriber<PointCloudChunk>(
        "map", std::bind(&FtpBridge::map_cb, this, std::placeholders::_1));
  }
}

void FtpBridge::map_cb(const Core::IncomingMessage<PointCloudChunk>& msg) {
  auto id = msg.content.getIndex();
  std::ofstream cloudFile(this->_ftp_folder + "/" + std::to_string(id),
                          std::ofstream::out | std::ofstream::binary);
  cloudFile << msg.buffer;
  this->_logger.debug("Wrote %s/%d", this->_ftp_folder.c_str(), id);
  cloudFile.close();
}

void FtpBridge::run() {
  if (auto uri = this->_args.present("--mavlink-uri")) {
    while (true) {
      this->poll_files();
      sleep(1);
    }
    return;
  }
  return Core::Vertex::run();
}

int main(int argc, char** argv) {
  Core::BaseArgumentParser args(argc, argv);
  args.add_argument("--mavlink-uri");
  FtpBridge bridge(args);
  bridge.run();
}
