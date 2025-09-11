#include "ftp_bridge.hpp"
#include "message.hpp"
#include <capnp/message.h>
#include <component_type.h>
#include <connection_result.h>
#include <cstdlib>
#include <fstream>
#include <vector>
#include <iterator>
#include <memory>
#include <plugins/ftp/ftp.h>
#include <sstream>
#include <stdexcept>
#include <unistd.h>
#include <vehicle.h>



#define BUFFER_INITIAL_SIZE 100000

FtpBridge::FtpBridge(const Core::ArgumentParser& args)
    : Core::Vertex(args),
      _mavsdk(
          mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation}) {
  this->_map_sub = nullptr;
  _logger.debug("FtpBridge initialized");
  if (auto uri = args.present("--mavlink-uri")) {
    _logger.debug("Connecting to MAVLink URI: %s", uri.value().c_str());
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
      _logger.info("FTP client initialized for CompanionComputer id=%d", (int)id);
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
  _logger.debug("Polling files from FTP server");
  if (!this->_ftp_client) {
    _logger.info("FTP client not initialized");
    return;
  }
  auto list = this->_ftp_client->list_directory("/");
  if (list.first != mavsdk::Ftp::Result::Success) {
    _logger.error("Error while listing FTP directory");
    return;
  }
  if (_logger.get_level() == Core::LogLevel::DEBUG) {
    std::stringstream filelist;
    filelist << "Found files: ";
    for (auto it = list.second.files.begin(); it != list.second.files.end();
         ++it) {
      filelist << *it;
      if (it + 1 != list.second.files.end()) {
        filelist << ", ";
      }
    }
    _logger.debug("%s", filelist.str().c_str());
  }
  for (const auto& item : list.second.files) {
    _logger.debug("Downloading file: %s", item.c_str());
    lock.busy();
    this->_logger.debug("%s", item.c_str());
    std::string local_path = this->_tmp_folder;
    this->_ftp_client->download_async(
        item, local_path, false,
        [=](mavsdk::Ftp::Result res, mavsdk::Ftp::ProgressData progress) {
          switch (res) {
            case mavsdk::Ftp::Result::Busy:
              _logger.debug("Download of %s in progress", item.c_str());
              return;
            case mavsdk::Ftp::Result::Next:
              _logger.debug("Downloading %s: %d / %d", item.c_str(),
                            progress.bytes_transferred, progress.total_bytes);
              return;
            case mavsdk::Ftp::Result::FileDoesNotExist:
              _logger.error("File %s does not exists", item.c_str());
              break;
            case mavsdk::Ftp::Result::Success: {
              _logger.info("Downloaded file %s ", item.c_str());
              std::ifstream file(_tmp_folder + "/" + item,
                                 std::ifstream::in | std::ifstream::binary);

              // detect image extensions (no case-insensitive handling for brevity)
              std::string extension;
              size_t pos = item.find_last_of('.');
              if (pos != std::string::npos) extension = item.substr(pos + 1);

              if (extension == "jpg" || extension == "jpeg" || extension == "png" || extension == "bmp") {
                // For images we read the full file into memory and publish as ImageData
                std::vector<char> buffer((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
                auto pub = this->create_publisher<ImageData>("image");
                if (pub) {
                  auto msg = pub->new_msg();
                  msg.content.setWidth(0); // TODO: extract real width
                  msg.content.setHeight(0); // TODO: extract real height
                  msg.content.setEncoding(extension);
                  msg.content.setTimestamp(static_cast<uint64_t>(std::time(nullptr)));
                  auto reader = ::capnp::Data::Reader(reinterpret_cast<const unsigned char*>(buffer.data()), buffer.size());
                  msg.content.setData(reader);
                  msg.publish();
                  _logger.info("Published image %s", item.c_str());
                } else {
                  _logger.error("Could not create image publisher");
                }
              } else {
                // Original chunked read logic for large binary files
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
              }

              this->_ftp_client->remove_file_async(
                  item, [=](mavsdk::Ftp::Result res) {
                    this->lock.free();
                    if (res != mavsdk::Ftp::Result::Success) {
                      _logger.error("Error while removing %s", item.c_str());
                      return;
                    }
                    std::string full_path = local_path + "/" + item;
                    std::remove(full_path.c_str());
                  });
              return;
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
  std::string key_str = std::string(key);
  std::string value_str = std::string(msg.content.getValue());
  _logger.debug("Received config message: key=%s, value=%s", key_str.c_str(), value_str.c_str());
  if (key != "FTP_DIR") return;
  this->_ftp_folder = msg.content.getValue();
  this->_logger.info("FTP folder set to %s", _ftp_folder.c_str());
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
  _logger.debug("Starting FtpBridge main loop");
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
