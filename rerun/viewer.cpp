#include "viewer.hpp"
#include "message.hpp"
#include "render.hpp"
#include "rerun/archetypes/arrows3d.hpp"
#include "rerun/archetypes/asset3d.hpp"
#include "rerun/archetypes/boxes3d.hpp"
#include "rerun/archetypes/encoded_image.hpp"
#include "rerun/components/fill_mode.hpp"
#include "rerun/components/media_type.hpp"
#include "rerun/components/translation3d.hpp"
#include "rerun/datatypes/quaternion.hpp"
#include "rerun/rotation3d.hpp"
#include "utils.hpp"
#include <Eigen/src/Geometry/Quaternion.h>
#include <capnp_schemas/zed.capnp.h>
#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <optional>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <rerun.hpp>
#include <rerun/recording_stream.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <zlib.h>

std::string Viewer::build_recording_path(
    const std::string& user_input,
    const std::optional<std::string>& recording_type) {
  namespace fs = std::filesystem;

  const auto root = Core::find_root();
  const fs::path base_path = root.empty() ? fs::current_path() : fs::path(root);
  fs::path recordings_dir = base_path / "rerun" / "recordings";

  if (recording_type && !recording_type->empty()) {
    fs::path type_dir = fs::path(*recording_type).filename();
    recordings_dir /= type_dir;
  }

  std::error_code ec;
  fs::create_directories(recordings_dir, ec);
  if (ec) {
    this->_logger.error("Failed to create recordings directory '%s': %s",
                        recordings_dir.string().c_str(), ec.message().c_str());
    throw std::runtime_error("Failed to create recordings directory");
  }

  fs::path user_path(user_input);
  std::string base_name = user_path.stem().string();
  if (base_name.empty()) {
    base_name = user_path.filename().string();
  }
  if (base_name.empty()) {
    base_name = "recording";
  }

  const auto now = std::chrono::system_clock::now();
  std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm tm_info{};
#ifdef _WIN32
  localtime_s(&tm_info, &now_time);
#else
  localtime_r(&now_time, &tm_info);
#endif

  std::ostringstream filename;
  filename << base_name << '-' << std::setfill('0') << std::setw(2)
           << tm_info.tm_mday << '-' << std::setw(2) << (tm_info.tm_mon + 1)
           << '-' << (tm_info.tm_year + 1900) << '-' << std::setw(2)
           << tm_info.tm_hour << '-' << std::setw(2) << tm_info.tm_min << '-'
           << std::setw(2) << tm_info.tm_sec << ".rrd";

  const fs::path final_path = recordings_dir / filename.str();
  this->_logger.info("Saving rerun recording to %s", final_path.string().c_str());
  return final_path.string();
}

Viewer::Viewer(Core::ArgumentParser args) : Core::Vertex(args), _args(args) {
  this->_rec = std::make_shared<rerun::RecordingStream>("Forerunner v2");
  const bool no_record = _args.get_argument<bool>("--no-record");
  const bool spawn_requested = _args.get_argument<bool>("--spawn");
  const auto recording_type = _args.present("--type");

  if (!no_record) {
    const auto recording_id = _args.get_argument<std::string>("--id");
    auto final_path = this->build_recording_path(recording_id, recording_type);
    auto error = this->_rec->save(final_path);
    if (error.is_err()) {
      this->_logger.error("Error while saving rerun streaming to file: %s",
                          error.description.c_str());
      throw error;
    }
  }

  if (auto url = _args.present("--grpc")) {
    auto error = this->_rec->connect_grpc();
    if (error.is_err()) {
      this->_logger.error(
          "Error while connecting to remote viewer '%s' streaming to file: %s",
          url->c_str(), error.description.c_str());
      throw error;
    }
  } else if (spawn_requested || no_record) {
    this->_rec->spawn().exit_on_failure();
  }
}

void Viewer::run() {
  this->_logger.info("Running");
  auto file_path = this->get_argument("--drone-model");
  this->_logger.debug("Drone model path: %s", file_path.c_str());
  this->_rec->log_static(
      "world", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);  // Set an up-axis
  this->_rec->log("world/drone",
                  rerun::Asset3D::from_file_path(file_path)
                      .value_or_throw());  // Set an up-axis
  while (true) sleep(1);
}

int main(int argc, char** argv) {
  Core::BaseArgumentParser args(argc, argv);
  auto root = Core::find_root();
  if (root.empty()) {
    root = ".";
  }
  if (root[root.size() - 1] != '/') {
    root.push_back('/');
  }
  std::string default_model_path = root + "rerun/assets/X500.glb";
  args.add_argument("--no-goal-sub")
      .default_value(false)
      .implicit_value(false)
      .help("Disable goal subscriber")
      .flag();
  args.add_argument("--no-octree-sub")
      .default_value(false)
      .implicit_value(false)
      .help("Disable octree subscriber")
      .flag();
  args.add_argument("--no-plan-sub")
      .default_value(false)
      .implicit_value(false)
      .help("Disable plan subscriber")
      .flag();
  args.add_argument("--no-map-sub")
      .default_value(false)
      .implicit_value(false)
      .help("Disable map subscriber")
      .flag();
  args.add_argument("--no-cloud-sub")
      .default_value(false)
      .implicit_value(false)
      .help("Disable cloud subscriber")
      .flag();
  args.add_argument("--drone-model")
      .default_value(default_model_path)
      .help("stl file to use for rendering the drone");
  args.add_argument("--map-compression")
      .default_value(false)
      .implicit_value(false)
      .help("Enable decompression of map chunks")
      .flag();
  args.add_argument("--id")
    .default_value(std::string("record"))
    .help("Identifier used to name the recording file");
  args.add_argument("--type")
    .help("Subdirectory under rerun/recordings to store the recording");
  args.add_argument("--no-record")
    .default_value(false)
    .implicit_value(true)
    .help("Disable recording output")
    .flag();
  args.add_argument("--no-render")
      .default_value(false)
      .implicit_value(true)
      .help("Disable point subscriber")
      .flag();
  auto& group = args.add_mutually_exclusive_group();
  group.add_argument("--grpc").help("URL of remote viewer");
  group.add_argument("--spawn")
      .help("Start or stream to already opened local viewer")
      .flag();
  auto& filter_list = args.add_mutually_exclusive_group();
  filter_list.add_argument("--whitelist")
    .help("Whitelist of topics, comma-separated")
    .default_value("");
  filter_list.add_argument("--blacklist")
    .help("Blacklist of topics, comma-separated")
    .default_value("");

  auto viewer = Viewer(args);
  std::unique_ptr<RegistryExample::Renderer> renderer;
  if (!args.get_argument<bool>("--no-render")) {
    renderer = std::make_unique<RegistryExample::Renderer>(&viewer);
    // TODO: Pasar vertex de viewer a render?
  }
  viewer.run();
  return 0;
}
