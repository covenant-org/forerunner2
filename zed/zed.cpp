#include "argument_parser.hpp"
#include "zed.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <Eigen/src/Geometry/Transform.h>
#include <capnp_schemas/zed.capnp.h>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <pcl/common/point_tests.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pthread.h>
#include <sl/Camera.hpp>
#include <sstream>
#include <unistd.h>
#include <vector>
#include <zlib.h>

Zed::Zed(const Core::ArgumentParser &parser) : Core::Vertex(parser) {
  this->_cloud_point_pub = this->create_publisher<PointCloud>("point_cloud");

  _camera = sl::Camera();

  sl::InitParameters init_parameters;
  init_parameters.depth_mode = sl::DEPTH_MODE::NEURAL;
  init_parameters.coordinate_units = sl::UNIT::METER;

  if (auto fn = parser.present("--svo")) {
    init_parameters.input.setFromSVOFile(fn->c_str());
  }

  if (auto fn = parser.present("--host")) {
    unsigned int a, b, c, d, port;
    if (sscanf(fn->c_str(), "%u.%u.%u.%u:%d", &a, &b, &c, &d, &port) == 5) {
      // Stream input mode - IP + port
      std::string ip_adress = std::to_string(a) + "." + std::to_string(b) +
                              "." + std::to_string(c) + "." + std::to_string(d);
      init_parameters.input.setFromStream(sl::String(ip_adress.c_str()), port);
    } else if (sscanf(fn->c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) == 4) {
      // Stream input mode - IP only
      init_parameters.input.setFromStream(sl::String(fn->c_str()));
    }
  }

  auto returned_state = _camera.open(init_parameters);
  if (returned_state != sl::ERROR_CODE::SUCCESS) {
    _camera.close();
    std::cout << returned_state << std::endl;
    throw std::runtime_error("zed camera can't start");
  }

  if (parser.get_argument<bool>("--map")) {
    sl::PositionalTrackingParameters ptp;
    ptp.mode = sl::POSITIONAL_TRACKING_MODE::GEN_3;
    returned_state = _camera.enablePositionalTracking(ptp);
    if (returned_state > sl::ERROR_CODE::SUCCESS) {
      _camera.close();
      throw std::runtime_error("zed camera can't enable positional tracking");
    }
    this->_map_pub = this->create_publisher<PointCloudChunk>("map");
  }

  pcl::io::compression_Profiles_e compressionProfile =
      pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
  _cloud_encoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>(
      compressionProfile, false);
}

void Zed::run() {
  sl::Pose pose;
  sl::FusedPointCloud map;
  sl::RuntimeParameters runtime_parameters;
  runtime_parameters.confidence_threshold = 30;
  sl::POSITIONAL_TRACKING_STATE tracking_state =
      sl::POSITIONAL_TRACKING_STATE::OFF;
  bool wait_for_mapping = true;
  bool request_new_mesh = true;

  sl::SpatialMappingParameters spatial_mapping_parameters;
  spatial_mapping_parameters.map_type =
      sl::SpatialMappingParameters::SPATIAL_MAP_TYPE::FUSED_POINT_CLOUD;
  // Set mapping range, it will set the resolution accordingly (a higher range,
  // a lower resolution)
  spatial_mapping_parameters.set(
      sl::SpatialMappingParameters::MAPPING_RESOLUTION::MEDIUM);
  spatial_mapping_parameters.set(
      sl::SpatialMappingParameters::MAPPING_RANGE::MEDIUM);
  // Request partial updates only (only the last updated chunks need to be
  // re-draw)
  spatial_mapping_parameters.use_chunk_only = true;
  // Stability counter defines how many times a stable 3D points should be seen
  // before it is integrated into the spatial mapping
  spatial_mapping_parameters.stability_counter = 5;
  spatial_mapping_parameters.decay = 1;

  sl::Resolution default_image_size = _camera.getRetrieveMeasureResolution();
  sl::Mat point_cloud;

  std::chrono::high_resolution_clock::time_point ts_last;
  while (true) {
    if (_camera.grab(runtime_parameters) == sl::ERROR_CODE::SUCCESS) {
      // get cloud point image
      _camera.retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA, sl::MEM::CPU,
                              default_image_size);
      auto msg = this->_cloud_point_pub->new_msg();
      msg.content.setWidth(default_image_size.width);
      msg.content.setHeight(default_image_size.height);

      auto ptr = point_cloud.getPtr<float>();
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
          new pcl::PointCloud<pcl::PointXYZRGBA>(default_image_size.width,
                                                 default_image_size.height));
      cloud->width = default_image_size.width;
      cloud->height = default_image_size.height;

      memcpy(cloud->points.data(), point_cloud.getPtr<sl::float4>(),
             sizeof(sl::float4) * default_image_size.area());

      Eigen::Affine3f transform = Eigen::Affine3f::Identity();
      transform.rotate(Eigen::AngleAxisf(-M_PI_2f, Eigen::Vector3f::UnitX()));
      transform.rotate(Eigen::AngleAxisf(M_PI_2f, Eigen::Vector3f::UnitY()));
      pcl::transformPointCloud(*cloud, *cloud, transform);

      std::stringstream encoded_cloud;
      _cloud_encoder->encodePointCloud(cloud, encoded_cloud);

      auto buffer = encoded_cloud.str();
      msg.content.initData(buffer.size());
      msg.content.setSize(buffer.size());
      auto reader =
          ::capnp::Data::Reader((unsigned char *)buffer.data(), buffer.size());
      msg.content.setData(reader);
      msg.publish();

      if (!_args.get_argument<bool>("--map")) continue;

      tracking_state = _camera.getPosition(pose);
      if (tracking_state == sl::POSITIONAL_TRACKING_STATE::OK) {
        if (wait_for_mapping) {
          _camera.enableSpatialMapping(spatial_mapping_parameters);
          _logger.debug("Enabled spatial mapping");
          wait_for_mapping = false;
          continue;
        }

        if (request_new_mesh) {
          auto duration =
              std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::high_resolution_clock::now() - ts_last)
                  .count();
          if (duration > 1000) {
            _camera.requestSpatialMapAsync();
            _logger.debug("Requested new mesh");
            request_new_mesh = false;
          }
        }

        // If the point cloud is ready to be retrieved
        if (_camera.getSpatialMapRequestStatusAsync() ==
                sl::ERROR_CODE::SUCCESS &&
            !request_new_mesh) {
          _camera.retrieveSpatialMapAsync(map);
          _logger.debug("Retrieved map");
          request_new_mesh = true;
          ts_last = std::chrono::high_resolution_clock::now();

          for (size_t i = 0; i < map.chunks.size(); i++) {
            auto &chunk = map.chunks[i];
            if (!chunk.has_been_updated || chunk.vertices.size() == 0) continue;

            auto chunk_points_size = chunk.vertices.size();
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr chunk_cloud(
                new pcl::PointCloud<pcl::PointXYZRGBA>(chunk_points_size, 1));
            for (size_t i = 0; i < chunk_points_size; i++) {
              chunk_cloud->points[i].x = chunk.vertices[i][0];
              chunk_cloud->points[i].y = chunk.vertices[i][1];
              chunk_cloud->points[i].z = chunk.vertices[i][2];
              uint32_t color_uint = *(uint32_t *)&chunk.vertices[i][3];
              unsigned char *color_uchar = (unsigned char *)&color_uint;
              color_uint =
                  ((uint32_t)color_uchar[0] << 16 |
                   (uint32_t)color_uchar[1] << 8 | (uint32_t)color_uchar[2]);
              chunk_cloud->points[i].rgb =
                  *reinterpret_cast<float *>(&color_uint);
            }

            Eigen::Affine3f chunk_transform = Eigen::Affine3f::Identity();
            chunk_transform.rotate(
                Eigen::AngleAxisf(-M_PI_2f, Eigen::Vector3f::UnitX()));
            chunk_transform.rotate(
                Eigen::AngleAxisf(M_PI_2f, Eigen::Vector3f::UnitY()));
            pcl::transformPointCloud(*chunk_cloud, *chunk_cloud,
                                     chunk_transform);

            auto chunk_msg = this->_map_pub->new_msg();
            auto chunk_msg_cloud = chunk_msg.content.initCloud();
            chunk_msg_cloud.setWidth(chunk_points_size);
            chunk_msg_cloud.setHeight(1);

            auto message_size =
                chunk_cloud->points.size() * sizeof(pcl::PointXYZRGBA);
            auto compression_level =
                _args.get_argument<int>("--map-compression-level");
            unsigned char *chunk_data =
                reinterpret_cast<unsigned char *>(chunk_cloud->points.data());
            std::vector<Bytef> compressed(compressBound(message_size));
            if (compression_level > 0) {
              auto compression_size = compressed.size();
              auto res =
                  compress2(compressed.data(), &compression_size,
                            (const unsigned char *)chunk_cloud->points.data(),
                            message_size, compression_level);
              if (res != Z_OK) {
                _logger.error("Compression error");
                continue;
              }
              _logger.debug("Compressed %d to %d bytes", message_size,
                            compression_size);
              message_size = compression_size;
              chunk_data = compressed.data();
            }
            chunk_msg_cloud.initData(message_size);
            chunk_msg_cloud.setSize(message_size);
            auto chunk_reader = ::capnp::Data::Reader(chunk_data, message_size);
            chunk_msg_cloud.setData(chunk_reader);
            chunk_msg.content.setIndex(i);
            chunk_msg.publish();
            _logger.debug("Sent %d chunk with %d points", i, chunk_points_size);
          }
        }
      }
    }
  }
}

int main(int argc, char **argv) {
  Core::BaseArgumentParser arguments(argc, argv);
  arguments.add_argument("--map")
      .default_value(false)
      .implicit_value(true)
      .help("Spatial Mapping feature");
  arguments.add_argument("--map-compression-level")
      .default_value(9)
      .choices(0, 1, 2, 3, 4, 5, 6, 7, 8, 9)
      .help(
          "Level of compression. 9 is the best comrpession and 1 is for the "
          "best speed. 0 disable compression")
      .scan<'d', int>();
  auto &group = arguments.add_mutually_exclusive_group();
  group.add_argument("--svo").help("SVO Recording File Path");
  group.add_argument("--host").help("Host IP Streaming");
  auto zed = Zed(arguments);
  zed.run();

  return 0;
}
