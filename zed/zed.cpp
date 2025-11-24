#include "argument_parser.hpp"
#include "zed.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <Eigen/src/Geometry/Transform.h>
#include <capnp_schemas/zed.capnp.h>
#include <capnp_schemas/sensors.capnp.h>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <opencv2/opencv.hpp>  // OpenCV header
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
#include <string>
#include <unistd.h>
#include <vector>
#include <zlib.h>

Zed::Zed(const Core::ArgumentParser &parser) : Core::Vertex(parser) {
  this->_cloud_point_pub = this->create_publisher<PointCloud>("point_cloud");
  this->_img_publisher = this->create_publisher<Image>("zed_image");

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
    this->_map_pub = this->create_publisher<PointCloudChunk>("map_chunk");
  }

  pcl::io::compression_Profiles_e compressionProfile =
      pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
  _cloud_encoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>(
      compressionProfile, false);
}

void Zed::publish_zed_image(const cv::Mat &image) {
  if (image.empty()) {
    this->_logger.debug("Skipping publish of empty ZED frame");
    return;
  }
  try {
    std::vector<uchar> buffer;
    const std::vector<int> jpeg_params = {cv::IMWRITE_JPEG_QUALITY, 50};
    cv::imencode(".jpg", image, buffer, jpeg_params);

    auto msg = _img_publisher->new_msg();
    auto image_data = msg.content;
    image_data.setWidth(image.cols);
    image_data.setHeight(image.rows);
    image_data.setType(static_cast<uint32_t>(image.type()));
    image_data.setData(kj::arrayPtr(buffer.data(), buffer.size()));

    msg.publish();
    this->_logger.debug("Published raw ZED frame (%dx%d)", image.cols,
                        image.rows);
  } catch (const std::exception &e) {
    this->_logger.error("Error publishing ZED frame: %s", e.what());
  }
}

int getOCVtype(sl::MAT_TYPE type) {
  int cv_type = -1;
  switch (type) {
    case sl::MAT_TYPE::F32_C1:
      cv_type = CV_32FC1;
      break;
    case sl::MAT_TYPE::F32_C2:
      cv_type = CV_32FC2;
      break;
    case sl::MAT_TYPE::F32_C3:
      cv_type = CV_32FC3;
      break;
    case sl::MAT_TYPE::F32_C4:
      cv_type = CV_32FC4;
      break;
    case sl::MAT_TYPE::U8_C1:
      cv_type = CV_8UC1;
      break;
    case sl::MAT_TYPE::U8_C2:
      cv_type = CV_8UC2;
      break;
    case sl::MAT_TYPE::U8_C3:
      cv_type = CV_8UC3;
      break;
    case sl::MAT_TYPE::U8_C4:
      cv_type = CV_8UC4;
      break;
    default:
      break;
  }
  return cv_type;
}

cv::Mat slMat2cvMat(sl::Mat &input) {
  // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer
  // from sl::Mat (getPtr<T>()) cv::Mat and sl::Mat will share a single memory
  // structure
  return cv::Mat(
      input.getHeight(), input.getWidth(), getOCVtype(input.getDataType()),
      input.getPtr<sl::uchar1>(sl::MEM::CPU), input.getStepBytes(sl::MEM::CPU));
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

      if (this->_args.present("--images-dir")) {
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(
                now - _last_published_frame)
                .count() >= 0.25) {
          _last_published_frame = now;
          sl::Mat zed_image;
          _camera.retrieveImage(zed_image, sl::VIEW::LEFT, sl::MEM::CPU);
          cv::Mat cv_image = slMat2cvMat(zed_image);
          std::string image_folder = this->_args.get_argument("--images-dir");
          std::string filename =
              image_folder + "/" +
              std::to_string(now.time_since_epoch().count()) + ".jpeg";
          cv::imwrite(filename, cv_image);
          this->publish_zed_image(cv_image);
        }
      }

      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
          new pcl::PointCloud<pcl::PointXYZRGBA>(default_image_size.width,
                                                 default_image_size.height));
      cloud->width = default_image_size.width;
      cloud->height = default_image_size.height;

      sl::float4 *data = point_cloud.getPtr<sl::float4>();
      for (size_t i = 0; i < point_cloud.getWidth() * point_cloud.getHeight();
           i++) {
        cloud->points[i].x = data[i].x;
        cloud->points[i].y = data[i].y;
        cloud->points[i].z = data[i].z;
        uint32_t color_uint = ((uint32_t)data[i].b << 16 |
                               (uint32_t)data[i].g << 8 | (uint32_t)data[i].r);
        cloud->points[i].rgb = *reinterpret_cast<float *>(&color_uint);
      }

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
                  ((uint32_t)color_uchar[2] << 16 |
                   (uint32_t)color_uchar[1] << 8 | (uint32_t)color_uchar[0]);
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
  arguments.add_argument("--images-dir").help("Directory to store frames");
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
