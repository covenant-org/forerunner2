#include "argument_parser.hpp"
#include "zed.hpp"
#include <capnp_schemas/zed.capnp.h>
#include <chrono>
#include <cmath>
#include <cstring>
#include <pcl/common/point_tests.h>
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

using namespace sl;

Zed::Zed(const Core::ArgumentParser &parser) : Core::Vertex(parser) {
  this->_cloud_point_pub = this->create_publisher<PointCloud>("point_cloud");

  _camera = Camera();

  InitParameters init_parameters;
  init_parameters.depth_mode = DEPTH_MODE::NEURAL;
  init_parameters.coordinate_units = UNIT::METER;
  init_parameters.input.setFromStream("127.0.0.1", 30000);

  auto returned_state = _camera.open(init_parameters);
  if (returned_state != ERROR_CODE::SUCCESS) {
    _camera.close();
    std::cout << returned_state << std::endl;
    throw std::runtime_error("zed camera can't start");
  }

  if (parser.get_argument<bool>("--map")) {
    sl::PositionalTrackingParameters ptp;
    ptp.mode = sl::POSITIONAL_TRACKING_MODE::GEN_3;
    returned_state = _camera.enablePositionalTracking(ptp);
    if (returned_state > ERROR_CODE::SUCCESS) {
      _camera.close();
      throw std::runtime_error("zed camera can't enable positional tracking");
    }
    this->_map_pub = this->create_publisher<PointCloud>("map");
  }

  pcl::io::compression_Profiles_e compressionProfile =
      pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
  _cloud_encoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>(
      compressionProfile, false);
  _map_encoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>(
      compressionProfile, false);
}

void Zed::run() {
  Pose pose;
  FusedPointCloud map;
  RuntimeParameters runtime_parameters;
  runtime_parameters.confidence_threshold = 30;
  POSITIONAL_TRACKING_STATE tracking_state = POSITIONAL_TRACKING_STATE::OFF;
  bool wait_for_mapping = true;
  bool request_new_mesh = true;

  SpatialMappingParameters spatial_mapping_parameters;
  spatial_mapping_parameters.map_type =
      SpatialMappingParameters::SPATIAL_MAP_TYPE::FUSED_POINT_CLOUD;
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
  spatial_mapping_parameters.stability_counter = 10;
  spatial_mapping_parameters.decay = 0.1;

  sl::Resolution default_image_size = _camera.getRetrieveMeasureResolution();
  sl::Mat point_cloud;

  std::chrono::high_resolution_clock::time_point ts_last;
  while (true) {
    if (_camera.grab(runtime_parameters) == ERROR_CODE::SUCCESS) {
      // get cloud point image
      _camera.retrieveMeasure(point_cloud, MEASURE::XYZRGBA, MEM::CPU,
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

      pcl::PassThrough<pcl::PointXYZRGBA> pass;
      pass.setInputCloud(cloud);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(0.0, 10.0);
      pass.filter(*cloud);

      std::stringstream encoded_cloud;
      _cloud_encoder->encodePointCloud(cloud, encoded_cloud);

      auto buffer = encoded_cloud.str();
      msg.content.initData(buffer.size());
      msg.content.setSize(buffer.size());
      auto reader =
          ::capnp::Data::Reader((unsigned char *)buffer.data(), buffer.size());
      msg.content.setData(reader);
      msg.publish();

      if (!parser.get_argument<bool>("--map")) continue;

      tracking_state = _camera.getPosition(pose);
      if (tracking_state == POSITIONAL_TRACKING_STATE::OK) {
        if (wait_for_mapping) {
          _camera.enableSpatialMapping(spatial_mapping_parameters);
          wait_for_mapping = false;
          continue;
        }

        if (request_new_mesh) {
          auto duration =
              std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::high_resolution_clock::now() - ts_last)
                  .count();
          if (duration > 100) {
            _camera.requestSpatialMapAsync();
            request_new_mesh = false;
          }
        }

        // If the point cloud is ready to be retrieved
        if (_camera.getSpatialMapRequestStatusAsync() == ERROR_CODE::SUCCESS &&
            !request_new_mesh) {
          _camera.retrieveSpatialMapAsync(map);
          request_new_mesh = true;
          ts_last = std::chrono::high_resolution_clock::now();
        }
        auto points = map.getNumberOfPoints();
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr map_cloud(
            new pcl::PointCloud<pcl::PointXYZRGBA>(points, 1));
        memcpy(map_cloud->points.data(), map.vertices.data(),
               sizeof(sl::float4) * points);
        pcl::PassThrough<pcl::PointXYZRGBA> pass;
        pass.setInputCloud(map_cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, 10.0);
        pass.filter(*map_cloud);
        auto msg = this->_cloud_point_pub->new_msg();
        msg.content.setWidth(points);
        msg.content.setHeight(1);
        std::stringstream encoded_cloud;
        _map_encoder->encodePointCloud(map_cloud, encoded_cloud);
        auto buffer = encoded_cloud.str();
        msg.content.initData(buffer.size());
        msg.content.setSize(buffer.size());
        auto reader = ::capnp::Data::Reader((unsigned char *)buffer.data(),
                                            buffer.size());
        msg.content.setData(reader);
        msg.publish();
      }
    }
  }
}

int main(int argc, char **argv) {
  Core::BaseArgumentParser arguments(argc, argv);
  arguments.add_argument("--map")
      .default_value(false)
      .implicit_value(true)
      .help("Start requesting maping feature from zed");
  auto zed = Zed(arguments);
  zed.run();

  return 0;
}
