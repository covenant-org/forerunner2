#include "zed.hpp"
#include <capnp_schemas/zed.capnp.h>
#include <cmath>
#include <cstring>
#include <pcl/common/point_tests.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pthread.h>
#include <sl/Camera.hpp>
#include <sstream>

using namespace sl;

Zed::Zed(int argc, char **argv) : Core::Vertex(argc, argv), viewer("Help") {
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

  pcl::io::compression_Profiles_e compressionProfile =
      pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
  _cloud_encoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>(
      compressionProfile, false);
}

void Zed::run() {
  sl::Resolution default_image_size = _camera.getRetrieveMeasureResolution();
  sl::Mat point_cloud;
  while (true) {
    if (_camera.grab() == ERROR_CODE::SUCCESS) {
      // get cloud point image
      _camera.retrieveMeasure(point_cloud, MEASURE::XYZRGBA, MEM::CPU,
                              default_image_size);
      auto msg = this->_cloud_point_pub->new_msg();
      msg.content.setWidth(default_image_size.width);
      msg.content.setHeight(default_image_size.height);

      pcl::PointXYZRGB point(0.0f, 0, 0);
      auto ptr = point_cloud.getPtr<float>();
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
          new pcl::PointCloud<pcl::PointXYZRGB>(
              default_image_size.height, default_image_size.width, point));
      cloud->width = default_image_size.width;
      cloud->height = default_image_size.height;
      for (int x = 0; x < default_image_size.width; x++) {
        for (int y = 0; y < default_image_size.height; y++) {
          sl::float4 pixel;
          point_cloud.getValue(x, y, &pixel);
          auto point = &cloud->points[y*default_image_size.width + x];
          point->r = 255;
          point->g = 255;
          point->b = 255;
          if(!std::isfinite(pixel.x) || !std::isfinite(pixel.y) || !std::isfinite(pixel.z)){
          point->x = 0;
          point->y = 0;
          point->z = 0;
            continue;
          }
          point->x = pixel.x;
          point->y = pixel.y;
          point->z = pixel.z;
          point->r = pixel.r;
          point->g = pixel.g;
          point->b = pixel.b;
        }
      }
      this->viewer.removeVisualizationCallable("cloud");
      this->viewer.showCloud(cloud);
      std::cout << "displayed" << std::endl;
      continue;
      //      memcpy(cloud->points.data(), point_cloud.getPtr<sl::float4>(),
      //             point_cloud.getPixelBytes() * default_image_size.area());

      //pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
      //sor.setInputCloud(cloud);
      //sor.setLeafSize(0.01f, 0.01f, 0.01f);
      //sor.filter(*cloud);

      //std::stringstream encoded_cloud;
      //_cloud_encoder->encodePointCloud(cloud, encoded_cloud);

      auto data = point_cloud.getPtr<uint8_t>();
      //auto size = point_cloud.getPixelBytes() * default_image_size.area();
      auto size = cloud->points.size() * sizeof(pcl::PointXYZRGBA);
      //auto buffer = encoded_cloud.str();
      msg.content.initData(size);
      msg.content.setSize(size);
      auto reader = ::capnp::Data::Reader(reinterpret_cast<unsigned char*>(cloud->points.data()), size);
      msg.content.setData(reader);
      msg.publish();
      std::cout << "publishing image" << std::endl;
    }
  }
}

int main(int argc, char **argv) {
  auto zed = Zed(argc, argv);
  zed.run();

  return 0;
}
