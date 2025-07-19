#include "zed.hpp"
#include <capnp_schemas/zed.capnp.h>
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

Zed::Zed(int argc, char **argv) : Core::Vertex(argc, argv) {
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

      auto ptr = point_cloud.getPtr<float>();
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
          new pcl::PointCloud<pcl::PointXYZRGBA>(default_image_size.height,
                                                 default_image_size.width));
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
    }
  }
}

int main(int argc, char **argv) {
  auto zed = Zed(argc, argv);
  zed.run();

  return 0;
}
