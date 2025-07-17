#include "zed.hpp"
#include <capnp_schemas/zed.capnp.h>
#include <functional>
#include <pthread.h>
#include <sl/Camera.hpp>

using namespace sl;

Zed::Zed(int argc, char **argv) : Core::Vertex(argc, argv) {
  this->_cloud_point_pub = this->create_publisher<PointCloud>("point_cloud");

  _camera = Camera();

  InitParameters init_parameters;
  init_parameters.depth_mode = DEPTH_MODE::NEURAL;
  init_parameters.coordinate_units = UNIT::MILLIMETER;
  init_parameters.input.setFromStream("127.0.0.1", 30000);

  auto returned_state = _camera.open(init_parameters);
  if (returned_state != ERROR_CODE::SUCCESS) {
    _camera.close();
    std::cout << returned_state << std::endl;
    throw std::runtime_error("zed camera can't start");
  }
}

void Zed::run() {
  sl::Resolution default_image_size = _camera.getRetrieveMeasureResolution();
  // without altering dimensions:
  // default_image_size.width = 720;
  // default_image_size.height = 480;
  sl::Mat image, depth, point_cloud;
  while (true) {
    if (_camera.grab() == ERROR_CODE::SUCCESS) {
      // get cloud point image
      _camera.retrieveMeasure(point_cloud, MEASURE::XYZ, MEM::CPU,
                              default_image_size);
      auto msg = this->_cloud_point_pub->new_msg();
      msg.content.setWidth(default_image_size.width);
      msg.content.setHeight(default_image_size.height);
      // std::cout << default_image_size.width << std::endl;
      // std::cout << default_image_size.height << std::endl;
      // std::cout << point_cloud.getPixelBytes() * default_image_size.area()
      //           << std::endl;
      // for (size_t i = 0; i < default_image_size.width; ++i) {
      //   for (size_t j = 0; j < default_image_size.height; ++j) {
      //     sl::float4 point_value;
      //     point_cloud.getValue(i, j, &point_value);
      //     std::cout << point_value.size() << std::endl;
      //   }
      // }

      auto data = point_cloud.getPtr<uint8_t>();
      auto size = point_cloud.getPixelBytes() * default_image_size.area();
      msg.content.initData(size);
      msg.content.setSize(size);
      auto reader = ::capnp::Data::Reader(data, size);
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
