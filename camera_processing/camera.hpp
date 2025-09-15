#include "argument_parser.hpp"
#include <opencv2/opencv.hpp>
#include "publisher.hpp"
#include "vertex.hpp"
#include <sl/Camera.hpp>
#include <memory>
#include "../messages/build/generated/sensors.capnp.h"

#ifndef CAMERA_HPP
#define CAMERA_HPP

class PeopleDetector : Core::Vertex {
 private:
  sl::Camera zed;
  std::shared_ptr<Core::Publisher<ImageData>> _people_image_pub;  // use Reader
 public:
  PeopleDetector(const Core::ArgumentParser&);
  void run(std::string svo2_path);
  void ftp_upload(std::list<cv::Mat>& images, std::unordered_set<int>& ids);
};

#endif
