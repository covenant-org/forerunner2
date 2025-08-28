#pragma once

#include <opencv2/opencv.hpp>
#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>
#include <mutex>
#include <atomic>

class CameraViewer {
 public:
  CameraViewer();
  ~CameraViewer();
  bool start(cv::dnn::Net &net, const std::vector<std::string> &class_list);
  void stop();

 private:
  void onImage(const gz::msgs::Image &msg,
               const gz::transport::MessageInfo &info);

  gz::transport::Node _node;
  std::mutex _frame_mtx;
  cv::Mat _latest_frame;
  std::atomic<bool> _running{false};
};