#ifndef DETECTION
#define DETECTION

#include "argument_parser.hpp"
#include "camera-tracking.hpp"
#include "message.hpp"
#include "publisher.hpp"
#include "subscriber.hpp"
#include "vertex.hpp"
#include <algorithm>
#include <capnp_schemas/detection_messages.capnp.h>
#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <future>
#include <iostream>
#include <numeric>
#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <regex>
#include <sl/Camera.hpp>
#include <string>
#include <unordered_set>
#include <vector>

// Structure to hold detection result with coordinates
struct DetectionResult {
  bool valid_person_detected;
  std::string coordinates;

  DetectionResult() : valid_person_detected(false), coordinates("") {}
  DetectionResult(bool valid, const std::string& coords)
      : valid_person_detected(valid), coordinates(coords) {}
};

// Structure to store valid person coordinates
struct PersonLocation {
  float x, y, z;
  std::chrono::steady_clock::time_point detection_time;

  PersonLocation(float x_pos, float y_pos, float z_pos)
      : x(x_pos),
        y(y_pos),
        z(z_pos),
        detection_time(std::chrono::steady_clock::now()) {}
};

class Detection : public Core::Vertex {
 private:
  std::shared_ptr<Core::Publisher<DetectionImage>> _detection_pub;
  std::shared_ptr<Core::Publisher<ImageData>> _zed_image_pub;
  std::shared_ptr<Core::Subscriber<LLMResult>> _result_sub;
  std::vector<PersonLocation> _detected_persons;
  std::string _description;
  std::string _svo_path;
  float _distance_threshold;
  uint32_t _next_object_id;
  cv::Mat _latest_frame;
  cv::Mat _latest_rgb;
  cv::Mat _latest_depth;
  std::mutex _depth_frame_mtx;
  std::mutex _rgb_frame_mtx;

  cv::dnn::Net _yolo_net;
  std::string _yolo_model_path;
  bool _yolo_use_cuda = false;

 public:
  Detection(Core::ArgumentParser);
  void handle_llm_result(const Core::IncomingMessage<LLMResult>&);
  void run_detection();
  void publish_zed_image(const cv::Mat&);

 private:
  void send_detection_image(const cv::Mat&, float, float, float);
};

#endif  // DETECTION
