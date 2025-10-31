#ifndef CAMERA_TRACKING_HPP
#define CAMERA_TRACKING_HPP

#include "logger.hpp"
#include <functional>
#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>
#include <string>
#include <unordered_set>

using namespace std;

// Converts sl::MAT_TYPE to OpenCV type
int getOCVtype(sl::MAT_TYPE type);

// Converts sl::Mat to cv::Mat
cv::Mat slMat2cvMat(sl::Mat& input);

// Initializes the ZED camera
void initZed(sl::Camera& zed, const std::string& svo_path,
             Core::Logger& _logger);

// Initializes object detection parameters
void initDetectionParameters(
    sl::Camera& zed, sl::ObjectDetectionParameters& detection_parameters,
    Core::Logger& _logger);

// Processes detected objects and draws bounding boxes on the image
void processDetections(
    sl::Camera& zed,
    const sl::ObjectDetectionRuntimeParameters& detection_parameters_rt,
    sl::Objects& objects, std::unordered_set<int>& saved_ids,
    float& depth_value, list<cv::Mat>& images, bool save_image,
    Core::Logger& _logger,
    const std::function<void(const cv::Mat&)>& publish_frame = {});

void setObjectDetectionRuntimeParameters(
    sl::Camera& zed,
    sl::ObjectDetectionRuntimeParameters& detection_parameters_rt,
    Core::Logger& _logger);

void getDepth(sl::Camera& zed, float& depth_value, cv::Rect roi);

#endif  // CAMERA_TRACKING_HPP
