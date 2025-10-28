// Standard includes
#include <cmath>
#include <fstream>
#include <iostream>

// ZED includes
#include "camera-tracking.hpp"  // Declares helper functions
#include <opencv2/opencv.hpp>   // OpenCV header
#include <sl/Camera.hpp>
#include <unordered_set>

// Using std and sl namespaces
using namespace std;

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

cv::Mat slMat2cvMat(sl::Mat& input) {
  // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer
  // from sl::Mat (getPtr<T>()) cv::Mat and sl::Mat will share a single memory
  // structure
  return cv::Mat(
      input.getHeight(), input.getWidth(), getOCVtype(input.getDataType()),
      input.getPtr<sl::uchar1>(sl::MEM::CPU), input.getStepBytes(sl::MEM::CPU));
}

void initZed(sl::Camera& zed, const std::string& svo_path,
             Core::Logger& _logger) {
  _logger.debug("initZed...");
  // Set configuration parameters
  sl::InitParameters init_parameters;
  if (svo_path.empty()) {
    _logger.debug("Zed live camera...");
    // 
    init_parameters.svo_real_time_mode = true;  // <-- Important for offline processing
    init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;  // Use right-handed Y-up
                                                   // coordinate system
  } else{
    init_parameters.input.setFromSVOFile(svo_path.c_str());
  }

    _logger.debug("Zed parameters...");
    init_parameters.depth_mode = sl::DEPTH_MODE::NEURAL;  // Use NEURAL depth mode
     init_parameters.coordinate_units = sl::UNIT::METER; // Set coordinate units
     init_parameters.sdk_verbose = true; // Enable verbose mode
     init_parameters.depth_maximum_distance = 10*1000.f; // Set maximum depth distance
  

  // Open the camera

  auto returned_state = zed.open(init_parameters);
  _logger.debug("Zed opened...");
  if (returned_state != sl::ERROR_CODE::SUCCESS) {
    _logger.error("Error %d, exit program.", returned_state);
    exit(EXIT_FAILURE);
  }
}

void initDetectionParameters(
    sl::Camera& zed, sl::ObjectDetectionParameters& detection_parameters,
    Core::Logger& _logger) {
  // Define the Objects detection module parameters
  detection_parameters.enable_tracking = true;  // Enable tracking
  detection_parameters.enable_segmentation =
      true;  // Enable segmentation (to get person pixel mask)
  // detection_parameters.detection_model = OBJECT_DETECTION_MODEL::YOLO_V8_1M;
  // // Use YOLOv8 model
  if (detection_parameters.enable_tracking)
    zed.enablePositionalTracking();  // Enable positional tracking (required for
                                     // object tracking)
  _logger.info("Enabling object detection...");
  auto returned_state = zed.enableObjectDetection(detection_parameters);
  if (returned_state != sl::ERROR_CODE::SUCCESS) {
    _logger.error("Error enabling object detection: %d", returned_state);
    zed.close();
    exit(EXIT_FAILURE);
  }
}

void setObjectDetectionRuntimeParameters(
    sl::Camera& zed,
    sl::ObjectDetectionRuntimeParameters& detection_parameters_rt,
    Core::Logger& _logger) {
  detection_parameters_rt.detection_confidence_threshold = 60;
  zed.setObjectDetectionRuntimeParameters(
      detection_parameters_rt);  // Can be set at any time
  _logger.info("Object detection runtime parameters set.");
}

void getDepth(sl::Camera& zed, float& depth_value, cv::Rect roi) {
  sl::Resolution image_size =
      zed.getCameraInformation().camera_configuration.resolution;
  // Create a sl::Mat with float type (32-bit)
  sl::Mat depth_zed(image_size.width, image_size.height, sl::MAT_TYPE::F32_C1);
  zed.retrieveMeasure(depth_zed, sl::MEASURE::DEPTH);
  cv::Mat depth_ocv = slMat2cvMat(depth_zed);
  // Crop to the region of interest
  depth_ocv = depth_ocv(roi);

  // Print the depth value at the center of the image
  depth_value = depth_ocv.at<float>(depth_ocv.rows / 2, depth_ocv.cols / 2);
}

void processDetections(
    sl::Camera& zed,
    const sl::ObjectDetectionRuntimeParameters& detection_parameters_rt,
    sl::Objects& objects, std::unordered_set<int>& saved_ids,
    float& depth_value, list<cv::Mat>& images, bool save_image,
    Core::Logger& _logger,
    const std::function<void(const cv::Mat&)>& publish_frame) {
  // Control variable for 3D bounding box visualization
  // Set to true: Shows 3D bounding boxes (yellow wireframe) with 3D coordinates
  // Set to false: Shows 2D bounding boxes (green rectangles) with calculated
  // depth
  static bool show_3d_bbox = false;

  zed.retrieveObjects(objects);
//  _logger.debug("Number of objects detected: %d", static_cast<int>(objects.object_list.size()));
  if (objects.is_new && !objects.object_list.empty()) {
    sl::Mat zed_image;
    zed.retrieveImage(zed_image, sl::VIEW::LEFT, sl::MEM::CPU);
    cv::Mat cv_image = slMat2cvMat(zed_image);
    // Keep a copy of the original frame before drawing detections
    cv::Mat original_image = cv_image.clone();

    for (auto& obj : objects.object_list) {
      std::string depth_label = "Depth: N/A";
      std::string coords_3d_label = "";
      bool valid_roi = false;
      cv::Rect roi;

      if (show_3d_bbox) {
        // Validate object data before accessing to prevent segfaults
        if (obj.bounding_box.size() != 8) {
          continue;  // Skip objects with invalid bounding box data
        }

        // Use 3D bounding box and get 3D position
        auto bbox_3d = obj.bounding_box;
        auto position_3d = obj.position;

        // Validate 3D position data
        if (std::isnan(position_3d.x) || std::isnan(position_3d.y) ||
            std::isnan(position_3d.z)) {
          continue;  // Skip objects with invalid position data
        }

        // Get camera calibration parameters for 3D to 2D projection
        auto camera_info = zed.getCameraInformation();
        auto calibration =
            camera_info.camera_configuration.calibration_parameters.left_cam;

        // Project 3D bounding box corners to 2D
        std::vector<cv::Point2f> projected_points;
        for (int i = 0; i < 8; i++) {
          // Validate each 3D point before accessing to prevent crashes
          const auto& point_3d = bbox_3d[i];
          if (std::isnan(point_3d.x) || std::isnan(point_3d.y) ||
              std::isnan(point_3d.z)) {
            continue;  // Skip invalid 3D points
          }

          float x_3d = point_3d.x;
          float y_3d = point_3d.y;
          float z_3d = point_3d.z;

          if (z_3d > 0) {  // Only project points in front of camera
            float x_2d = (x_3d * calibration.fx / z_3d) + calibration.cx;
            float y_2d = (y_3d * calibration.fy / z_3d) + calibration.cy;

            // Validate projected coordinates
            if (std::isfinite(x_2d) && std::isfinite(y_2d)) {
              projected_points.push_back(cv::Point2f(x_2d, y_2d));
            }
          }
        }

        // Draw 3D bounding box edges
        if (projected_points.size() == 8) {
          // Define the 12 edges of a 3D bounding box
          std::vector<std::pair<int, int>> edges = {
              {0, 1}, {1, 2}, {2, 3}, {3, 0},  // Bottom face
              {4, 5}, {5, 6}, {6, 7}, {7, 4},  // Top face
              {0, 4}, {1, 5}, {2, 6}, {3, 7}   // Vertical edges
          };

          for (auto& edge : edges) {
            cv::line(cv_image,
                     cv::Point(projected_points[edge.first].x,
                               projected_points[edge.first].y),
                     cv::Point(projected_points[edge.second].x,
                               projected_points[edge.second].y),
                     cv::Scalar(0, 255, 255), 2);  // Yellow for 3D bbox
          }
        }

        // Create 2D ROI from projected 3D bbox for cropping
        if (!projected_points.empty()) {
          float min_x =
              std::min_element(projected_points.begin(), projected_points.end(),
                               [](const cv::Point2f& a, const cv::Point2f& b) {
                                 return a.x < b.x;
                               })
                  ->x;
          float max_x =
              std::max_element(projected_points.begin(), projected_points.end(),
                               [](const cv::Point2f& a, const cv::Point2f& b) {
                                 return a.x < b.x;
                               })
                  ->x;
          float min_y =
              std::min_element(projected_points.begin(), projected_points.end(),
                               [](const cv::Point2f& a, const cv::Point2f& b) {
                                 return a.y < b.y;
                               })
                  ->y;
          float max_y =
              std::max_element(projected_points.begin(), projected_points.end(),
                               [](const cv::Point2f& a, const cv::Point2f& b) {
                                 return a.y < b.y;
                               })
                  ->y;

          int x_min = std::max(0, (int)min_x);
          int y_min = std::max(0, (int)min_y);
          int x_max = std::min(cv_image.cols - 1, (int)max_x);
          int y_max = std::min(cv_image.rows - 1, (int)max_y);

          if (x_max > x_min && y_max > y_min) {
            roi = cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min);
            valid_roi = true;

            // Use 3D position for depth and coordinates
            depth_value = position_3d.z;
            depth_label = "Depth: " + std::to_string(depth_value) + " m";
            coords_3d_label = " 3D: (" + std::to_string(position_3d.x) + ", " +
                              std::to_string(position_3d.y) + ", " +
                              std::to_string(position_3d.z) + ")";
          }
        }
      } else {
        //_logger.debug("Drawing 2D bounding box for object ID: %d", obj.id);
        // Use 2D bounding box (original code)
        auto bbox = obj.bounding_box_2d;
        int x_min = std::min({bbox[0].x, bbox[1].x, bbox[2].x, bbox[3].x});
        int y_min = std::min({bbox[0].y, bbox[1].y, bbox[2].y, bbox[3].y});
        int x_max = std::max({bbox[0].x, bbox[1].x, bbox[2].x, bbox[3].x});
        int y_max = std::max({bbox[0].y, bbox[1].y, bbox[2].y, bbox[3].y});

        int x_min2 = std::max(0, x_min);
        int y_min2 = std::max(0, y_min);
        int x_max2 = std::min(cv_image.cols - 1, x_max);
        int y_max2 = std::min(cv_image.rows - 1, y_max);

        valid_roi = (x_max2 > x_min2 && y_max2 > y_min2);

        if (valid_roi) {
          roi = cv::Rect(x_min2, y_min2, x_max2 - x_min2, y_max2 - y_min2);
          getDepth(zed, depth_value, roi);
          depth_label = "Depth: " + std::to_string(depth_value) + " m";

          // Draw 2D bounding box
          cv::rectangle(cv_image, cv::Point(x_min, y_min),
                        cv::Point(x_max, y_max), cv::Scalar(0, 255, 0), 2);
        }
      }

      // Display label with ID, confidence, depth, and 3D coordinates (if 3D
      // mode)
      std::string label = "ID:" + std::to_string(obj.id) +
                          " conf:" + std::to_string(obj.confidence) + " " +
                          depth_label + coords_3d_label;

      // Position label above the bounding box
      cv::Point label_pos;
      if (valid_roi) {
        label_pos = cv::Point(roi.x, roi.y - 5);
      } else {
        label_pos = cv::Point(50, 50);  // Fallback position
      }

      cv::putText(cv_image, label, label_pos, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  cv::Scalar(0, 255, 0), 2);

      bool valid_person =
          (obj.confidence >=
               detection_parameters_rt.detection_confidence_threshold &&
           obj.label == sl::OBJECT_CLASS::PERSON &&
           saved_ids.find(obj.id) == saved_ids.end());

      if (valid_roi && valid_person) {
        // Crop from the original (undrawn) frame so saved images don't include
        // overlays
        cv::Mat cropped = original_image(roi).clone();
        if (save_image) {
          // Include 3D coordinates in filename for later retrieval
          std::string filename;
          if (show_3d_bbox) {
            auto position_3d = obj.position;
            filename = "../images/det_" + std::to_string(obj.id) + "_x" +
                       std::to_string(position_3d.x) + "_y" +
                       std::to_string(position_3d.y) + "_z" +
                       std::to_string(position_3d.z) + "_obj.jpg";
          } else {
            filename = "../images/det_" + std::to_string(obj.id) + "_obj.jpg";
          }
          cv::imwrite(filename, cropped);
          images.push_back(cropped);
        }
        saved_ids.insert(obj.id);
      }
    }
    cv::resize(cv_image, cv_image, cv::Size(), 0.7, 0.7);
    if (publish_frame) {
      static auto last_publish = std::chrono::steady_clock::now();
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::seconds>(now - last_publish)
              .count() >= 1) {
        publish_frame(cv_image);
        last_publish = now;
      }
    }
    cv::waitKey(1);
  }
}
