#include "detection.hpp"

// Function to calculate 3D distance between two points
float calculateDistance(const PersonLocation& loc1, float x2, float y2,
                        float z2) {
  float dx = loc1.x - x2;
  float dy = loc1.y - y2;
  float dz = loc1.z - z2;
  return sqrt(dx * dx + dy * dy + dz * dz);
}

// Function to extract coordinates as floats from filename
bool extractCoordinatesAsFloats(const std::string& filename, float& x, float& y,
                                float& z) {
  std::regex coord_regex(R"(x([-\d\.]+)_y([-\d\.]+)_z([-\d\.]+)_)");
  std::smatch matches;

  if (std::regex_search(filename, matches, coord_regex)) {
    x = std::stof(matches[1].str());
    y = std::stof(matches[2].str());
    z = std::stof(matches[3].str());
    return true;
  }
  return false;
}

// Function to extract coordinates from filename
std::string extractCoordinatesFromFilename(const std::string& filename) {
  std::regex coord_regex(R"(x([-\d\.]+)_y([-\d\.]+)_z([-\d\.]+)_)");
  std::smatch matches;

  if (std::regex_search(filename, matches, coord_regex)) {
    return "(" + matches[1].str() + ", " + matches[2].str() + ", " +
           matches[3].str() + ")";
  }
  return "";
}

Detection::Detection(Core::ArgumentParser parser) : Core::Vertex(parser) {
  _description = this->get_argument<std::string>("--description");
  _svo_path = this->get_argument<std::string>("--svo-path");
  if (std::all_of(_svo_path.begin(), _svo_path.end(),
                  [](unsigned char ch) { return std::isspace(ch); })) {
    _svo_path.clear();
  }
  _distance_threshold = this->get_argument<double>("--distance-threshold");
  //_yolo_model_path = this->get_argument<std::string>("--yolo-model");
  //_yolo_net = loadYoloModel(_yolo_model_path, _yolo_use_cuda);
  _next_object_id = 1;

  // Setup communication
  _detection_pub = this->create_publisher<DetectionImage>("detection_images");
  _zed_image_pub = this->create_publisher<ImageData>("zed_image");
  _result_sub = this->create_subscriber<LLMResult>(
      "llm_results",
      std::bind(&Detection::handle_llm_result, this, std::placeholders::_1));

  this->_logger.info("Detector initialized with description: %s",
                     _description.c_str());
}

void Detection::handle_llm_result(const Core::IncomingMessage<LLMResult>& msg) {
  auto result = msg.content;
  auto coordinates = result.getCoordinates();

  if (result.getIsValidPerson()) {
    this->_logger.info(
        "Valid person detected at coordinates (%.3f, %.3f, %.3f)",
        coordinates.getX(), coordinates.getY(), coordinates.getZ());

    // Add to detected persons
    _detected_persons.emplace_back(coordinates.getX(), coordinates.getY(),
                                   coordinates.getZ());
  } else {
    this->_logger.info("Detection ID %u: NOT a valid person",
                       result.getObjectId());
  }
}

void Detection::publish_zed_image(const cv::Mat& image) {
  if (!_zed_image_pub) return;
  if (image.empty()) {
    this->_logger.debug("Skipping publish of empty ZED frame");
    return;
  }
  try {
    std::vector<uchar> buffer;
    const std::vector<int> jpeg_params = {cv::IMWRITE_JPEG_QUALITY, 50};
    cv::imencode(".jpg", image, buffer, jpeg_params);

    auto msg = _zed_image_pub->new_msg();
    auto image_data = msg.content;
    image_data.setWidth(image.cols);
    image_data.setHeight(image.rows);
    image_data.setType(static_cast<uint32_t>(image.type()));
    image_data.setData(kj::arrayPtr(buffer.data(), buffer.size()));

    msg.publish();
    this->_logger.debug("Published raw ZED frame (%dx%d)", image.cols,
                        image.rows);
  } catch (const std::exception& e) {
    this->_logger.error("Error publishing ZED frame: %s", e.what());
  }
}

void Detection::run_detection() {
  this->_logger.info("Starting detection loop...");
  if (_svo_path.empty()) {
    this->_logger.info("Using live ZED camera input");
  } else {
    this->_logger.info("Using SVO file: %s", _svo_path.c_str());
  }
  std::unordered_set<int> saved_ids;

  // Create a ZED camera object
  sl::Camera zed;
  initZed(zed, _svo_path, _logger);
  _logger.info("ZED camera initialized.");

  // Define the Objects detection module parameters
  sl::ObjectDetectionParameters detection_parameters;
  initDetectionParameters(zed, detection_parameters, _logger);
  sl::ObjectDetectionRuntimeParameters detection_parameters_rt;
  setObjectDetectionRuntimeParameters(zed, detection_parameters_rt, _logger);
  _logger.info("Object detection module initialized.");

  // detection output
  sl::Objects objects;
  float depth_value = 0.0f;
  bool save_image = true;
  list<cv::Mat> images;
  auto timer = std::chrono::high_resolution_clock::now();
  const auto publish_frame = [this](const cv::Mat& frame) {
    this->publish_zed_image(frame);
  };
  _logger.info("Entering main detection loop...");
  while (true) {
    auto grab_status = zed.grab();
    if (grab_status == sl::ERROR_CODE::SUCCESS) {
      processDetections(zed, detection_parameters_rt, objects, saved_ids,
                        depth_value, images, save_image, _logger,
                        publish_frame);
      // DEBUG: Log how many images were detected
      if (!images.empty()) {
        this->_logger.debug("Found %zu detected images", images.size());
      } else {
        // DEBUG: Log every 100 frames that no detection found (to avoid spam)
        static int no_detection_counter = 0;
        no_detection_counter++;
        if (no_detection_counter % 100 == 0) {
          this->_logger.debug("No detections found (frame %d)",
                              no_detection_counter);
        }
      }

      // Process detected images - send via publisher instead of saving
      if (!images.empty()) {
        for (auto& image : images) {
          this->_logger.debug("Processing image %dx%d", image.cols, image.rows);

          // Create coordinates string (using depth_value as rough estimate)
          std::string coordinates = "x" + std::to_string(depth_value * 0.1f) +
                                    "_y0.0_z" + std::to_string(depth_value) +
                                    "_";

          this->_logger.debug("Created coordinates: %s", coordinates.c_str());

          // Check if too close to existing detection
          float x, y, z;
          if (extractCoordinatesAsFloats(coordinates, x, y, z)) {
            this->_logger.debug("Extracted coordinates: x=%.2f, y=%.2f, z=%.2f",
                                x, y, z);

            bool too_close = false;
            for (const auto& person : _detected_persons) {
              if (calculateDistance(person, x, y, z) < _distance_threshold) {
                too_close = true;
                this->_logger.debug("Too close to existing person");
                break;
              }
            }

            if (!too_close) {
              this->_logger.debug("Sending detection image...");
              send_detection_image(image, x, y, z);
            } else {
              this->_logger.debug("Skipping - too close to existing detection");
            }
          }
        }
        _next_object_id = 0;
        images.clear();
      }

    } else if (grab_status == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
      this->_logger.info("SVO file reached end. Exiting...");
      break;
    } else {
      this->_logger.error("Error during camera grab: %d",
                          static_cast<int>(grab_status));
      break;
    }

    if (std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::high_resolution_clock::now() - timer)
            .count() >= 5) {
      saved_ids.clear();
      timer = std::chrono::high_resolution_clock::now();
    }
  }

  zed.disableObjectDetection();
  zed.close();
}

void Detection::send_detection_image(const cv::Mat& image, float x, float y,
                                     float z) {
  this->_logger.debug(
      "send_detection_image called with %dx%d image, coordinates: x=%.3f, "
      "y=%.3f, z=%.3f",
      image.cols, image.rows, x, y, z);
  try {
    // Encode image to JPEG
    std::vector<uchar> buffer;
    cv::imencode(".jpg", image, buffer);

    // Create message
    auto msg = _detection_pub->new_msg();

    // Fill image data
    auto image_data = msg.content.initImage();
    image_data.setWidth(image.cols);
    image_data.setHeight(image.rows);
    image_data.setType(CV_8UC3);  // OpenCV type for 3-channel 8-bit image
    image_data.setData(kj::arrayPtr(buffer.data(), buffer.size()));

    // Fill detection data with Point structure
    auto coordinates = msg.content.initCoordinates();
    coordinates.setX(x);
    coordinates.setY(z);
    coordinates.setZ(y);

    msg.content.setDescription(_description);
    msg.content.setObjectId(_next_object_id++);

    // Publish
    msg.publish();

    this->_logger.info(
        "Successfully sent detection image ID %u with coordinates (%.3f, "
        "%.3f, %.3f)",
        msg.content.getObjectId(), x, y, z);

  } catch (const std::exception& e) {
    this->_logger.error("Error sending detection image: %s", e.what());
  }
}

int main(int argc, char** argv) {
  Core::BaseArgumentParser parser(argc, argv);
  parser.add_argument("--description")
      .required()
      .help("Description of the person/object to detect");
  parser.add_argument("--svo-path")
      .default_value(std::string())
      .help(
          "Path to the SVO2 file for ZED camera; omit to use the live camera");
  parser.add_argument("--distance-threshold")
      .default_value(0.5)
      .scan<'g', double>()
      .help(
          "Distance threshold to consider detections as the same person "
          "(meters)");
  try {
    std::shared_ptr<Detection> detector = std::make_shared<Detection>(parser);
    detector->run_detection();
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
