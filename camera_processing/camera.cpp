#include "argument_parser.hpp"
#include <sl/Camera.hpp>
#include "camera.hpp"
#include <opencv2/opencv.hpp>
#include "../people-detection/src/camera_tracking/camera_tracking.hpp"
#include "logger.hpp"

PeopleDetector::PeopleDetector(const Core::ArgumentParser& parser)
    : Core::Vertex(parser) {
    this->_logger.info("PeopleDetector: Base class Core::Vertex initialized");
    try {
        this->_logger.info("PeopleDetector: Creating publisher instance...");
        this->_people_image_pub = this->create_publisher<DetectionRawImage>("people_image");
        this->_logger.info("PeopleDetector: Publisher created successfully.");
        // ...existing code...
    } catch (const std::exception& e) {
        this->_logger.error("PeopleDetector: Failed to create publisher: %s", e.what());
        throw;
    }
}

void PeopleDetector::image_publisher(std::list<cv::Mat>& images, std::unordered_set<int>& ids) {
    // Implementation for FTP upload
    
    for(int i = 0; i < images.size(); i++) {
        try {
                std::vector<uchar> buf;
                cv::Mat image = images.front();
                cv::imencode(".jpg", image, buf);
                // Upload buf to FTP server
                auto msg = this->_people_image_pub->new_msg();
                auto id_ = ids.begin();
                std::advance(id_, i);
                auto id = *id_;
                msg.content.setId(id);
                msg.content.setWidth(image.cols);
                msg.content.setHeight(image.rows);
                msg.content.setType(image.type());  // OpenCV type
                
                auto reader = ::capnp::Data::Reader(buf.data(), buf.size());
                msg.content.setData(reader);

                msg.publish();

        } catch (const std::exception& e) {
            this->_logger.error("image_publisher exception: %s", e.what());
        }

    }
}



void PeopleDetector::run(std::string svo2_path) {
    this->_people_image_pub = this->create_publisher<DetectionRawImage>("people_image");

    std::unordered_set<int> saved_ids;
    this->_logger.info("SVO2 file path: %s", svo2_path.c_str());
    
    initZed(zed, svo2_path);
    ObjectDetectionParameters detection_parameters;
    ObjectDetectionRuntimeParameters detection_parameters_rt;  

    // run detection for every Camera grab
    initDetectionParameters(zed, detection_parameters);
    setObjectDetectionRuntimeParameters(zed, detection_parameters_rt);

    Objects objects;
    float depth_value = 0.0f;
    std::list<cv::Mat> images;
    auto start_time = std::chrono::steady_clock::now();
    bool save_image = false;
    this->_logger.info("Starting main loop...");

    while (true) {
        auto grab_result = zed.grab();
        if (grab_result == ERROR_CODE::SUCCESS) {
            processDetections(zed, detection_parameters_rt, objects, saved_ids, depth_value, images, save_image);
            auto end_time = std::chrono::steady_clock::now();
            auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            if (elapsed_time.count() >= 5000) {
                this->_logger.info("Uploading images via FTP...");
                image_publisher(images, saved_ids);
                saved_ids.clear();
                start_time = end_time; // Reset start time
                this->_logger.info("Upload complete. Continuing detection...");
                images.clear();
            }

        } else {
            this->_logger.error("Error during camera grab: %d", static_cast<int>(grab_result));
            break;
        }
    }
}

int main(int argc, char** argv) {
    Core::BaseArgumentParser arguments(argc, argv);
    
    // Change to match the style used in zed.cpp
    auto& group = arguments.add_mutually_exclusive_group();
    group.add_argument("--svo")
         .help("SVO2 Recording File Path");
    
    Core::ArgumentParser args(arguments);
    
    Core::Logger app_logger(Core::LogLevel::INFO, "app.log", "PeopleDetectorMain");

    try {
        app_logger.info("Initializing People Detector...");
        PeopleDetector detector(args);
        app_logger.info("People Detector initialized.");
        // Get the SVO path from the argument
        if (auto svo_path = args.present("--svo")) {
            app_logger.info("Running People Detector with SVO2 file: %s", svo_path->c_str());
            detector.run(*svo_path);
        } else {
            throw std::runtime_error("SVO2 file path is required");
        }
        
        return 0;
    } catch (const std::exception& e) {
        app_logger.error("Error: %s", e.what());
        return 1;
    }
}