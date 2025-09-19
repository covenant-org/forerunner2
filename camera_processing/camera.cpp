#include "argument_parser.hpp"
#include <sl/Camera.hpp>
#include "camera.hpp"
#include <opencv2/opencv.hpp>
#include "../people-detection/src/camera_tracking/camera_tracking.hpp"

PeopleDetector::PeopleDetector(const Core::ArgumentParser& parser)
    : Core::Vertex(parser) {
    std::cout << "PeopleDetector: Base class Core::Vertex initialized" << std::endl;
    try {
        std::cout << "PeopleDetector: Creating publisher instance..." << std::endl;
        this->_people_image_pub = this->create_publisher<ImageData>("people_image");
        std::cout << "PeopleDetector: Publisher created successfully." << std::endl;
        // ...existing code...
    } catch (const std::exception& e) {
        std::cerr << "PeopleDetector: Failed to create publisher: " << e.what() << std::endl;
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
                
                auto reader = ::capnp::Data::Reader(buf.data(), buf.size());
                msg.content.setData(reader);

                msg.publish();

        } catch (const std::exception& e) {
            std::cerr << e.what() << std::endl;
        }

    }
}



void PeopleDetector::run(std::string svo2_path) {
    this->_people_image_pub = this->create_publisher<ImageData>("people_image");

    std::unordered_set<int> saved_ids;
    std::cout << "SVO2 file path: " << svo2_path << std::endl;
    
    initZed(zed, svo2_path);
    ObjectDetectionParameters detection_parameters;
    ObjectDetectionRuntimeParameters detection_parameters_rt;  

    // run detection for every Camera grab
    initDetectionParameters(zed, detection_parameters);
    setObjectDetectionRuntimeParameters(detection_parameters_rt);

    Objects objects;
    std::cout << std::setprecision(3);
    float depth_value = 0.0f;
    std::list<cv::Mat> images;
    auto start_time = std::chrono::steady_clock::now();
    bool save_image = false;
    std::cout << "Starting main loop..." << std::endl;

    while (true) {
        auto grab_result = zed.grab();
        if (grab_result == ERROR_CODE::SUCCESS) {
            processDetections(zed, detection_parameters_rt, objects, saved_ids, depth_value, images, save_image);
            auto end_time = std::chrono::steady_clock::now();
            auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            if (elapsed_time.count() >= 5000) {
                std::cout << "Uploading images via FTP..." << std::endl;
                image_publisher(images, saved_ids);
                saved_ids.clear();
                start_time = end_time; // Reset start time
                std::cout << "Upload complete. Continuing detection..." << std::endl;
                images.clear();
            }

        } else {
            std::cerr << "Error during camera grab: " << grab_result << std::endl;
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
    
    try {
        std::cout << "Initializing People Detector..." << std::endl;
        PeopleDetector detector(args);
        std::cout << "People Detector initialized." << std::endl;
        // Get the SVO path from the argument
        if (auto svo_path = args.present("--svo")) {
            std::cout << "Running People Detector with SVO2 file: " << *svo_path << std::endl;
            detector.run(*svo_path);
        } else {
            throw std::runtime_error("SVO2 file path is required");
        }
        
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}