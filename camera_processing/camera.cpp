#include <sl/Camera.hpp>
#include "camera.hpp"
#include <opencv2/opencv.hpp>
#include "../people-detection/src/camera_tracking/camera_tracking.hpp"

PeopleDetector::PeopleDetector(const Core::ArgumentParser& parser) : Core::Vertex(parser) {
    this->_people_image_pub = this->create_publisher<ImageData>("people_image");

}

void PeopleDetector::ftp_upload(std::list<cv::Mat>& images, std::unordered_set<int>& ids) {
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

    std::unordered_set<int> saved_ids;
    
    initZed(zed, svo2_path);
    ObjectDetectionParameters detection_parameters;
    ObjectDetectionRuntimeParameters detection_parameters_rt;  

    // run detection for every Camera grab
    // track detects object accross time and space
    initDetectionParameters(zed, detection_parameters);
    // detection runtime parameters
    setObjectDetectionRuntimeParameters(zed, detection_parameters_rt);

    // detection output
    Objects objects;
    cout << setprecision(3);
    float depth_value = 0.0f;
    list<cv::Mat> images;
    auto start_time = std::chrono::steady_clock::now();
    bool save_image = false;
    while (true) {
      if(zed.grab() == ERROR_CODE::SUCCESS){

        processDetections(zed, detection_parameters_rt, objects, saved_ids, depth_value, images, save_image);


        auto end_time = std::chrono::steady_clock::now(); // Capture end time
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        if (elapsed_time.count() >= 5000) {
            saved_ids.clear();
            start_time = end_time; // Reset start time
            }
        }else {
            std::cerr << "Error during camera grab." << std::endl;
            break;
        }
        auto end_time = std::chrono::steady_clock::now(); // Capture end time
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        if (elapsed_time.count() >= 5000) {
            ftp_upload(images, saved_ids);
            saved_ids.clear();
            start_time = end_time; // Reset start time
        }
    }
}

int main(int argc, char** argv) {

    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << "<svo2_path>" << std::endl;
        return 1;
    }
    std::string svo2_path = argv[1];

    PeopleDetector detector(Core::ArgumentParser(argc, argv));
    detector.run(svo2_path);
  
  }