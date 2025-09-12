#include "argument_parser.hpp"
#include <opencv2/opencv.hpp>
#include "publisher.hpp"
#include "vertex.hpp"
#include <sl/Camera.hpp>


class PeopleDetector : Core::Vertex{
  private:
    sl::Camera zed;
    std::shared_ptr<Core::Publisher<cv::Mat>> _people_image_pub;
    ObjectDetectionParameters detection_parameters;
    ObjectDetectionRuntimeParameters detection_parameters_rt;  
  public:
    PeopleDetector(const Core::ArgumentParser&);
    void run();
    void ftp_upload();
}