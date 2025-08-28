#include "camera.hpp"
#include <fstream> 
#include <opencv2/imgproc.hpp>
#include <signal.h>
#include <thread>
#include <chrono>

std::vector<std::string> load_class_list()
{
    std::vector<std::string> class_list;
    // change this txt file  to your txt file that contains labels 
    std::ifstream ifs("model/coco-classes.txt");
    std::string line;
    while (getline(ifs, line))
    {
        class_list.push_back(line);
    }
    return class_list;
}

void load_net(cv::dnn::Net &net)
{   
    // change this path to your model path 
    auto result = cv::dnn::readNet("model/yolov11n.onnx");

    std::cout << "Running on CPU/n";
    result.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    result.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
 
    net = result;
}

const std::vector<cv::Scalar> colors = {cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 0)};

const float INPUT_WIDTH = 640.0;
const float INPUT_HEIGHT = 640.0;
const float SCORE_THRESHOLD = 0.5;
const float NMS_THRESHOLD = 0.5;
const float CONFIDENCE_THRESHOLD = 0.5;

struct Detection
{
    int class_id;
    float confidence;
    cv::Rect box;
};

cv::Mat format_yolov11(const cv::Mat &source) {
    int col = source.cols;
    int row = source.rows;
    int _max = MAX(col, row);
    cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);
    source.copyTo(result(cv::Rect(0, 0, col, row)));
    return result;
}

void detect(cv::Mat &image, cv::dnn::Net &net, std::vector<Detection> &output, const std::vector<std::string> &className) {
    cv::Mat blob;

    // Format the input image to fit the model input requirements
    auto input_image = format_yolov11(image);
    
    // Convert the image into a blob and set it as input to the network
    cv::dnn::blobFromImage(input_image, blob, 1./255., cv::Size(INPUT_WIDTH, INPUT_HEIGHT), cv::Scalar(), true, false);
    net.setInput(blob);
    std::vector<cv::Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());

    // Scaling factors to map the bounding boxes back to original image size
    float x_factor = input_image.cols / INPUT_WIDTH;
    float y_factor = input_image.rows / INPUT_HEIGHT;
    
    float *data = (float *)outputs[0].data;

    const int dimensions = 85;
    const int rows = 25200;
    
    std::vector<int> class_ids; // Stores class IDs of detections
    std::vector<float> confidences; // Stores confidence scores of detections
    std::vector<cv::Rect> boxes;   // Stores bounding boxes

   // Loop through all the rows to process predictions
    for (int i = 0; i < rows; ++i) {

        // Get the confidence of the current detection
        float confidence = data[4];

        // Process only detections with confidence above the threshold
        if (confidence >= CONFIDENCE_THRESHOLD) {
            
            // Get class scores and find the class with the highest score
            float * classes_scores = data + 5;
            cv::Mat scores(1, className.size(), CV_32FC1, classes_scores);
            cv::Point class_id;
            double max_class_score;
            minMaxLoc(scores, 0, &max_class_score, 0, &class_id);

            // If the class score is above the threshold, store the detection
            if (max_class_score > SCORE_THRESHOLD) {

                confidences.push_back(confidence);
                class_ids.push_back(class_id.x);

                // Calculate the bounding box coordinates
                float x = data[0];
                float y = data[1];
                float w = data[2];
                float h = data[3];
                int left = int((x - 0.5 * w) * x_factor);
                int top = int((y - 0.5 * h) * y_factor);
                int width = int(w * x_factor);
                int height = int(h * y_factor);
                boxes.push_back(cv::Rect(left, top, width, height));
            }
        }

        data += 85;
    }

    // Apply Non-Maximum Suppression
    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, nms_result);

    // Draw the NMS filtered boxes and push results to output
    for (int i = 0; i < nms_result.size(); i++) {
        int idx = nms_result[i];

        // Only push the filtered detections
        Detection result;
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];
        result.box = boxes[idx];
        output.push_back(result);

        // Draw the final NMS bounding box and label
        cv::rectangle(image, boxes[idx], cv::Scalar(0, 255, 0), 3);
        std::string label = className[class_ids[idx]];
        cv::putText(image, label, cv::Point(boxes[idx].x, boxes[idx].y - 5), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 255, 255), 2);
    }
}


static std::atomic<bool> g_running{true};

static void sigint_handler(int) { g_running = false; }

CameraViewer::CameraViewer() = default;
CameraViewer::~CameraViewer() { stop(); }

void CameraViewer::onImage(const gz::msgs::Image &msg,
                           const gz::transport::MessageInfo &) {
  const int w = msg.width();
  const int h = msg.height();
  const int step = msg.step();
  const std::string &data = msg.data();

  if (data.empty() || w == 0 || h == 0) return;

  // Common cases: 3 channels (RGB8) or 1 channel (GRAY8)
  cv::Mat frame;
  if (step == w * 3) {
    // Interpret as RGB8
    frame = cv::Mat(h, w, CV_8UC3, const_cast<char *>(data.data()), step).clone();
    cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);  // Convert to BGR for OpenCV
  } else if (step == w) {
    // Grayscale
    frame = cv::Mat(h, w, CV_8UC1, const_cast<char *>(data.data()), step).clone();
  } else if (step == w * 4) {
    // Possibly RGBA
    frame = cv::Mat(h, w, CV_8UC4, const_cast<char *>(data.data()), step).clone();
    cv::cvtColor(frame, frame, cv::COLOR_RGBA2BGR);
  } else {
    // Fallback: try to create a single-channel image
    frame = cv::Mat(h, w, CV_8UC1, const_cast<char *>(data.data()), step).clone();
  }

  {
    std::lock_guard<std::mutex> lock(_frame_mtx);
    _latest_frame = frame;
  }
}

bool CameraViewer::start(cv::dnn::Net &net, const std::vector<std::string> &class_list) {
  if (_running) return true;
  _running = true;

  // Subscribe to the /camera topic (matches your request)
  if (!_node.Subscribe("/camera", &CameraViewer::onImage, this)) {
    return false;
  }

  // Main display loop on the main thread
  std::thread([this, &net, &class_list]() {
    cv::namedWindow("camera", cv::WINDOW_AUTOSIZE);
    while (_running && g_running) {
      std::vector<Detection> output;
      cv::Mat frame;
      {
        std::lock_guard<std::mutex> lock(_frame_mtx);
        if (!_latest_frame.empty()) frame = _latest_frame.clone();
      }
      if (!frame.empty()) {
        detect(frame, net, output, class_list);
        cv::imshow("camera", frame);
        // waitKey with small delay to keep UI responsive
        if (cv::waitKey(1) == 27) {  // ESC to quit
          g_running = false;
          break;
        }
      } else {
        // no frame yet
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
    cv::destroyWindow("camera");
    _running = false;
  }).detach();

  return true;
}

void CameraViewer::stop() { _running = false; }
 
int main(int argc, char **argv) {
  signal(SIGINT, sigint_handler);
  std::vector<std::string> class_list = load_class_list();
  cv::dnn::Net net;
  load_net(net);
  CameraViewer viewer;
  if (!viewer.start(net, class_list)) {
    fprintf(stderr, "Failed to subscribe to /camera\n");
    return 1;
  }

  // Keep process alive until SIGINT or window closed
  while (g_running) std::this_thread::sleep_for(std::chrono::milliseconds(50));
  viewer.stop();
  return 0;
}

