#include "camera.hpp"
#include <opencv2/imgproc.hpp>
#include <signal.h>
#include <thread>
#include <chrono>

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

bool CameraViewer::start() {
  if (_running) return true;
  _running = true;

  // Subscribe to the /camera topic (matches your request)
  if (!_node.Subscribe("/camera", &CameraViewer::onImage, this)) {
    return false;
  }

  // Main display loop on the main thread
  std::thread([this]() {
    cv::namedWindow("camera", cv::WINDOW_AUTOSIZE);
    while (_running && g_running) {
      cv::Mat frame;
      {
        std::lock_guard<std::mutex> lock(_frame_mtx);
        if (!_latest_frame.empty()) frame = _latest_frame.clone();
      }
      if (!frame.empty()) {
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

  CameraViewer viewer;
  if (!viewer.start()) {
    fprintf(stderr, "Failed to subscribe to /camera\n");
    return 1;
  }

  // Keep process alive until SIGINT or window closed
  while (g_running) std::this_thread::sleep_for(std::chrono::milliseconds(50));
  viewer.stop();
  return 0;
}