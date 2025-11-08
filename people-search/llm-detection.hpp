#ifndef LLM_DETECTION
#define LLM_DETECTION

#include <string>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include "vertex.hpp"
#include "publisher.hpp"
#include "subscriber.hpp"
#include <capnp_schemas/detection_messages.capnp.h>

// Reads an image file and encodes it to Base64
std::string readFileToBase64(const std::string& img_path);

// Converts cv::Mat directly to base64 (for pub/sub)
std::string matToBase64(const cv::Mat& image);

// Loads the API key from a `.env` file
std::string load_api_key(const std::string& path = "../.env");

std::string load_api_key_alt(const std::string& path = "../.env");

// Sends an HTTP request to the LLM API and returns the response as a string
std::string sendLLMRequest(const std::string& api_key, const std::string& api_key_alt, const std::string& prompt, const std::string& image_base64);

// Parses the LLM response to determine if the image matches the description
bool parseLLMResponse(const std::string& response);

// LLM Processor class for pub/sub functionality
class LLMDetection : public Core::Vertex {
private:
    std::shared_ptr<Core::Subscriber<DetectionImage>> _detection_sub;
    std::shared_ptr<Core::Publisher<LLMResult>> _result_pub;
    std::string _api_key;
    std::string _api_key_alt;
    
public:
    LLMDetection(Core::ArgumentParser parser);
    void process_detection(const Core::IncomingMessage<DetectionImage>& msg);
    void run();
};

#endif // LLM_DETECTION_UTILS_HPP

