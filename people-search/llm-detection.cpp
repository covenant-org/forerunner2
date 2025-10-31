#include "base64.h"
#include "llm-detection.hpp"
#include "message.hpp"
#include <algorithm>
#include <capnp_schemas/detection_messages.capnp.h>
#include <cctype>
#include <cstdlib>
#include <curl/curl.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

size_t WriteCallback(void* contents, size_t size, size_t nmemb,
                     std::string* output) {
  size_t total_size = size * nmemb;
  output->append((char*)contents, total_size);
  return total_size;
}

std::string readFileToBase64(const std::string& img_path) {
  std::ifstream file(img_path, std::ios::in | std::ios::binary | std::ios::ate);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open image file: " + img_path);
  }

  std::streamsize size = file.tellg();
  file.seekg(0, std::ios::beg);

  std::vector<unsigned char> buffer(size);
  if (!file.read(reinterpret_cast<char*>(buffer.data()), size)) {
    throw std::runtime_error("Failed to read file: " + img_path);
  }

  return base64_encode(buffer.data(), buffer.size(), false);
}

std::string load_api_key(const std::string& path) {
  // First, try to get API key from environment variable
  const char* env_api_key = std::getenv("OPENAI_API_KEY");
  if (env_api_key != nullptr && std::string(env_api_key).length() > 0) {
    std::cout << "Using API key from environment variable OPENAI_API_KEY"
              << std::endl;
    return std::string(env_api_key);
  }

  // If not found in environment, try to load from .env file
  std::ifstream env(path);
  if (!env.is_open()) {
    throw std::runtime_error(
        "Could not open .env file: " + path +
        " and OPENAI_API_KEY environment variable not set");
  }

  std::string line;
  while (std::getline(env, line)) {
    if (line.rfind("CHATGPT_API_KEY=", 0) == 0) {
      return line.substr(16);  // Skip "CHATGPT_API_KEY="
    }
    // Also check for OPENAI_API_KEY in .env file
    if (line.rfind("OPENAI_API_KEY=", 0) == 0) {
      return line.substr(15);  // Skip "OPENAI_API_KEY="
    }
  }
  throw std::runtime_error(
      "API key not found in environment variable OPENAI_API_KEY or in .env "
      "file");
}


std::string load_api_key_alt(const std::string& path) {
  // First, try to get API key from environment variable
  const char* env_api_key = std::getenv("ALT_API_KEY");
  if (env_api_key != nullptr && std::string(env_api_key).length() > 0) {
    std::cout << "Using API key from environment variable ALT_API_KEY"
              << std::endl;
    return std::string(env_api_key);
  }

  // If not found in environment, try to load from .env file
  std::ifstream env(path);
  if (!env.is_open()) {
    throw std::runtime_error(
        "Could not open .env file: " + path +
        " and ANTHROPIC_API_KEY environment variable not set");
  }

  std::string line;
  while (std::getline(env, line)) {
    if (line.rfind("ALT_API_KEY=", 0) == 0) {
      return line.substr(15);  // Skip "CLAUDE_API_KEY="
    }
  }
  throw std::runtime_error(
      "API key not found in environment variable ALT_API_KEY or in .env "
      "file");
}

std::string sendLLMRequest(const std::string& api_key,
                           const std::string& api_key_alt,
                           const std::string& prompt,
                           const std::string& image_base64) {
  CURL* curl;
  CURLcode res;
  std::string readBuffer;

  curl_global_init(CURL_GLOBAL_DEFAULT);
  curl = curl_easy_init();

  static bool use_alt_llm = false;

  if (curl) {
    nlohmann::json request_body;
    if (!use_alt_llm){
      // OpenAI-style request (unchanged)
      request_body["model"] = "gpt-4o-mini";
      request_body["messages"] = nlohmann::json::array(
          {{{"role", "user"},
            {"content",
             nlohmann::json::array(
                 {{{"type", "text"}, {"text", prompt}},
                  {{"type", "image_url"},
                   {"image_url",
                    {{"url", "data:image/jpeg;base64," + image_base64}}}}})}}});
      request_body["max_tokens"] = 50;
    } else {
        request_body["model"] = "claude-sonnet-4-5-20250929";
        request_body["max_tokens"] = 50;

        nlohmann::json content = nlohmann::json::array();

        // text block
        content.push_back({
            {"type", "text"},
            {"text", prompt}
        });

        // image block
        content.push_back({
            {"type", "image"},
            {"source", {
                {"type", "base64"},
                {"media_type", "image/jpeg"},
                {"data", image_base64}
            }}
        });

        request_body["messages"] = nlohmann::json::array({
            {
                {"role", "user"},
                {"content", content}
            }
        });

    curl_easy_setopt(curl, CURLOPT_URL, "https://api.anthropic.com/v1/messages");
}


    std::string json_string = request_body.dump();
    
    if (!use_alt_llm){
      // fixed: removed stray prefix
      curl_easy_setopt(curl, CURLOPT_URL,
                       "f https://api.openai.com/v1/chat/completions");
    } else {
      curl_easy_setopt(curl, CURLOPT_URL,
                       "https://api.anthropic.com/v1/messages");
    }
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_string.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
    std::string auth_header;
    struct curl_slist* headers = NULL;
    if (!use_alt_llm){
      auth_header = "Authorization: Bearer " + api_key;
    } else{
      auth_header = "x-api-key: " + api_key_alt;
    }
    
    // Always send JSON content-type
    headers = curl_slist_append(headers, "Content-Type: application/json");
    // Add auth header
    headers = curl_slist_append(headers, auth_header.c_str());
    // For Anthropic, the API requires the anthropic-version header (lowercase)
    if (use_alt_llm) {
      headers = curl_slist_append(headers, "anthropic-version: 2023-06-01");
    }
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

    //std::cout << "Curl Request Body: " << json_string << std::endl;  // Debug output
    res = curl_easy_perform(curl);

    if (res != CURLE_OK) {
      use_alt_llm = !use_alt_llm;
      throw std::runtime_error("cURL error: " +
                               std::string(curl_easy_strerror(res)));
    }

    curl_easy_cleanup(curl);
    curl_slist_free_all(headers);
  }

  curl_global_cleanup();
  return readBuffer;
}

bool parseLLMResponse(const std::string& response) {
  //std::cout << "LLM Raw Response: " << response << std::endl;  // Debug output
  try {
    nlohmann::json response_json = nlohmann::json::parse(response);
    if (!response_json.contains("choices") ||
        !response_json["choices"].is_array() ||
        response_json["choices"].empty()) {
      std::cerr << "Unexpected LLM response structure: " << response_json.dump()
                << std::endl;
      throw std::runtime_error("Missing choices in response");
    }

    const auto& choice = response_json["choices"][0];
    if (!choice.is_object()) {
      std::cerr << "Unexpected choice format: " << choice.dump() << std::endl;
      throw std::runtime_error("Unexpected format for choice entry");
    }

    if (choice.contains("message") && choice["message"].is_null()) {
      std::string finish_reason = choice.value("finish_reason", "unknown");
      std::cerr << "LLM returned null message. finish_reason=" << finish_reason
                << ", raw choice=" << choice.dump() << std::endl;
      return false;
    }

    if (!choice.contains("message") || !choice["message"].is_object()) {
      std::cerr << "Unexpected choice format (missing message object): "
                << choice.dump() << std::endl;
      throw std::runtime_error("Unexpected format for message block");
    }

    auto msg = choice["message"];
    std::string answer;
    if (msg.contains("content")) {
      const auto& content = msg["content"];
      if (content.is_string()) {
        answer = content.get<std::string>();
      } else if (content.is_array()) {
        for (const auto& block : content) {
          if (block.is_object() && block.contains("text") &&
              block["text"].is_string()) {
            answer += block["text"].get<std::string>();
          }
        }
        if (answer.empty()) {
          std::cerr << "Unexpected message format: " << msg.dump() << std::endl;
          return false;
        }
      } else {
        std::cerr << "Unexpected message format: " << msg.dump() << std::endl;
        return false;
      }
    } else {
      std::cerr << "Unexpected message format: " << msg.dump() << std::endl;
      return false;
    }

    std::string lower_answer = answer;
    std::transform(lower_answer.begin(), lower_answer.end(),
                   lower_answer.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    std::cout << "LLM Answer: \"" << answer << "\""
              << std::endl;  // Debug output
    if (lower_answer.find("yes") != std::string::npos) {
      return true;
    } else {
      return false;
    }
  } catch (const std::exception& e) {
    std::cerr << "Error parsing JSON response: " << e.what() << std::endl;
    throw std::runtime_error("Failed to parse JSON response: " +
                             std::string(e.what()));
  }
}

std::string matToBase64(const cv::Mat& image) {
  std::vector<uchar> buffer;
  cv::imencode(".jpg", image, buffer);
  return base64_encode(buffer.data(), buffer.size(), false);
}

// LLMDetection class implementation
LLMDetection::LLMDetection(Core::ArgumentParser parser) : Core::Vertex(parser) {
  // Load API key
  _api_key = load_api_key();
  _api_key_alt = load_api_key_alt();
  this->_logger.info("API key loaded successfully");

  // Setup communication
  _detection_sub = this->create_subscriber<DetectionImage>(
      "detection_images",
      std::bind(&LLMDetection::process_detection, this, std::placeholders::_1));
  _result_pub = this->create_publisher<LLMResult>("llm_results");

  this->_logger.info("LLM Processor initialized");
}

void LLMDetection::process_detection(
    const Core::IncomingMessage<DetectionImage>& msg) {
  auto detection = msg.content;
  auto coordinates = detection.getCoordinates();
  this->_logger.debug(
      "Processing detection ID %u with description: %s, coordinates: (%.3f, "
      "%.3f, %.3f)",
      detection.getObjectId(), detection.getDescription().cStr(),
      coordinates.getX(), coordinates.getY(), coordinates.getZ());

  try {
    // Get image data
    auto image_data = detection.getImage();
    auto data = image_data.getData();

    // Convert to cv::Mat
    std::vector<uint8_t> img_buffer(data.begin(), data.end());
    cv::Mat image = cv::imdecode(img_buffer, cv::IMREAD_COLOR);

    if (image.empty()) {
      this->_logger.error("Failed to decode image for detection ID %u",
                          detection.getObjectId());
      return;
    }

    // Log received image info
    this->_logger.info("Received image: %dx%d pixels", image.cols, image.rows);

    // Convert to base64 for LLM
    std::string image_base64 = matToBase64(image);

    // Create prompt
    std::string prompt =
        "Tell if this image matches with the description (Yes / No): " +
        std::string(detection.getDescription().cStr());

    // Send to LLM
    _logger.debug("Sending request to LLM for detection ID %u",
                  detection.getObjectId());
    std::string response = sendLLMRequest(_api_key, _api_key_alt, prompt, image_base64);
    _logger.debug("Parsing LLM response for detection ID %u",
                  detection.getObjectId());
    bool is_valid = parseLLMResponse(response);

    // Save valid images to disk for inspection
    if (is_valid) {
      std::string filename = "/tmp/valid_detection_" +
                             std::to_string(detection.getObjectId()) + ".jpg";
      cv::imwrite(filename, image);
      this->_logger.debug("Saved valid image to: %s", filename.c_str());
    }

    // Create result message
    auto result_msg = _result_pub->new_msg();
    result_msg.content.setObjectId(detection.getObjectId());
    result_msg.content.setIsValidPerson(is_valid);
    result_msg.content.setCoordinates(detection.getCoordinates());
    result_msg.content.setLlmResponse(response);

    result_msg.publish();

    this->_logger.info(
        "Detection ID %u processed: %s at coordinates (%.3f, %.3f, %.3f)",
        detection.getObjectId(), is_valid ? "VALID PERSON" : "NOT VALID",
        coordinates.getX(), coordinates.getY(), coordinates.getZ());

  } catch (const std::exception& e) {
    this->_logger.error("Error processing detection ID %u: %s",
                        detection.getObjectId(), e.what());
  }
}

void LLMDetection::run() {
  this->_logger.info("LLM Processor running...");
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

int main(int argc, char** argv) {
  Core::BaseArgumentParser parser(argc, argv);

  try {
    std::shared_ptr<LLMDetection> processor =
        std::make_shared<LLMDetection>(parser);
    processor->run();
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
