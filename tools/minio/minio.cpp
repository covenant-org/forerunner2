#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <miniocpp/client.h>
#include <miniocpp/providers.h>

namespace {
constexpr const char* kDefaultEndpoint = "minio.covenant.space";  // Update with your MinIO host.

bool ParseBoolEnv(const char* value, bool fallback) {
    if (!value) {
        return fallback;
    }

    std::string normalized(value);
    std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });

    if (normalized == "1" || normalized == "true" || normalized == "yes" || normalized == "on") {
        return true;
    }
    if (normalized == "0" || normalized == "false" || normalized == "no" || normalized == "off") {
        return false;
    }
    return fallback;
}

}

int main(int argc, char* argv[]) {
    static_cast<void>(argc);
    static_cast<void>(argv);

    const char* access_key = std::getenv("MINIO_ACCESS_KEY");
    const char* secret_key = std::getenv("MINIO_SECRET_KEY");
    if (!access_key || !secret_key) {
        std::cerr << "Environment variables MINIO_ACCESS_KEY and MINIO_SECRET_KEY must be set." << std::endl;
        return EXIT_FAILURE;
    }

    const char* endpoint_env = std::getenv("MINIO_ENDPOINT");
    std::string host = endpoint_env && *endpoint_env ? endpoint_env : kDefaultEndpoint;
    bool use_https = ParseBoolEnv(std::getenv("MINIO_USE_HTTPS"), true);
    unsigned port = 0;

    if (const char* port_env = std::getenv("MINIO_PORT")) {
        char* end_ptr = nullptr;
        const unsigned long parsed = std::strtoul(port_env, &end_ptr, 10);
        if (end_ptr != port_env && *end_ptr == '\0' && parsed <= std::numeric_limits<unsigned>::max()) {
            port = static_cast<unsigned>(parsed);
        }
    }

    if (host.empty()) {
        std::cerr << "No valid host found in MINIO_ENDPOINT." << std::endl;
        return EXIT_FAILURE;
    }

    minio::s3::BaseUrl base_url(host.c_str(), use_https);
    base_url.port = port;

    const char* bucket_env = std::getenv("MINIO_BUCKET");
    if (!bucket_env || !*bucket_env) {
        std::cerr << "MINIO_BUCKET must be set to list objects." << std::endl;
        return EXIT_FAILURE;
    }
    const std::string bucket = bucket_env;

    std::cout << "Listing objects from bucket '" << bucket << "' using endpoint: "
              << (use_https ? "https" : "http") << "://" << host;
    if (port != 0) {
        std::cout << ':' << port;
    }
    std::cout << '\n';

    minio::creds::StaticProvider provider(access_key, secret_key);
    minio::s3::Client client(base_url, &provider);

    minio::s3::BucketExistsArgs exists_args;
    exists_args.bucket = bucket;
    const minio::s3::BucketExistsResponse exists_resp = client.BucketExists(exists_args);
    if (!exists_resp) {
        std::cout << "unable to check bucket '" << bucket << "'; " << exists_resp.Error().String() << std::endl;
        return EXIT_FAILURE;
    }
    if (!exists_resp.exist) {
        std::cout << "Bucket '" << bucket << "' does not exist or is not accessible." << std::endl;
        return EXIT_FAILURE;
    }

    minio::s3::ListObjectsArgs list_args;
    list_args.bucket = bucket;
    list_args.recursive = true;

    bool found_any = false;
    for (auto it = client.ListObjects(list_args); it; ++it) {
        auto& item = *it;
        if (!item) {
            std::cout << "unable to list objects; " << item.Error().String() << std::endl;
            return EXIT_FAILURE;
        }
        found_any = true;
        if (item.is_prefix) {
            std::cout << "[DIR]  " << item.name << std::endl;
        } else {
            std::cout << "[FILE] " << item.name << " (" << item.size << " bytes)" << std::endl;
        }
    }

    if (!found_any) {
        std::cout << "Bucket '" << bucket << "' is empty." << std::endl;
    }

    return EXIT_SUCCESS;
}
