#include <bug_recorder.h>
#include <curl/curl.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

namespace patterns
{
// Static members initialization
std::shared_ptr<BugRecorder> BugRecorder::instance = nullptr;
std::once_flag BugRecorder::initFlag;

bool Post(bug_zapper_msgs::msg::BugDetection)
{
    return true;
}

std::shared_ptr<BugRecorder> BugRecorder::GetInstance(const std::string &postAddress)
{
    std::call_once(initFlag, [&]() {
        instance = std::shared_ptr<BugRecorder>(new BugRecorder(postAddress));
    });
    return instance;
}

// JSON für die Anfrage erstellen
std::string BugRecorder::createJson(uint64_t timestamp, uint64_t frameNumber, const std::vector<cv::Vec3f> &circles)
{
    std::string json = "[";

    for (size_t i = 0; i < circles.size(); i++) {
        json += "{";
        json += "\"id\": " + std::to_string(frameNumber + i) + ", ";
        json += "\"timestamp\": " + std::to_string(timestamp) + ", ";
        json += "\"radius\": " + std::to_string(circles[i][2]) + ", ";
        // json += "\"speed\": " + std::to_string(speed) + ", ";
        json += "\"position\": {\"x\": " + std::to_string(circles[i][0]) + ", \"y\": " + std::to_string(circles[i][1]) +
                "}";
        json += "}";

        if (i < circles.size() - 1) {
            json += ", ";
        }
    }

    json += "]";
    return json;
}

// Funktion für die POST-Anfrage
bool BugRecorder::Post(const std::string &jsonData)
{
    CURL *curl;
    CURLcode res;

    curl_global_init(CURL_GLOBAL_ALL);
    curl = curl_easy_init();

    if (curl) {
        struct curl_slist *headers = NULL;
        headers = curl_slist_append(headers, "Content-Type: application/json");

        curl_easy_setopt(curl, CURLOPT_URL, m_postAddress.c_str());
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_POST, 1L);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, jsonData.c_str());

        res = curl_easy_perform(curl);
        if (res != CURLE_OK) {
            std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
        }
        else {
            std::cout << "POST request erfolgreich gesendet!" << std::endl;
        }

        curl_easy_cleanup(curl);
        curl_slist_free_all(headers);
        curl_global_cleanup();

        return res == CURLE_OK;
    }

    return false;
}
} // namespace patterns