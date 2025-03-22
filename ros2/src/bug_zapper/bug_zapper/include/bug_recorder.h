#pragma once
#ifndef BUG_RECORDER_H
#define BUG_RECORDER_H

#include <iostream>
#include <memory>
#include <mutex>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <bug_zapper_msgs/msg/bug_detection.hpp>

namespace patterns
{

class BugRecorder
{
public:
    // Get the singleton instance (initialization requires a postAddress)
    static std::shared_ptr<BugRecorder> GetInstance(const std::string &postAddress = "");
    std::string createJson(uint64_t timestamp, uint64_t frameNumber, const std::vector<cv::Vec3f> &circles);
    bool Post(const std::string &jsonData);

    ~BugRecorder()
    {
        std::cout << "BugRecorder Destructor\n";
    }

private:
    explicit BugRecorder(std::string postAddress) : m_postAddress(std::move(postAddress))
    {
        std::cout << "BugRecorder Constructor: " << m_postAddress << '\n';
    }

    // Deleted copy constructor and assignment operator
    BugRecorder(const BugRecorder &) = delete;
    BugRecorder &operator=(const BugRecorder &) = delete;

    std::string m_postAddress;
    static std::shared_ptr<BugRecorder> instance;
    static std::once_flag initFlag;
};

} // namespace patterns
#endif