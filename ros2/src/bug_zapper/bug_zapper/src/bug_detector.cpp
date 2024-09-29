#include <bug_detector.h>

using namespace std::chrono_literals;
namespace patterns
{

BugDetector::BugDetector(uint8_t id) : rclcpp_lifecycle::LifecycleNode("bug_zap_lifecycle_node"), m_id(id)
{
    m_startTime = this->get_clock()->now();
    m_cameraFrameSub = this->create_subscription<sensor_msgs::msg::Image>(
        "cameraFrame", 10, std::bind(&BugDetector::cameraFrameSubCb, this, std::placeholders::_1));
}

void BugDetector::cameraFrameSubCb(const sensor_msgs::msg::Image::SharedPtr imgMsg)
{
    // Convert the ROS image message to OpenCV format
    cv::Mat cameraFrame = cv_bridge::toCvShare(imgMsg, "bgr8")->image;

    // Process the camera frame here
    // For example, display it
    cv::imshow("Camera Frame", cameraFrame);
    cv::waitKey(100); // Wait for a short time to allow OpenCV to process the display
}
} // namespace patterns
