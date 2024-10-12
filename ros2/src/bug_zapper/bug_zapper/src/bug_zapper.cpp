#include <bug_zapper.h>

using namespace std::chrono_literals;
namespace patterns
{

BugZapper::BugZapper(uint8_t id) : rclcpp_lifecycle::LifecycleNode("bug_zap_lifecycle_node"), m_id(id)
{
    m_startTime = this->get_clock()->now();
    m_cameraFrameSub = this->create_subscription<sensor_msgs::msg::Image>(
        "cameraFrame", 10, std::bind(&BugZapper::cameraFrameSubCb, this, std::placeholders::_1));
    // Create a 10Hz timer to call the Tick function
    auto timerInterval = std::chrono::milliseconds(100);  // 100ms = 10Hz
    m_tickTimer = this->create_wall_timer(
        timerInterval, std::bind(&BugZapper::Tick, this));
}

void BugZapper::cameraFrameSubCb(const sensor_msgs::msg::Image::SharedPtr imgMsg)
{
    // Convert the ROS image message to OpenCV format
    std::cout << "Received!" << std::endl;
    cv::Mat cameraFrame = cv_bridge::toCvShare(imgMsg, "bgr8")->image;
    m_detector->AddImage(cameraFrame);

}

void BugZapper::Tick()
{
    m_detector->Tick();
    std::cout << "\nTicked the detector";

    // for (std::shared_ptr<Bug> bug : m_bugs)
}

void BugZapper::SetDetector(std::shared_ptr<BugDetector> det)
{
    m_detector = det;
}

} // namespace patterns
