#include <bug_zapper.h>

using namespace std::chrono_literals;
namespace patterns
{

BugZapper::BugZapper(uint8_t id)
    : rclcpp_lifecycle::LifecycleNode("bug_zap_lifecycle_node"), m_id(id),
      m_tfBuffer(std::make_shared<tf2_ros::Buffer>(this->get_clock())), m_tfListener(*m_tfBuffer)
{
    m_startTimeMs = RCL_NS_TO_MS(this->get_clock()->now().nanoseconds());
    m_cameraFrameSub = this->create_subscription<sensor_msgs::msg::Image>(
        "cameraFrame", 10, std::bind(&BugZapper::cameraFrameSubCb, this, std::placeholders::_1));
    // Create a 10Hz timer to call the Tick function
    auto timerInterval = std::chrono::milliseconds(100); // 100ms = 10Hz
    m_tickTimer = this->create_wall_timer(timerInterval, std::bind(&BugZapper::Tick, this));

    m_cmdVelSub = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&BugZapper::cmdVelSubCallback, this, std::placeholders::_1));

    m_odomSub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "odom", 10, std::bind(&BugZapper::odomSubCallback, this, std::placeholders::_1));
}

void BugZapper::cameraFrameSubCb(const sensor_msgs::msg::Image::SharedPtr imgMsg)
{
    // Convert the ROS image message to OpenCV format
    cv::Mat cameraFrame = cv_bridge::toCvShare(imgMsg, "bgr8")->image;
    // cv::Mat cameraFrameCopy = cameraFrame.clone();
    uint64_t timestampMillis =
        static_cast<uint64_t>(imgMsg->header.stamp.sec) * 1000 + RCL_NS_TO_MS(imgMsg->header.stamp.nanosec) - m_startTimeMs;
    std::cout << "Received with timestamp " << timestampMillis << "ms." << std::endl;

    patterns::ImageInfo imageinfo;
    imageinfo.frame = cameraFrame.clone();
    imageinfo.timestampMillisecs = timestampMillis;
    imageinfo.frameNumber = m_frameNumber++;
    m_detector->AddImage(std::move(imageinfo));
}

void BugZapper::cmdVelSubCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // Handle the received velocity message
    double velocityX = msg->linear.x;
    double velocityY = msg->linear.y;

    // RCLCPP_INFO(this->get_logger(), "Received Velocity - X: %.2f, Y: %.2f", velocityX, velocityY);
}

void BugZapper::odomSubCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // Handle the received pose message
    double posX = msg->pose.position.x;
    double posY = msg->pose.position.y;

    // RCLCPP_INFO(this->get_logger(), "Received Position - X: %.2f, Y: %.2f", posX, posY);
}

void BugZapper::updateTransform()
{
    // Get the transform for the odometry frame
    try {
        m_transform = m_tfBuffer->lookupTransform("odom", "base_link", tf2::TimePointZero);
        RCLCPP_INFO(this->get_logger(), "Latest Transform from odom to base_link: [X: %.2f, Y: %.2f, Z: %.2f]",
                    m_transform.transform.translation.x, m_transform.transform.translation.y,
                    m_transform.transform.translation.z);
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform odom to base_link: %s", ex.what());
    }
}

void BugZapper::Tick()
{   
    m_timeNowMs = RCL_NS_TO_MS(rclcpp::Clock().now().nanoseconds()) - m_startTimeMs;
    updateTransform();
    m_detector->Tick(m_timeNowMs);
}

void BugZapper::SetDetector(std::shared_ptr<BugDetector> det)
{
    m_detector = det;
}

} // namespace patterns
