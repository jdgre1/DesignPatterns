#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <bug_detector.h>

namespace patterns
{

BugDetector::BugDetector(uint8_t id, std::shared_ptr<tf2_ros::Buffer> tfBuffer)
    : m_id(id), m_tfBuffer(tfBuffer), m_logger(rclcpp::get_logger("Detector_" + std::to_string(id))),
      m_startTimeMs(RCL_NS_TO_MS(rclcpp::Clock().now().nanoseconds()))
{
    setupCameraCalibrationConfig();
}

void BugDetector::setupCameraCalibrationConfig()
{
    std::string config_path =
        ament_index_cpp::get_package_share_directory("bug_zapper") + "/config/camera_calibration.xml";

    cv::FileStorage fs(config_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        RCLCPP_FATAL(m_logger, "Failed to open camera calibration file.\n");
        return;
    }

    fs["camera_matrix"] >> m_cameraCalibParams.cameraMatrix;
    fs["distortion_coefficients"] >> m_cameraCalibParams.distCoeffs;
    fs.release();
}

void BugDetector::Tick()
{
    cv::Mat frame = consumeFifoBuffer();
    if (!frame.empty() && frame.data) {
        processImage(frame);
    }
    updateTransform();
}

void BugDetector::updateTransform()
{
    // Get the transform for the odometry frame
    try {
        m_transform = m_tfBuffer->lookupTransform("odom", "camera_link", tf2::TimePointZero);
        RCLCPP_INFO(m_logger, "Latest Transform from odom to camera_link: [X: %.2f, Y: %.2f, Z: %.2f]",
                    m_transform.transform.translation.x, m_transform.transform.translation.y,
                    m_transform.transform.translation.z);
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(m_logger, "Could not transform base_link to camera_link: %s", ex.what());
    }
}

void BugDetector::detectBugs(cv::Mat &frame)
{
    if (frame.empty()) {
        std::cout << "Could not open or find the image!" << std::endl;
        return;
    }

    // Convert to grayscale
    cv::Mat gray, inverted;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::bitwise_not(gray, inverted);
    // Vector to store the detected circles
    std::vector<cv::Vec3f> circles;

    // Detect circles using Hough Transform
    cv::HoughCircles(inverted, circles, cv::HOUGH_GRADIENT,
                     1,  // Accumulator resolution (same as input image)
                     25,  // Minimum distance between circles (adjust based on spacing)
                     26, // Canny high threshold (lower if circles are missed)
                     12, // Accumulator threshold (lower if detection is poor)
                     2,
                     150 // Min and max radius based on the circle size
    );                   // Min and max radius of circles

    if (circles.size()) {
        std::cout << "\ncircles!";
    }
    // Draw the detected circles
    for (size_t i = 0; i < circles.size(); i++) {
        m_bugManager.push(circles[i]);

        // ToDo - continue implementation below
        cv::Vec3f circle = circles[i];
        cv::Point center(cvRound(circle[0]), cvRound(circle[1]));
        int radius = cvRound(circle[2]);

        // Draw circle center
        cv::circle(frame, center, 3, cv::Scalar(0, 255, 0), -1); // Green dot
        // Draw circle outline
        cv::circle(frame, center, radius, cv::Scalar(0, 0, 255), 2); // Red circle
    }
}

cv::Mat BugDetector::undistortImage(cv::Mat &image)
{
    cv::Mat imageUndistorted;
    cv::undistort(image.clone(), imageUndistorted, m_cameraCalibParams.cameraMatrix, m_cameraCalibParams.distCoeffs);
    return imageUndistorted;
}

void BugDetector::processImage(cv::Mat &image)
{
    // Processing code here
    if (!image.empty()) {
        image = undistortImage(image);
        detectBugs(image);
        cv::imshow("Camera Frame", image);
        cv::waitKey(100); // Wait for a short time to allow OpenCV to process the display
    }
}

cv::Mat BugDetector::consumeFifoBuffer()
{
    uint64_t timeNowMs = RCL_NS_TO_MS(rclcpp::Clock().now().nanoseconds()) - m_startTimeMs;
    if (!m_imageTupleBuffer.empty()) {
        ImageTimestampTuple imgTuple = m_imageTupleBuffer.front(); // Get the first image
        m_imageTupleBuffer.pop();                                  // Remove the image from the buffer
        uint64_t timestamp = imgTuple.timestampMillisecs;
        uint64_t timestampDiff = timeNowMs - timestamp;
        if (timestampDiff < 2000) {
            return imgTuple.frame;
        }
        else {
            RCLCPP_WARN(m_logger, "Timestamp too old: %2ld milliseconds already passed", timestampDiff);
            RCLCPP_WARN(m_logger, "Size of ImageBuffer: %zu images available", m_imageTupleBuffer.size());

            // Take the next frame if too old
            consumeFifoBuffer();
        }
    }
    return cv::Mat();
}

void BugDetector::AddImage(ImageTimestampTuple imgTuple)
{
    // Process the camera frame here
    // For example, display itcv::Mat frame
    m_imageTupleBuffer.push(imgTuple);
}
} // namespace patterns
