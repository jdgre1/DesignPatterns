#include <bug_detector.h>

namespace patterns
{

BugDetector::BugDetector(uint8_t id) : m_id(id) {}

void BugDetector::Tick()
{
    cv::Mat frame = consumeFifoBuffer();
    if (!frame.empty() && frame.data) {
        processImage(frame);
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
    cv::imshow("Camera Frame Gray", inverted);

    // Detect circles using Hough Transform
    cv::HoughCircles(
        inverted, circles, cv::HOUGH_GRADIENT,
        1,               // Accumulator resolution (same as input image)
        1,              // Minimum distance between circles (adjust based on spacing)
        26,              // Canny high threshold (lower if circles are missed)
        12,              // Accumulator threshold (lower if detection is poor)
        2, 150           // Min and max radius based on the circle size
    );// Min and max radius of circles

    if (circles.size())
        std::cout << "\ncircles!";
    // Draw the detected circles
    for (size_t i = 0; i < circles.size(); i++) {
        cv::Vec3f circle = circles[i];
        cv::Point center(cvRound(circle[0]), cvRound(circle[1]));
        int radius = cvRound(circle[2]);

        // Draw circle center
        cv::circle(frame, center, 3, cv::Scalar(0, 255, 0), -1); // Green dot
        // Draw circle outline
        cv::circle(frame, center, radius, cv::Scalar(0, 0, 255), 2); // Red circle
    }
}

void BugDetector::processImage(cv::Mat &image)
{
    // Processing code here
    if (!image.empty()) {
        detectBugs(image);
        cv::imshow("Camera Frame", image);
        cv::waitKey(100); // Wait for a short time to allow OpenCV to process the display
    }
}

cv::Mat BugDetector::consumeFifoBuffer()
{
    if (!m_imageBuffer.empty()) {
        cv::Mat img = m_imageBuffer.front(); // Get the first image
        m_imageBuffer.pop();                 // Remove the image from the buffer
        return img;
    }
    return cv::Mat();
}

void BugDetector::AddImage(cv::Mat frame)
{
    // Process the camera frame here
    // For example, display it
    m_imageBuffer.push(frame);
}
} // namespace patterns
