#include <bug_detector.h>

namespace patterns
{

BugDetector::BugDetector(uint8_t id) : m_id(id) {}

void BugDetector::Tick()
{
    consumeFifoBuffer();
}

void BugDetector::processImage(const cv::Mat &img)
{
    // Processing code here
    cv::imshow("Camera Frame", img);
    cv::waitKey(100); // Wait for a short time to allow OpenCV to process the display
}

void BugDetector::consumeFifoBuffer()
{
    while (!m_imageBuffer.empty()) {
        cv::Mat img = m_imageBuffer.front(); // Get the first image
        m_imageBuffer.pop();                 // Remove the image from the buffer
        processImage(img);                   // Process the image
    }
}

void BugDetector::AddImage(const cv::Mat &frame)
{
    // Process the camera frame here
    // For example, display it
    m_imageBuffer.push(frame);
}
} // namespace patterns
