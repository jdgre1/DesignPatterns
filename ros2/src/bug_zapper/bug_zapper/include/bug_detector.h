#pragma once
#ifndef BUGDETECTOR_H
#define BUGDETECTOR_H

#include <iostream>
#include <queue>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <bug_zapper_msgs/msg/bug_detection.hpp>

namespace patterns
{
    struct ImageInfo
    {
        cv::Mat frame;
        uint64_t timestampMillisecs;
        uint64_t frameNumber;
    };

    struct CameraCalibrationParams
    {
        cv::Mat cameraMatrix;
        cv::Mat distCoeffs;
    };

class BugDetector
{
public:
    BugDetector(uint8_t id, std::shared_ptr<tf2_ros::Buffer> tfBuffer);
    cv::Mat undistortImage(cv::Mat &image);
    void AddImage(ImageInfo imgInfo);
    void setupCameraCalibrationConfig();
    std::vector<bug_zapper_msgs::msg::BugDetection> Tick(uint64_t &timeNowMs);

private:
    ImageInfo consumeFifoBuffer();
    void detectBugs(ImageInfo &frame);
    void processImage(ImageInfo &image);
    void updateTransform();

    CameraCalibrationParams m_cameraCalibParams;
    rclcpp::Logger m_logger;
    std::queue<ImageInfo> m_imageInfoBuffer;
    std::shared_ptr<tf2_ros::Buffer> m_tfBuffer;
    geometry_msgs::msg::TransformStamped m_transform;

    uint8_t m_id;
    uint64_t m_timeNowMs;
    std::vector<bug_zapper_msgs::msg::BugDetection> m_bugDetectionBuffer;
    size_t m_currentIdx = 0;

};

} // namespace patterns
#endif