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

namespace patterns
{
    struct ImageTimestampTuple
    {
        cv::Mat frame;
        uint64_t timestampMillisecs;
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
    void AddImage(ImageTimestampTuple imgTuple);
    void setupCameraCalibrationConfig();
    void Tick();

private:
    cv::Mat consumeFifoBuffer();
    void detectBugs(cv::Mat &frame);
    void processImage(cv::Mat &image);
    void updateTransform();

    CameraCalibrationParams m_cameraCalibParams;
    rclcpp::Logger m_logger;
    std::queue<ImageTimestampTuple> m_imageTupleBuffer;
    std::shared_ptr<tf2_ros::Buffer> m_tfBuffer;
    geometry_msgs::msg::TransformStamped m_transform;
    uint8_t m_id;
    uint64_t m_startTimeMs;

};

} // namespace patterns
#endif