#pragma once
#ifndef BUGDETECTOR_H
#define BUGDETECTOR_H

#include <iostream>
#include <queue>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <cv_bridge/cv_bridge.h>

namespace patterns
{

class BugDetector
{
public:
    BugDetector(uint8_t id);//, tf2_ros::Buffer &tfBuffer);
    void AddImage(cv::Mat image);
    void Tick();

private:
    cv::Mat consumeFifoBuffer();
    void detectBugs(cv::Mat& frame);
    void processImage(cv::Mat& image);
    void updateTransform();

    std::queue<cv::Mat> m_imageBuffer;
    // tf2_ros::Buffer& m_tfBuffer; 
    geometry_msgs::msg::TransformStamped m_transform;
    uint8_t m_id;
};

} // namespace patterns
#endif