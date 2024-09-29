#pragma once
#ifndef BUGDETECTOR_H
#define BUGDETECTOR_H

#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <bug_factory.h>

namespace patterns
{

class BugDetector : public rclcpp_lifecycle::LifecycleNode
{
public:
    BugDetector(uint8_t id);

private:
    void cameraFrameSubCb(const sensor_msgs::msg::Image::SharedPtr imgMsg);

    uint8_t m_id;
    rclcpp::Time m_startTime;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_cameraFrameSub;
};

} // namespace patterns
#endif