#pragma once
#ifndef BUGSIM_H
#define BUGSIM_H

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

class BugSim : public rclcpp_lifecycle::LifecycleNode
{
public:
    BugSim(double bugSpeedMin, double bugSpeedMax, uint8_t m_bugStrength);

private:
    void simTimerCallback();
    void AddRandomBug(BugType &bugtype);
    void DrawBug(std::shared_ptr<Bug> bug, cv::Mat &frame);
    void drawCameraFrame(cv::Mat &cameraImage);
    void processBugs(cv::Mat &frame);

    // comfig
    double m_bugSpeedMin;
    double m_bugSpeedMax;
    uint8_t m_bugStrength;
    const int m_bugSpawnTickInterval = 10;
    const int FIELD_WIDTH_PIXELS = 800;
    const int FIELD_LENGTH_PIXELS = 1080;
    const int BUG_OFFSET_FROM_WIDTH_PIXELS = 50;
    // ~ config
    BugFactory m_bugfactory;

    int m_tickCounter = 0;
    rclcpp::Time m_startTime;
    rclcpp::TimerBase::SharedPtr m_timer;
    std::vector<std::shared_ptr<Bug>> m_bugs;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_cameraFramePub;
};

} // namespace patterns
#endif