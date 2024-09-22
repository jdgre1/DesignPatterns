#pragma once
#ifndef BUGSIM_H
#define BUGSIM_H

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

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
    void DrawBug(std::shared_ptr<Bug> bug, cv::Mat &frame, const int &timePassedMs);

    double m_bugSpeedMin;
    double m_bugSpeedMax;
    uint8_t m_bugStrength;
    BugFactory m_bugfactory;

    rclcpp::Time m_startTime;
    rclcpp::TimerBase::SharedPtr m_timer;
    std::vector<std::shared_ptr<Bug>> m_bugs;
    const int m_bugSpawnIntervalMs = 4000;
    const int FIELD_WIDTH_PIXELS = 1000;
    const int FIELD_LENGTH_PIXELS = 1800;
    const int BUG_OFFSET_FROM_WIDTH_PIXELS = 50;
};

} // namespace patterns
#endif