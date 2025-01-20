#pragma once
#ifndef BUGSIM_H
#define BUGSIM_H

#include <iostream>
#include <queue>
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
#include <bug_zapper_msgs/msg/fire_command.hpp>

namespace patterns
{

struct FireCommandItem {
    rclcpp::Time opening_time;
    rclcpp::Time closing_time;
    bug_zapper_msgs::msg::FireCommand::SharedPtr fireCmdMsg;

    // Comparison operator for priority queue (min-heap)
    bool operator<(const FireCommandItem& other) const {
        return opening_time > other.opening_time;  // Reverse for min-heap
    }
};


class BugSim : public rclcpp_lifecycle::LifecycleNode
{
public:
    BugSim(double bugSpeedMin, double bugSpeedMax, uint8_t m_bugStrength);

private:
    void simTimerCallback();
    void fireCommandSubCallback(const bug_zapper_msgs::msg::FireCommand::SharedPtr fireCmdMsg);
    void AddRandomBug(BugType &bugtype);
    void DrawBug(std::shared_ptr<Bug> bug, cv::Mat &frame);
    void drawCameraFrame(cv::Mat &cameraImage);
    void processBugs(cv::Mat &frame);
    void processFireCommandQueue(cv::Mat &frame);
    void drawGunTriggers(cv::Mat &frame, uint8_t gunID);


    // comfig
    double m_bugSpeedMin;
    double m_bugSpeedMax;
    uint8_t m_bugStrength;

    // ~ config
    BugFactory m_bugfactory;

    int m_tickCounter = 0;
    rclcpp::Time m_timeNow;
    uint64_t m_startTimeMs;
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::TimerBase::SharedPtr m_fireCommandTimer;
    std::priority_queue<FireCommandItem> m_fireCommandQueue;

    std::vector<std::shared_ptr<Bug>> m_bugs;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_cameraFramePub;
    rclcpp::Subscription<bug_zapper_msgs::msg::FireCommand>::SharedPtr m_fireCommandSub;

};

} // namespace patterns
#endif