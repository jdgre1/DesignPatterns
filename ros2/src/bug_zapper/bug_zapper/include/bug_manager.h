#pragma once
#ifndef BUGMANAGER_H
#define BUGMANAGER_H

#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <bug_zapper_msgs/msg/bug_detection.hpp>
#include <bug_tracker.h>

namespace patterns
{

class BugManager
{

public:

    BugManager();
    // Member functions
    void push(const bug_zapper_msgs::msg::BugDetection &detection); // Add an element
    void pop();                           // Remove the last element
    void erase(int index);                // Remove an element at a specific index
    int size() const;                     // Get current size
    bool empty() const;                   // Check if empty
    // cv::Vec3f at(int index);             // Access an element
    void clear(); // Clear all elements
    void processDetections();
    void setDetectedBugs(std::vector<bug_zapper_msgs::msg::BugDetection>& detections)
    {
        m_detections = detections;
    }
    void Tick(uint64_t &timeNowMs);
    void processBugs(uint64_t &timeNowMs);

private:
    rclcpp::Logger m_logger;
    std::unique_ptr<BugTracker> m_bugTracker;
    float m_timesToFireAtBugsMs[NUMBER_OF_BUGS]; 
    std::vector<bug_zapper_msgs::msg::BugDetection> m_detections;

};

} // namespace patterns
#endif