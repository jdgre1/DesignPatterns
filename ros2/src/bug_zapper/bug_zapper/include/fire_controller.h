#pragma once
#ifndef FIRECONTROLLER_H
#define FIRECONTROLLER_H

#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <bug_detection.h>
#include <bug_tracker.h>
#include <bug_zapper/msg/fire_command.hpp>

namespace patterns
{

class FireController
{

public:
    explicit FireController(rclcpp::Publisher<bug_zapper::msg::FireCommand>::SharedPtr fireCommandPub, rclcpp::Logger logger);

    // Member functions
    void fire(uint64_t &timeStart, uint64_t &timeEnd); // Add an element

private:
    rclcpp::Logger m_logger;
    rclcpp::Publisher<bug_zapper::msg::FireCommand>::SharedPtr m_fireCommandPub;
};

} // namespace patterns
#endif