#pragma once
#ifndef FIRECONTROLLER_H
#define FIRECONTROLLER_H

#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <bug_tracker.h>
#include <bug_zapper_msgs/msg/fire_command.hpp>

namespace patterns
{

class FireController
{

public:
    explicit FireController(rclcpp::Publisher<bug_zapper_msgs::msg::FireCommand>::SharedPtr fireCommandPub, rclcpp::Logger logger);

    // Member functions
    void fire(std::vector<bug_zapper_msgs::msg::FireCommand>& fireCmdMsgs); // Add an element

private:
    rclcpp::Logger m_logger;
    rclcpp::Publisher<bug_zapper_msgs::msg::FireCommand>::SharedPtr m_fireCommandPub;
};

} // namespace patterns
#endif