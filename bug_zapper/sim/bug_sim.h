#pragma once
#ifndef BUGSIM_H
#define BUGSIM_H

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace patterns
{

class BugSim : public rclcpp_lifecycle::LifecycleNode
{
public:
    BugSim(double bugSpeedMin, double bugSpeedMax, uint8_t m_bugStrength);

    void start();

private:

    double m_bugSpeedMin;
    double m_bugSpeedMax;
    uint8_t m_bugStrength;

};

}  // namespace patterns
#endif