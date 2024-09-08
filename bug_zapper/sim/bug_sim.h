#pragma once
#ifndef BUGSIM_H
#define BUGSIM_H

#include <iostream>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <bug.h>

namespace patterns
{


class BugSim : public rclcpp_lifecycle::LifecycleNode
{
public:
    BugSim(double bugSpeedMin, double bugSpeedMax, uint8_t m_bugStrength);
    

private:
    void simTimerCallback();

    double m_bugSpeedMin;
    double m_bugSpeedMax;
    uint8_t m_bugStrength;

    rclcpp::Time m_startTime;
    rclcpp::TimerBase::SharedPtr m_timer;
    std::vector<Bug> m_bugs;

};

}  // namespace patterns
#endif