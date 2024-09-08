
#include <opencv2/core/core.hpp>

#include <bug_sim.h>

using namespace std::chrono_literals;
namespace patterns
{
    BugSim::BugSim(double bugSpeedMin, double bugSpeedMax, uint8_t bugStrength)
    : rclcpp_lifecycle::LifecycleNode("bug_sim_lifecycle_node")
        , m_bugSpeedMin(bugSpeedMin)
        , m_bugSpeedMax(bugSpeedMax)
        , m_bugStrength(bugStrength)
    {
        m_startTime = this->get_clock()->now();
        m_timer =  this->create_wall_timer(500ms, std::bind(&BugSim::simTimerCallback, this));

    }


    void BugSim::simTimerCallback()
    {
        cv::Mat cameraFrame(cv::Size(1000,800), CV_8UC3, cv::Scalar(255,255,255));


        

    }

    
}