
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
    }

    void BugSim::start()
    {
        rclcpp::Time now;// = this.get_clock()->now();
        //::now();
    
    }

    
}