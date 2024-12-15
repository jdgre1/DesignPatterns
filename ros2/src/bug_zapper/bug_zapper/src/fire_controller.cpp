#include <fire_controller.h>

namespace patterns
{
FireController::FireController(rclcpp::Publisher<bug_zapper_msgs::msg::FireCommand>::SharedPtr fireCommandPub,
                               rclcpp::Logger logger)
    : m_fireCommandPub(fireCommandPub), m_logger(logger)
{}

void FireController::fire(uint64_t &timeStart, uint64_t &timeEnd)
{
    bug_zapper_msgs::msg::FireCommand fireCmdMsg;
    fireCmdMsg.opening_time = timeStart;
    fireCmdMsg.closing_time = timeEnd;
}

} // namespace patterns