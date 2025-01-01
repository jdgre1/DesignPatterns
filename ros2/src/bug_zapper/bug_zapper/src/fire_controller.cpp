#include <fire_controller.h>

namespace patterns
{
FireController::FireController(rclcpp::Publisher<bug_zapper_msgs::msg::FireCommand>::SharedPtr fireCommandPub,
                               rclcpp::Logger logger)
    : m_fireCommandPub(fireCommandPub), m_logger(logger)
{}

void FireController::fire(std::vector<bug_zapper_msgs::msg::FireCommand>& fireCmdMsgs)
{
    for (bug_zapper_msgs::msg::FireCommand fireCmdMsg : fireCmdMsgs) {
        m_fireCommandPub->publish(fireCmdMsg);
    }
}

} // namespace patterns