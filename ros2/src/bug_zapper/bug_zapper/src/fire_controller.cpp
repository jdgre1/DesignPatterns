#include <fire_controller.h>

namespace patterns
{
FireController::FireController(rclcpp::Publisher<bug_zapper::msg::FireCommand>::SharedPtr fireCommandPub,
        rclcpp::Logger logger)
        : m_fireCommandPub(fireCommandPub), m_logger(logger) {}

void FireController::fire(uint64_t &timeStart, uint64_t &timeEnd) {}

} // namespace patterns