#include <bug_zapper.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    // Create a ROS 2 node

    auto bug_zapper_node = std::make_shared<patterns::BugZapper>(1);
    std::shared_ptr<patterns::BugDetector> bug_detector = std::make_shared<patterns::BugDetector>(1);

    bug_zapper_node->SetDetector(bug_detector);
    executor.add_node(bug_zapper_node->get_node_base_interface());

    rclcpp::Rate rate(10);

    while (rclcpp::ok()) {
        executor.spin_once();
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}