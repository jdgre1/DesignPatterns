#include <bug_detector.h>


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    // Create a ROS 2 node
    auto bug_detect_node = std::make_shared<patterns::BugDetector>(1);
    // std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node = std::make_shared<SomeClass>();
    executor.add_node(bug_detect_node->get_node_base_interface());

    rclcpp::Rate rate(10);

    while(rclcpp::ok())
        executor.spin_once();
        rate.sleep();

    rclcpp::shutdown();
    return 0;
}