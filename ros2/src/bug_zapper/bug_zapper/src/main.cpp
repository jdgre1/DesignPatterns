#include <bug_zapper.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    // Create a ROS 2 node

    // Create the tf2 buffer and listener
    // Create the tf2 buffer with the node context
    auto bug_zapper_node = std::make_shared<patterns::BugZapper>(1);

    std::shared_ptr<tf2_ros::Buffer> tfBuffer = bug_zapper_node->getTfBuffer();
    // tf2_ros::TransformListener tfListener(tfBuffer, bug_zapper_node);

    // Create the VehicleSubscriber and pass the tfBuffer
    // auto bug_detector = std::make_shared<patterns::BugDetector>(std::make_shared<rclcpp::Node>("vehicle_subscriber_node"), tfBuffer);
    std::shared_ptr<patterns::BugDetector> bug_detector = std::make_shared<patterns::BugDetector>(1, tfBuffer);


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