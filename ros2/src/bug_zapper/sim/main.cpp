#include <bug_sim.h>


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    // Create a ROS 2 node
    double bugSpeedMin = 1.0;
    double bugSpeedMax = 5.0;
    uint8_t bugStrength = 10;

    auto bug_sim_node = std::make_shared<patterns::BugSim>(bugSpeedMin, bugSpeedMax, bugStrength);
    // std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node = std::make_shared<SomeClass>();
    executor.add_node(bug_sim_node->get_node_base_interface());

    while(rclcpp::ok())
        executor.spin_once();


    rclcpp::shutdown();
    return 0;
}