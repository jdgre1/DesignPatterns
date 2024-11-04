#include <bug_sim.h>
#include <vehicle_sim_node.h>


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    // Create a ROS 2 node
    double bugSpeedMin = 1.0;
    double bugSpeedMax = 5.0;
    uint8_t bugStrength = 10;

    auto bug_sim_node = std::make_shared<patterns::BugSim>(bugSpeedMin, bugSpeedMax, bugStrength);
    auto vehicle_sim_node = std::make_shared<patterns::VehicleSimNode>();

    // std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node = std::make_shared<SomeClass>();
    executor.add_node(bug_sim_node->get_node_base_interface());
    executor.add_node(vehicle_sim_node->get_node_base_interface());

    rclcpp::Rate rate(10);

    while(rclcpp::ok())
        executor.spin_once();
        rate.sleep();

    rclcpp::shutdown();
    return 0;
}