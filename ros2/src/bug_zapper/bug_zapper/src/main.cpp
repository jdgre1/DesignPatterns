#include <bug_zapper.h>
#include <config.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto bug_zapper_node = std::make_shared<patterns::BugZapper>(1);

    std::shared_ptr<tf2_ros::Buffer> tfBuffer = bug_zapper_node->getTfBuffer();

    std::shared_ptr<patterns::BugDetector> bug_detector = std::make_shared<patterns::BugDetector>(1, tfBuffer);
    std::shared_ptr<patterns::BugRecorder> bug_recorder =
        patterns::BugRecorder::GetInstance(patterns::config::POST_ADDRESS);
    bug_zapper_node->SetDetector(bug_detector);
    bug_zapper_node->SetRecorder(bug_recorder);

    executor.add_node(bug_zapper_node->get_node_base_interface());

    rclcpp::Rate rate(10);

    while (rclcpp::ok()) {
        executor.spin_once();
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}