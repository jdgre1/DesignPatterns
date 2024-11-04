#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <chrono>
#include <memory>
#include <random>

#include <vehicle_sim.h>

namespace patterns {

class VehicleSimNode : public rclcpp::Node
{
public:
    VehicleSimNode() 
        : Node("vehicle_sim_node"), 
          m_vehicleSim(std::make_shared<VehicleSim>()),
          m_lastUpdateTime(this->now()),
          m_noiseGen(m_randomDevice()),
          m_noiseDist(-0.278, 0.278)  // Noise range for ±1 km/h in m/s
    {
        // Initialize publishers for velocity and pose
        m_cmdVelPub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        m_odomPub = this->create_publisher<geometry_msgs::msg::PoseStamped>("odom", 10);

        // Initialize transform broadcaster
        m_tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Timer to periodically update and publish data
        m_timer = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&VehicleSimNode::updateAndPublish, this)
        );

        // Set initial velocity
        m_vehicleSim->setVelocity(0.5, 0.0);  // Base velocity in x direction
    }

private:

    void publishCameraTransform()
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = "base_link";
        transform_stamped.child_frame_id = "camera_link";

        transform_stamped.transform.translation.x = 0.1;  // Example offset
        transform_stamped.transform.translation.y = 0.0;
        transform_stamped.transform.translation.z = 0.2;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);  // No rotation for simplicity
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        m_tfBroadcaster->sendTransform(transform_stamped);
    }

    void publishOdomTransform(double pos_x, double pos_y)
    {
        geometry_msgs::msg::TransformStamped odom_transform;
        odom_transform.header.stamp = this->now();
        odom_transform.header.frame_id = "odom";
        odom_transform.child_frame_id = "base_link";

        odom_transform.transform.translation.x = pos_x;
        odom_transform.transform.translation.y = pos_y;
        odom_transform.transform.translation.z = 0.0;  // Assuming 2D

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);  // Assuming no rotation
        odom_transform.transform.rotation.x = q.x();
        odom_transform.transform.rotation.y = q.y();
        odom_transform.transform.rotation.z = q.z();
        odom_transform.transform.rotation.w = q.w();

        m_tfBroadcaster->sendTransform(odom_transform);
    }

    void updateAndPublish()
    {
        // Calculate time difference since last update
        auto currentTime = this->now();
        double deltaTime = (currentTime - m_lastUpdateTime).seconds();
        m_lastUpdateTime = currentTime;

        // Apply random noise to the velocity to simulate slight fluctuations
        double noisyVelocityX = m_vehicleSim->getVelocityX() + m_noiseDist(m_noiseGen);
        double noisyVelocityY = m_vehicleSim->getVelocityY() + m_noiseDist(m_noiseGen);
        m_vehicleSim->setVelocity(noisyVelocityX, noisyVelocityY);

        // Update the vehicle's position based on velocity and elapsed time
        m_vehicleSim->updatePosition(deltaTime);

        // Get the current position and velocity after applying noise
        double posX = m_vehicleSim->getPositionX();
        double posY = m_vehicleSim->getPositionY();
        double velocityX = m_vehicleSim->getVelocityX();
        double velocityY = m_vehicleSim->getVelocityY();

        // Publish velocity
        geometry_msgs::msg::Twist velocityMsg;
        velocityMsg.linear.x = velocityX;
        velocityMsg.linear.y = velocityY;
        m_cmdVelPub->publish(velocityMsg);

        // Publish pose
        geometry_msgs::msg::PoseStamped poseMsg;
        poseMsg.header.stamp = currentTime;
        poseMsg.header.frame_id = "base_link";
        poseMsg.pose.position.x = posX;
        poseMsg.pose.position.y = posY;
        m_odomPub->publish(poseMsg);

        publishCameraTransform();
        publishOdomTransform(posX, posY);

        // // Broadcast transform between camera_link and base_link
        // geometry_msgs::msg::TransformStamped transformStamped;
        // transformStamped.header.stamp = currentTime;
        // transformStamped.header.frame_id = "base_link";
        // transformStamped.child_frame_id = "camera_link";

        // transformStamped.transform.translation.x = 0.1;  // Example offset, set as needed
        // transformStamped.transform.translation.y = 0.0;
        // transformStamped.transform.translation.z = 0.2;

        // tf2::Quaternion q;
        // q.setRPY(0, 0, 0);  // No rotation for simplicity
        // transformStamped.transform.rotation.x = q.x();
        // transformStamped.transform.rotation.y = q.y();
        // transformStamped.transform.rotation.z = q.z();
        // transformStamped.transform.rotation.w = q.w();

        // m_tfBroadcaster->sendTransform(transformStamped);
    }

    std::shared_ptr<VehicleSim> m_vehicleSim;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_cmdVelPub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_odomPub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tfBroadcaster;
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Time m_lastUpdateTime;

    // Random noise generation for velocity
    std::random_device m_randomDevice;
    std::mt19937 m_noiseGen;
    std::uniform_real_distribution<> m_noiseDist;
};

}  // namespace patterns
