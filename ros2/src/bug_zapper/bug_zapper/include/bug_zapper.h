#ifndef BUG_ZAPPER_H
#define BUG_ZAPPER_H
#pragma once

// bug_zapper class representing the entire tracking-, movement-control- and mapping-system of the robot to zap bugs

#include <iostream>
#include <camera.h>
#include <map.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <bug_tracker.h>
#include <bug_detector.h>


namespace patterns {

class BugZapper: public rclcpp_lifecycle::LifecycleNode
{
	public:
		BugZapper(uint8_t id);
		void Tick();
		void SetDetector(std::shared_ptr<BugDetector> det);
		// Getter function for tfBuffer
		// tf2_ros::Buffer& getTfBuffer()
		// {
		// 	return m_tfBuffer;
		// }

	private:
    	void cameraFrameSubCb(const sensor_msgs::msg::Image::SharedPtr imgMsg);
    	void odomSubCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
		void cmdVelSubCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
		void updateTransform();


		// Map map;
		// MovementSystem mv_system;
		// MotorController mc;
    	rclcpp::Time m_startTime;
		rclcpp::TimerBase::SharedPtr m_tickTimer;  // Timer for Tick function
    	geometry_msgs::msg::TransformStamped m_transform;

		std::shared_ptr<BugDetector> m_detector;
   		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_cameraFrameSub;
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_cmdVelSub;
    	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_odomSub;
		std::shared_ptr<tf2_ros::Buffer> m_tfBuffer;  // ToDo - change from shared-ptr
    	tf2_ros::TransformListener m_tfListener;  // Listener for the transforms
		//
		size_t m_id;



};













}
#endif
