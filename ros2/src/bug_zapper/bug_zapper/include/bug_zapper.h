#ifndef BUG_ZAPPER_H
#define BUG_ZAPPER_H
#pragma once

// bug_zapper class representing the entire tracking-, movement-control- and mapping-system of the robot to zap bugs

#include <iostream>
#include <camera.h>
#include <map.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
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

	private:
    	void cameraFrameSubCb(const sensor_msgs::msg::Image::SharedPtr imgMsg);

		// Map map;
		// MovementSystem mv_system;
		// MotorController mc;
    	rclcpp::Time m_startTime;
		rclcpp::TimerBase::SharedPtr m_tickTimer;  // Timer for Tick function

		std::shared_ptr<BugDetector> m_detector;
   		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_cameraFrameSub;

		//
		size_t m_id;



};













}
#endif
