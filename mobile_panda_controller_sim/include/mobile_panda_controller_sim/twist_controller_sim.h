// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <cstdlib>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka/robot_state.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <Eigen/Dense>
#include "pseudo_inverse.h"

const double DELTA_T = 0.001;
double ALPHA = 0.01;

namespace twist_controller_sim
{

	class TwistControllerSim : public controller_interface::MultiInterfaceController<
										   hardware_interface::PositionJointInterface,
										   franka_hw::FrankaModelInterface,
										   franka_hw::FrankaStateInterface>
	{
	public:
		bool init(hardware_interface::RobotHW *robot_hardware, ros::NodeHandle &node_handle) override;
		void starting(const ros::Time &) override;
		void update(const ros::Time &, const ros::Duration &period) override;
		void tfTransform(const tf2_msgs::TFMessage::ConstPtr &msg);
		void odomCb(const nav_msgs::Odometry::ConstPtr &base);
		void cmdCb(const geometry_msgs::Twist::ConstPtr &cmd);
		Eigen::Matrix<double, 7, 1> nextState(Eigen::VectorXd q_dot);

		void initMarker();
		void addMarker(float x, float y, float z);

	private:
		hardware_interface::PositionJointInterface *position_joint_interface_;
		std::vector<hardware_interface::JointHandle> position_joint_handles_;
		ros::Duration elapsed_time_;
		std::array<double, 7> initial_pose_{};

		std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
		std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;

		tf::TransformListener listener;
		Eigen::Matrix4d bl_hand;

		ros::Subscriber sub_odom;
		ros::Subscriber sub_cmd;
		ros::Subscriber sub_tf;

		ros::Publisher pub_cmd;
		ros::Publisher pub_vis;

		visualization_msgs::Marker marker;

		Eigen::Matrix<double, 6, 1> base_twist;
		Eigen::Matrix<double, 6, 1> cmd_twist;

		Eigen::Matrix4d eef_pose_;
		Eigen::Matrix<double, 7, 1> curr_q;
		Eigen::Matrix4d start_pose;

		double kp;
		double kv;

		std::string cmd_topic;
		std::string eef_link;

		bool eef_set;
	};

} // namespace franka_example_controllers
