// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <string>
#include <vector>
#include <algorithm>
#include <memory>

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
#include <tf/tf.h>

#include <Eigen/Dense>

#include "utils.h"
#include "pseudo_inverse.h"

const double TAU = 2 * M_PI;

const int FREQ = 2;
const double DELTA_T = 0.001;

const double MAX_X = 0.7 * 0.1;
const double MAX_TH = 0.7 * M_PI / 8.0;
const double MAX_JOINT = 0.175;

const double WHEEL_R = 0.0625;

const double BASE_D = 0.365;

const double ALPHA = 0.01;

const double x_ref = -0.03;
const double y_ref = 0.02;

namespace path_controller_sim
{

	class PathControllerSim : public controller_interface::MultiInterfaceController<
										  hardware_interface::PositionJointInterface,
										  franka_hw::FrankaModelInterface,
										  franka_hw::FrankaStateInterface>
	{
	public:
		bool init(hardware_interface::RobotHW *robot_hardware, ros::NodeHandle &node_handle) override;
		void starting(const ros::Time &) override;
		void update(const ros::Time &, const ros::Duration &period) override;


	private:

		Eigen::VectorXd nextState(Eigen::VectorXd q_dot);
		void tfTransform(const tf2_msgs::TFMessage::ConstPtr &msg);
		void odomCb(const nav_msgs::Odometry::ConstPtr &odom);
		Eigen::VectorXd feedbackControl(Eigen::MatrixXd Xse, Eigen::MatrixXd Xsd, Eigen::MatrixXd Xsd_next);
		void pubCmd(const Eigen::VectorXd controls);
		void initStart();
		void setCommand(Eigen::VectorXd q_dot);
		Eigen::VectorXd redundancyResolution(Eigen::MatrixXd J_arm, Eigen::VectorXd Ves_arm);
		std::vector<Eigen::MatrixXd> circle();
		std::vector<Eigen::MatrixXd> wave();
		std::vector<Eigen::MatrixXd> line_();
		void initMarker();
		void addMarker(float x, float y, float z);
		void initLine();
		void addLine(geometry_msgs::Pose line_);

		hardware_interface::PositionJointInterface *position_joint_interface_;
		std::vector<hardware_interface::JointHandle> position_joint_handles_;
		ros::Duration elapsed_time_;
		std::array<double, 7> initial_pose_{};

		std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
		std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;

		ros::Subscriber sub_odom;
		ros::Subscriber sub_goal;
		ros::Subscriber sub_tf;

		ros::Publisher pub_vis;
		ros::Publisher pub_line;
		ros::Publisher pub_cmd;

		geometry_msgs::Pose robot_pose;
		geometry_msgs::Twist robot_twist;
		Eigen::Matrix4d Xsb;

		visualization_msgs::Marker marker;
		visualization_msgs::Marker line;

		tf::TransformListener listener;

		bool running_flag;

		Eigen::MatrixXd goal;
		Eigen::MatrixXd start;
		Eigen::MatrixXd curr_desired;

		std::vector<Eigen::MatrixXd> eef_traj;

		Eigen::Matrix<double, 6, 1> Vse;

		Eigen::Matrix<double, 7, 1> curr_q;
		Eigen::Matrix<double, 7, 1> q_d_nullspace_;

		Eigen::Matrix4d hand_pose;

		double kp_arm;
		double kp_base;
		double g_x;
		double g_y;
		double g_z;
		int number_of_path_points;

		double weight_arm;
		int redundancy_method;
		double nullspace_stiffness_;
        double circle_radius;
		int trajectory;

		std::string cmd_topic;
		std::string eef_link;
		std::string panda_link0;

		double roll;
		double pitch;
		double yaw;

		double scale;
		bool eef_set;
		double base_yaw;
	};

} // namespace franka_example_controllers
