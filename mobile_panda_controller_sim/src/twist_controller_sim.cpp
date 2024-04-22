// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <mobile_panda_controller_sim/twist_controller_sim.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace twist_controller_sim
{

	bool TwistControllerSim::init(hardware_interface::RobotHW *robot_hardware,
										  ros::NodeHandle &node_handle)
	{
		position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
		if (position_joint_interface_ == nullptr)
		{
			ROS_ERROR(
				"JointPositionExampleController: Error getting position joint interface from hardware!");
			return false;
		}
		std::vector<std::string> joint_names;
		if (!node_handle.getParam("joint_names", joint_names))
		{
			ROS_ERROR("JointPositionExampleController: Could not parse joint names");
		}
		if (joint_names.size() != 7)
		{
			ROS_ERROR_STREAM("JointPositionExampleController: Wrong number of joint names, got "
							 << joint_names.size() << " instead of 7 names!");
			return false;
		}
		position_joint_handles_.resize(7);
		for (size_t i = 0; i < 7; ++i)
		{
			try
			{
				position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
			}
			catch (const hardware_interface::HardwareInterfaceException &e)
			{
				ROS_ERROR_STREAM(
					"JointPositionExampleController: Exception getting joint handles: " << e.what());
				return false;
			}
		}

		auto *model_interface = robot_hardware->get<franka_hw::FrankaModelInterface>();
		if (model_interface == nullptr)
		{
			ROS_ERROR_STREAM(
				"CartesianImpedanceExampleController: Error getting model interface from hardware");
			return false;
		}
		try
		{
			model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
				model_interface->getHandle("panda_model"));
		}
		catch (hardware_interface::HardwareInterfaceException &ex)
		{
			ROS_ERROR_STREAM(
				"CartesianImpedanceExampleController: Exception getting model handle from interface: "
				<< ex.what());
			return false;
		}

		auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
		if (state_interface == nullptr)
		{
			ROS_ERROR("MobilePandaController: Could not get state interface from hardware");
			return false;
		}

		try
		{
			state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
				state_interface->getHandle("panda_robot"));
		}
		catch (const hardware_interface::HardwareInterfaceException &e)
		{
			ROS_ERROR_STREAM(
				"MobilePandaController: Exception getting state handle: " << e.what());
			return false;
		}

		node_handle.getParam("/cmd_topic", cmd_topic);
		node_handle.getParam("/eef_link", eef_link);
		node_handle.getParam("/kp", kp);
		node_handle.getParam("/kv", kv);

		initMarker();

		sub_odom = node_handle.subscribe("/odom", 1, &TwistControllerSim::odomCb, this);
		sub_cmd = node_handle.subscribe("/cmd_vel", 1, &TwistControllerSim::cmdCb, this);
		sub_tf = node_handle.subscribe("/tf", 1000000, &TwistControllerSim::tfTransform, this);

		pub_cmd = node_handle.advertise<geometry_msgs::Twist>(cmd_topic.c_str(), 1);
		pub_vis = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 1);

		ros::Duration(2.0).sleep();

		if (!eef_set)
		{
			ROS_ERROR_STREAM("Could not get eef_pose from tf tree");
			return false;
		}

		return true;
	}

	void TwistControllerSim::tfTransform(const tf2_msgs::TFMessage::ConstPtr &msg)
	{
		tf::StampedTransform base_hand, odom_eef;
		try
		{
			listener.waitForTransform("/base_link", eef_link.c_str(), ros::Time(0), ros::Duration(0.5));
			listener.lookupTransform("/base_link", eef_link.c_str(), ros::Time(0), base_hand);
			bl_hand << base_hand.getBasis()[0][0], base_hand.getBasis()[0][1], base_hand.getBasis()[0][2], base_hand.getOrigin()[0],
				base_hand.getBasis()[1][0], base_hand.getBasis()[1][1], base_hand.getBasis()[1][2], base_hand.getOrigin()[1],
				base_hand.getBasis()[2][0], base_hand.getBasis()[2][1], base_hand.getBasis()[2][2], base_hand.getOrigin()[2],
				0, 0, 0, 1;

			listener.waitForTransform("/odom", eef_link.c_str(), ros::Time(0), ros::Duration(0.5));
			listener.lookupTransform("/odom", eef_link.c_str(), ros::Time(0), odom_eef);
			eef_pose_ << odom_eef.getBasis()[0][0], odom_eef.getBasis()[0][1], odom_eef.getBasis()[0][2], odom_eef.getOrigin()[0],
				odom_eef.getBasis()[1][0], odom_eef.getBasis()[1][1], odom_eef.getBasis()[1][2], odom_eef.getOrigin()[1],
				odom_eef.getBasis()[2][0], odom_eef.getBasis()[2][1], odom_eef.getBasis()[2][2], odom_eef.getOrigin()[2],
				0, 0, 0, 1;
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}
		eef_set = true;
	}

	void TwistControllerSim::odomCb(const nav_msgs::Odometry::ConstPtr &odom)
	{
		// for angular base movements
		Eigen::Vector3d r(bl_hand(0, 3), bl_hand(1, 3), bl_hand(2, 3));
		Eigen::Vector3d w(0, 0, odom->twist.twist.angular.z);
		Eigen::Vector3d angular_move = w.cross(r);

		base_twist[0] = odom->twist.twist.linear.x + angular_move[0];
		base_twist[1] = odom->twist.twist.linear.y + angular_move[1];
		base_twist[2] = odom->twist.twist.linear.z + angular_move[2];

		base_twist[3] = 0;
		base_twist[4] = 0;
		base_twist[5] = 0;
	}

	void TwistControllerSim::cmdCb(const geometry_msgs::Twist::ConstPtr &cmd)
	{
		// for angular base movements
		Eigen::Vector3d r(bl_hand(0, 3), bl_hand(1, 3), bl_hand(2, 3));
		Eigen::Vector3d w(0, 0, cmd->angular.z);
		Eigen::Vector3d angular_move = w.cross(r);

		cmd_twist[0] = cmd->linear.x + angular_move[0];
		cmd_twist[1] = cmd->linear.y + angular_move[1];
		cmd_twist[2] = cmd->linear.z + angular_move[2];

		cmd_twist[3] = 0;
		cmd_twist[4] = 0;
		cmd_twist[5] = 0;
	}

	Eigen::Matrix<double, 7, 1> TwistControllerSim::nextState(Eigen::VectorXd q_dot)
	{
		franka::RobotState robot_state = state_handle_->getRobotState();
		Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());

		Eigen::VectorXd del_theta = q_dot * DELTA_T;
		Eigen::VectorXd q_next = q + del_theta;
		curr_q = q_next * (ALPHA) + (1 - ALPHA) * curr_q;

		return curr_q;
	}

	void TwistControllerSim::starting(const ros::Time & /* time */)
	{
		for (size_t i = 0; i < 7; ++i)
		{
			initial_pose_[i] = position_joint_handles_[i].getPosition();
		}

		Eigen::Map<Eigen::Matrix<double, 7, 1>> q_temp(initial_pose_.data());
		curr_q = q_temp;
		start_pose = eef_pose_;

		elapsed_time_ = ros::Duration(0.0);
	}

	void TwistControllerSim::update(const ros::Time & /*time*/,
											const ros::Duration &period)
	{
		franka::RobotState robot_state = state_handle_->getRobotState();
		static int pub_counter = 0;

		// compute jacobian of arm in base frame
		std::array<double, 42> jacobian_array =
			model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
		Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

		elapsed_time_ += period;

		Eigen::Matrix<double, 6, 1> error;

		Eigen::Vector3d start_pos;
		start_pos << start_pose(0, 3), start_pose(1, 3), start_pose(2, 3);

		Eigen::Vector3d hand_pos;
		hand_pos << eef_pose_(0, 3), eef_pose_(1, 3), eef_pose_(2, 3);

		error.head(3) << hand_pos - start_pos;
		error[2] *= -1;
		error.tail(3) << 0, 0, 0;

		Eigen::Matrix<double, 6, 1> twist = kv * cmd_twist + kp * error;

		// compute pseudoInverse of jacobian
		Eigen::MatrixXd M_pinv;
		pseudoInverse(jacobian, M_pinv);

		Eigen::VectorXd q_dot = M_pinv * twist;

		Eigen::Matrix<double, 7, 1> q = nextState(q_dot);

		for (size_t i = 0; i < 7; ++i)
		{
			position_joint_handles_[i].setCommand(q[i]);
		}

		// publish cmd only with 10 Hz
		if (pub_counter % 100 == 0)
		{
			marker.points.clear();
			addMarker(start_pose(0, 3), start_pose(1, 3), start_pose(2, 3));
			addMarker(eef_pose_(0, 3), eef_pose_(1, 3), eef_pose_(2, 3));
			pub_vis.publish(marker);
		}
		pub_counter++;
	}

	void TwistControllerSim::addMarker(float x, float y, float z)
	{
		geometry_msgs::Point p;
		marker.pose.orientation.x = 0;
		marker.pose.orientation.y = 0;
		marker.pose.orientation.z = 0;
		marker.pose.orientation.w = 1;
		marker.color.a = 1; // halbdurchsichtig
		p.x = x;
		p.y = y;
		p.z = z;
		marker.points.push_back(p);
	}

	void TwistControllerSim::initMarker()
	{
		marker.header.frame_id = "odom";
		marker.header.stamp = ros::Time::now();
		marker.ns = "marker";
		marker.action = visualization_msgs::Marker::ADD;
		marker.id = 0;
		marker.type = visualization_msgs::Marker::POINTS;

		marker.scale.x = 0.08;
		marker.scale.y = 0.08;
		marker.scale.z = 0.08;
		marker.color.r = 0;
		marker.color.g = 0;
		marker.color.b = 1;
	}

} // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(twist_controller_sim::TwistControllerSim,
					   controller_interface::ControllerBase)
