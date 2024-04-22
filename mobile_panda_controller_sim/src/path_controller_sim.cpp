// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <mobile_panda_controller_sim/path_controller_sim.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace path_controller_sim
{

	bool PathControllerSim::init(hardware_interface::RobotHW *robot_hardware,
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

		initMarker();
		initLine();

		node_handle.getParam("/panda_link0", panda_link0);
		node_handle.getParam("/kp_arm", kp_arm);
		node_handle.getParam("/kp_base", kp_base);

		node_handle.getParam("/weight_arm", weight_arm);
		node_handle.getParam("/redundancy_method", redundancy_method);
		node_handle.getParam("/nullspace_stiffness_", nullspace_stiffness_);
		node_handle.getParam("/circle_radius", circle_radius);

		node_handle.getParam("/g_x", g_x);
		node_handle.getParam("/g_y", g_y);
		node_handle.getParam("/g_z", g_z);

		node_handle.getParam("/number_of_path_points", number_of_path_points);
		node_handle.getParam("/cmd_topic", cmd_topic);

		node_handle.getParam("/roll", roll);
		node_handle.getParam("/pitch", pitch);
		node_handle.getParam("/yaw", yaw);
		node_handle.getParam("/eef_link", eef_link);
		node_handle.getParam("/trajectory", trajectory);

		sub_tf = node_handle.subscribe("/tf", 1000000, &PathControllerSim::tfTransform, this);
		sub_odom = node_handle.subscribe("/odom", 1, &PathControllerSim::odomCb, this);

		pub_vis = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 1);
		pub_line = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 1);
		pub_cmd = node_handle.advertise<geometry_msgs::Twist>(cmd_topic.c_str(), 1);

		running_flag = true;
		eef_set = false;

		ros::Duration(2.0).sleep();

		if (!eef_set)
		{
			ROS_ERROR_STREAM("Could not get eef_pose from tf tree");
			return false;
		}
		
		initStart();

		switch (trajectory)
		{
		case 1:
		{
			eef_traj = line_();
			break;
		}
		case 2:
		{
			eef_traj = wave();
			break;
		}
		case 3:
		{
			eef_traj = circle();
			break;
		}
		default:
		{
			ROS_ERROR("Trajectory is invalid!");
			return false;
		}
		}

		return true;
	}
	
	void PathControllerSim::initStart()
	{
		start = hand_pose;
		curr_desired = hand_pose;
		goal = start;
		goal(0, 3) += g_x;
		goal(1, 3) += g_y;
		goal(2, 3) += g_z;
	}

	std::vector<Eigen::MatrixXd> PathControllerSim::line_()
	{
		std::vector<Eigen::MatrixXd> line_l;
        Eigen::Matrix3d rot;
        Eigen::Matrix3d start_orientation = start.topLeftCorner<3, 3>();
        Eigen::Vector3d start_euler = start_orientation.eulerAngles(0, 1, 2);
        double x = 0;
        double y = 0;
        double z = 0;
        double rot_x = 0;
        double rot_y = 0;
        double rot_z = 0;

        for (int i = 0; i < number_of_path_points; i++)
        {
            Eigen::MatrixXd point = start;
            point(0, 3) += x;
            point(1, 3) += y;
            point(2, 3) += z;
            rot = Eigen::AngleAxisd(start_euler[0] + rot_x, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(start_euler[1] + rot_y, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(start_euler[2] + rot_z, Eigen::Vector3d::UnitZ());
            point.topLeftCorner<3, 3>() = rot;
            rot_x += roll * M_PI / number_of_path_points;
            rot_y += pitch * M_PI / number_of_path_points;
            rot_z += yaw * M_PI / number_of_path_points;
            x += g_x / number_of_path_points;
            y += g_y / number_of_path_points;
            z += g_z / number_of_path_points;
            line_l.push_back(point);
        }
        return line_l;
	}

	std::vector<Eigen::MatrixXd> PathControllerSim::wave()
	{
		std::vector<Eigen::MatrixXd> wave;
        Eigen::Matrix3d rot;
        Eigen::Matrix3d start_orientation = start.topLeftCorner<3, 3>();
        Eigen::Vector3d start_euler = start_orientation.eulerAngles(0, 1, 2);
        double x = 0;
        double y = 0;
        double z = 0;
        double rot_x = 0;
        double rot_y = 0;
        double rot_z = 0;
        double phi = 0;
        double period = 1;

        for (int i = 0; i < number_of_path_points; i++)
        {
            Eigen::MatrixXd point = start;
            point(0, 3) += x;
            point(1, 3) += y;
            point(2, 3) += z - 0.1 + 0.1 * std::sin(period * (2 * M_PI / number_of_path_points) * i);
            rot = Eigen::AngleAxisd(start_euler[0] + rot_x, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(start_euler[1] + rot_y, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(start_euler[2] + rot_z, Eigen::Vector3d::UnitZ());
            point.topLeftCorner<3, 3>() = rot;
            rot_x += roll * M_PI / number_of_path_points;
            rot_y += pitch * M_PI / number_of_path_points;
            rot_z += yaw * M_PI / number_of_path_points;
            x += g_x / number_of_path_points;
            y += g_y / number_of_path_points;
            z += g_z / number_of_path_points;
            wave.push_back(point);
        }
        return wave;
	}

	std::vector<Eigen::MatrixXd> PathControllerSim::circle()
    {
        std::vector<Eigen::MatrixXd> circle;
        Eigen::Matrix3d rot;
        Eigen::Matrix3d start_orientation = start.topLeftCorner<3, 3>();
        Eigen::Vector3d start_euler = start_orientation.eulerAngles(0, 1, 2);
        double rot_x = 0;
        double rot_y = 0;
        double rot_z = 0;
		double period = 0.25;
        double period_z = 1; 

        for (int i = 0; i < number_of_path_points; i++)
        {
            Eigen::MatrixXd point = start;
            point(0, 3) += -circle_radius * std::sin(period * (2 * M_PI / number_of_path_points) * i);
            point(1, 3) += -circle_radius + circle_radius * std::cos(period * (2 * M_PI / number_of_path_points) * i);
            // point(2, 3) += -0.1 + 0.1 * std::sin(period_z * (2 * M_PI / number_of_path_points) * i);
            rot = Eigen::AngleAxisd(start_euler[0] + rot_x, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(start_euler[1] + rot_y, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(start_euler[2] + rot_z, Eigen::Vector3d::UnitZ());
            point.topLeftCorner<3, 3>() = rot;
            rot_x += roll * M_PI / number_of_path_points;
            rot_y += pitch * M_PI / number_of_path_points;
            rot_z += yaw * M_PI / number_of_path_points;
            circle.push_back(point);
        }
        return circle;
    }

	void PathControllerSim::odomCb(const nav_msgs::Odometry::ConstPtr &odom)
	{
		robot_pose = odom->pose.pose;
		base_yaw = tf::getYaw(robot_pose.orientation);
	}

	void PathControllerSim::pubCmd(const Eigen::VectorXd controls)
	{
		geometry_msgs::Twist cmd;
        cmd.linear.x = controls[0];
        cmd.angular.z = controls[1];
        pub_cmd.publish(cmd);
	}

	Eigen::VectorXd PathControllerSim::redundancyResolution(Eigen::MatrixXd J_arm, Eigen::VectorXd Ves_arm)
	{
        Eigen::VectorXd q_dot(7);
        Eigen::MatrixXd M_pinv_arm;

        franka::RobotState robot_state = state_handle_->getRobotState();
        Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

        switch (redundancy_method)
        {
        case 1:
        {
            pseudoInverse(J_arm, M_pinv_arm);
            q_dot << M_pinv_arm * Ves_arm;
            break;
        }
        case 2:
        {
            Eigen::VectorXd q_nullspace(7);

            pseudoInverse(J_arm, M_pinv_arm);

            // computed velocities here have no impact on the Endeffektor velocity
            // arm is held near the initial postion through PD controller
            q_nullspace << (Eigen::MatrixXd::Identity(7, 7) - M_pinv_arm * J_arm) * (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                                                                                     (2.0 * sqrt(nullspace_stiffness_)) * dq);

            q_dot << M_pinv_arm * Ves_arm + q_nullspace;
            break;
        }
        case 3:
        {
            Eigen::MatrixXd weight_a = Eigen::Matrix<double, 7, 7>::Identity();

            weight_a(0, 0) = weight_arm; // joint 1
            weight_a(1, 1) = weight_arm; // joint 2
            weight_a(2, 2) = weight_arm; // joint 3
            weight_a(3, 3) = weight_arm; // joint 4
            weight_a(4, 4) = weight_arm; // joint 5
            weight_a(5, 5) = weight_arm; // joint 6
            weight_a(6, 6) = weight_arm; // joint 7

            J_arm = J_arm * weight_a;
            pseudoInverse(J_arm, M_pinv_arm);
            q_dot << M_pinv_arm * Ves_arm;
            break;
        }
        default:
        {
            q_dot << 0, 0, 0, 0, 0, 0, 0;
            ROS_ERROR("No valid redundancy resolution method was chosen!");
        }
        }

		return q_dot;
	}

	Eigen::VectorXd PathControllerSim::feedbackControl(Eigen::MatrixXd Xse, Eigen::MatrixXd Xsd, Eigen::MatrixXd Xsd_next)
	{
		static int pub_counter = 0;

        Eigen::Matrix<double, 6, 1> error_arm;
		Eigen::Matrix<double, 2, 1> error_base;
        Eigen::VectorXd Xse_vec(3), Xsd_vec(3);

        Xse_vec << Xse(0, 3), Xse(1, 3), Xse(2, 3);
        Xsd_vec << Xsd(0, 3), Xsd(1, 3), Xsd(2, 3);

		// compute arm error
        Eigen::Quaterniond orientation(Xse.topLeftCorner<3, 3>());
        Eigen::Quaterniond orientation_d_(Xsd.topLeftCorner<3, 3>());

        // orientation error
        if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0)
        {
            orientation.coeffs() << -orientation.coeffs();
        }
        // "difference" quaternion
        Eigen::Quaterniond error_quaternion(orientation * orientation_d_.inverse());

        Eigen::AngleAxisd error_angle_axis(error_quaternion);

		// Position error
        error_arm.head(3) << Xse_vec - Xsd_vec;
        // error_arm[0] = 0;
        // error_arm[1] = 0;
        error_arm[2] *= -1;

        // Orientation error
        error_arm.tail(3) << error_angle_axis.axis() * error_angle_axis.angle();
        error_arm[5] *= -1;
		// orientation does not work in simulation --> set to zero
		error_arm.tail(3) << 0, 0, 0;

		// compute base error
		Eigen::VectorXd base_ref(2);
		base_ref[0] = x_ref;
		base_ref[1] = y_ref;
		Eigen::Matrix<double, 3, 3> rot_2d;
        rot_2d << std::cos(base_yaw), -std::sin(base_yaw),
                  std::sin(base_yaw), std::cos(base_yaw);
		base_ref = rot_2d * base_ref;

		Eigen::MatrixXd mat(2, 2);
		mat << base_ref[0] * std::cos(base_yaw) - base_ref[1] * std::sin(base_yaw), base_ref[0] * std::sin(base_yaw) + base_ref[1] * std::cos(base_yaw),
			   -std::sin(base_yaw), std::cos(base_yaw);
		mat *= 1 / base_ref[0];

        error_base[0] = Xsd_vec[0] - (robot_pose.position.x + base_ref[0]);
        error_base[1] = Xsd_vec[1] - (robot_pose.position.y + base_ref[1]);

		Eigen::VectorXd controls(2); 
		controls = mat * error_base;

        // velocity in World frame Ves [angular velocity, linear velocity (x,y,z)]
        Eigen::VectorXd Ves_arm = kp_arm * error_arm;
        Eigen::VectorXd base_vel = kp_base * controls;

        // compute jacobian of arm
        // Ja [6 x 7]
        std::array<double, 42> jacobian_array =
            model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
        Eigen::Map<Eigen::Matrix<double, 6, 7>> Ja(jacobian_array.data());

        Eigen::VectorXd q_dot = redundancyResolution(Ja, Ves_arm);
		Eigen::VectorXd joint_vel(9);
		joint_vel << base_vel, q_dot;

        if (pub_counter % 100 == 0)
        {
			// std::cout << "base_vel: " << base_vel << std::endl;
        }
        pub_counter++;

		return joint_vel;
	}

	void PathControllerSim::tfTransform(const tf2_msgs::TFMessage::ConstPtr &msg)
	{
		tf::StampedTransform hand_pose_;
		try
		{
			listener.lookupTransform("/odom", eef_link.c_str(), ros::Time(0), hand_pose_);
			hand_pose << hand_pose_.getBasis()[0][0], hand_pose_.getBasis()[0][1], hand_pose_.getBasis()[0][2], hand_pose_.getOrigin()[0],
				hand_pose_.getBasis()[1][0], hand_pose_.getBasis()[1][1], hand_pose_.getBasis()[1][2], hand_pose_.getOrigin()[1],
				hand_pose_.getBasis()[2][0], hand_pose_.getBasis()[2][1], hand_pose_.getBasis()[2][2], hand_pose_.getOrigin()[2],
				0, 0, 0, 1;
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}
		eef_set = true;
	}

	Eigen::VectorXd PathControllerSim::nextState(Eigen::VectorXd q_dot)
	{
		franka::RobotState robot_state = state_handle_->getRobotState();
		Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());

		Eigen::VectorXd del_theta = q_dot * DELTA_T;
		Eigen::VectorXd q_next = q + del_theta;
		curr_q = q_next * (ALPHA) + (1 - ALPHA) * curr_q;

		return curr_q;
	}

	void PathControllerSim::starting(const ros::Time & /* time */)
	{
		for (size_t i = 0; i < 7; ++i)
		{
			initial_pose_[i] = position_joint_handles_[i].getPosition();
		}
		Eigen::Map<Eigen::Matrix<double, 7, 1>> q_temp(initial_pose_.data());
		curr_q = q_temp;
		q_d_nullspace_ = q_temp;
		elapsed_time_ = ros::Duration(0.0);
	}

	void PathControllerSim::setCommand(Eigen::VectorXd q)
	{
		for (size_t i = 0; i < 7; ++i)
		{
			position_joint_handles_[i].setCommand(q[i]);
		}
	}

	void PathControllerSim::update(const ros::Time & /*time*/,
										   const ros::Duration &period)
	{
		static int pub_counter = 0;
		static int counter = 0;
		franka::RobotState robot_state = state_handle_->getRobotState();
		Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
		elapsed_time_ += period;

		// compute pose of eef in world with postion of base
		Eigen::MatrixXd Xse = hand_pose;

		if (running_flag && number_of_path_points > 1 && counter < eef_traj.size() - 1)
        {

            Eigen::MatrixXd Xsd = eef_traj[counter];
            Eigen::MatrixXd Xsd_next = eef_traj[counter + 1];

            Eigen::Vector3d curr(Xse(0, 3), Xse(1, 3), Xse(2, 3));
            Eigen::Vector3d goal_(goal(0, 3), goal(1, 3), goal(2, 3));
            Eigen::Vector3d desired_pose(Xsd(0, 3), Xsd(1, 3), Xsd(2, 3));
            Eigen::Vector3d next_pose(Xsd_next(0, 3), Xsd_next(1, 3), Xsd_next(2, 3));

            Eigen::Vector3d desired_next = next_pose - desired_pose;
            Eigen::Vector3d desired_next_perpendicular(desired_next[1], -desired_next[0], 0);
            Eigen::Vector3d desired_next_perpendicular2 = desired_pose + 100 * desired_next_perpendicular;

            Eigen::Vector2d first(desired_pose[0], desired_pose[1]);
            Eigen::Vector2d second(desired_next_perpendicular2[0], desired_next_perpendicular2[1]);
            Eigen::Vector2d hand(curr[0], curr[1]);

            auto [dist, t] = projection(first, second, hand);

            if (std::abs((goal_ - curr).norm()) < 0.05)
            {
                pub_cmd.publish(geometry_msgs::Twist());
                curr_desired = Xsd;
                running_flag = false;
            }
            else if (dist < 0.03)
            {
                curr_desired = Xsd;
                counter++;
            }
			else
			{
				Eigen::VectorXd q_dot = feedbackControl(Xse, Xsd, Xsd_next);

				Eigen::VectorXd q_next = nextState(q_dot.tail(7));

				setCommand(q_next);

				// publish cmd and visualisations only with 100Hz
				if (pub_counter % 100 == 0)
                {
                    pubCmd(q_dot.head(2));

                    // visualisation
                    marker.points.clear();
                    addMarker(Xsd(0, 3), Xsd(1, 3), Xsd(2, 3));
                    for (int i = 0; i < eef_traj.size(); ++i)
                    {
                        addMarker(eef_traj[i](0, 3), eef_traj[i](1, 3), eef_traj[i](2, 3));
                    }
                    pub_vis.publish(marker);
                    // visualisation
                }
			}
		}
		else
		{
			pub_cmd.publish(geometry_msgs::Twist());
			setCommand(curr_q);
		}
		pub_counter++;
	}

	void PathControllerSim::addMarker(float x, float y, float z)
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

	void PathControllerSim::initMarker()
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

	void PathControllerSim::addLine(geometry_msgs::Pose line_)
	{
		geometry_msgs::Point p;
		line.pose.orientation.x = 0;
		line.pose.orientation.y = 0;
		line.pose.orientation.z = 0;
		line.pose.orientation.w = 1;
		line.color.a = 1; // halbdurchsichtig
		p.x = line_.position.x;
		p.y = line_.position.y;
		p.z = line_.position.z;
		line.points.push_back(p);
	}

	void PathControllerSim::initLine()
	{
		line.header.frame_id = "odom";
		line.header.stamp = ros::Time::now();
		line.ns = "line";
		line.action = visualization_msgs::Marker::ADD;
		line.id = 1;
		line.type = visualization_msgs::Marker::LINE_LIST;
		// color Black
		line.color.g = 1;
		line.color.r = 0;
		line.color.b = 0;

		line.scale.x = 0.02;
		line.scale.y = 0.02;
		line.scale.z = 0.02;
	}

} // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(path_controller_sim::PathControllerSim,
					   controller_interface::ControllerBase)
