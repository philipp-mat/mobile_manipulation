#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

class BaseSimulation
{
public:
	BaseSimulation(ros::NodeHandle &nh, ros::NodeHandle &nh_priv)
	{
		position_x_ = nh_priv.param("initial_position_x", 0.0);
		position_y_ = nh_priv.param("initial_position_y", 0.0);
		orientation_ = nh_priv.param("initial_orientation", 0.0);
		sampling_time_ = 1.0 / nh_priv.param("controller_frequency", 20.0);

		twist_subscriber_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &BaseSimulation::onTwistReceived, this);
		joint_state_subscriber_ = nh.subscribe<sensor_msgs::JointState>("/joint_states", 1, &BaseSimulation::jointStates, this);

		joint_state_publisher_ = nh.advertise<sensor_msgs::JointState>("/simulation/base/joint_states", 1);
		joint_state_arm_publisher_ = nh.advertise<sensor_msgs::JointState>("/simulation/joint_states", 1);
		odom_publisher_ = nh.advertise<nav_msgs::Odometry>("/simulation/odom", 1);
		timer_ = nh.createTimer(ros::Duration(sampling_time_), &BaseSimulation::timerCallback, this);
	}

	void onTwistReceived(const geometry_msgs::TwistConstPtr &twist)
	{
		twist_ = *twist;
		// update orientation
		orientation_ += twist->angular.z * sampling_time_;
		// normalize orientation using atan2(y, x)
		double c = std::cos(orientation_);
		double s = std::sin(orientation_);
		orientation_ = std::atan2(s, c);
		// twist command is expressed in local frame of mobile base
		// update position in world frame
		position_x_ += (c * twist->linear.x) * sampling_time_;
		position_y_ += (s * twist->linear.x) * sampling_time_;
	}

	void timerCallback(const ros::TimerEvent &event)
	{
		publishJointState();
	}

	void publishJointState()
	{
		// joint state is expressed in world frame
		sensor_msgs::JointState msg;
		msg.name = {"base_x_joint", "base_y_joint", "base_yaw_joint"};
		msg.position = {position_x_, position_y_, orientation_};
		joint_state_publisher_.publish(msg);

		nav_msgs::Odometry odom;
		odom.pose.pose.position.x = position_x_;
		odom.pose.pose.position.y = position_y_;
		odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(orientation_);

		odom.twist.twist.linear.x = twist_.linear.x;
		odom.twist.twist.angular.z = twist_.angular.z;
		odom_publisher_.publish(odom);
	}

	void jointStates(const sensor_msgs::JointStateConstPtr &states)
	{
		sensor_msgs::JointState msg_;
		msg_.name = {"panda_sim_joint1", "panda_sim_joint2", "panda_sim_joint3", "panda_sim_joint4", "panda_sim_joint5", "panda_sim_joint6", "panda_sim_joint7"};
		msg_.position = {states->position[0], states->position[1], states->position[2], states->position[3], states->position[4], states->position[5], states->position[6]};
		joint_state_arm_publisher_.publish(msg_);
	}

private:
	double position_x_;
	double position_y_;
	double orientation_;
	double sampling_time_;

	geometry_msgs::Twist twist_;

	tf::TransformListener listener;
	ros::Subscriber twist_subscriber_;
	ros::Subscriber joint_state_subscriber_;
	ros::Subscriber tf_subscriber_;

	ros::Publisher joint_state_arm_publisher_;
	ros::Publisher joint_state_publisher_;
	ros::Publisher odom_publisher_;
	ros::Timer timer_;
};

int main(int argc, char **argv)
{
	// init ROS node
	ros::init(argc, argv, "base_simulation");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	// start simulation of mobile base
	BaseSimulation simulator(nh, nh_priv);
	ros::spin();
	return 0;
}