#include "../include/mobile_panda_controller_sim/utils.h"

std::tuple<double, double> projection(Vector2d start, Vector2d end, Vector2d p)
{
    Vector2d line = end - start;
    Vector2d to_p = p - start;
    double t = (line / line.norm()).dot(to_p);
    Vector2d proj = start + t * (line / line.norm());
    double dist = (proj - p).norm();
    return {dist, t};
}

Eigen::MatrixXd poseToMatrix(const geometry_msgs::Pose &pose)
{
    Eigen::MatrixXd P(4,4);
    Eigen::MatrixXd R;
    Eigen::Vector3d T(pose.position.x, pose.position.y, pose.position.z);
    
    Eigen::Quaternion<double> Q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    Q.normalize();
    R = Q.toRotationMatrix();

    P.setIdentity();

    P << R(0,0), R(0,1), R(0,2), T[0],
         R(1,0), R(1,1), R(1,2), T[1],
         R(2,0), R(2,1), R(2,2), T[2],
         0     , 0     , 0     , 1;

    return P;
}

geometry_msgs::Pose matrixTopose(const Eigen::MatrixXd &m)
{
    geometry_msgs::Pose P;
    Eigen::Matrix3d R;

    R << m(0,0), m(0,1), m(0,2),
         m(1,0), m(1,1), m(1,2),
         m(2,0), m(2,1), m(2,2);

    Eigen::Quaternion<double> Q(R);

    P.position.x = m(0,3);
    P.position.y = m(1,3);
    P.position.z = m(2,3);

    P.orientation.x = Q.x();
    P.orientation.y = Q.y();
    P.orientation.z = Q.z();
    P.orientation.w = Q.w();
    return P;
}

std::vector<double> vec2std(const Eigen::VectorXd vec1)
{
	std::vector<double> vec2;
	vec2.resize(vec1.size());
	Eigen::VectorXd::Map(&vec2[0], vec1.size()) = vec1;
	return vec2;
}

/*Eigen::VectorXd vec2Eigen(std::vector<double> vec1)
{
	Eigen::VectorXd vec2 = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(vec1.data(), vec1.size());
	return vec2;
}*/

void pseudoInverseTest(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped)
{
	double lamda_ = damped ? 0.2 : 0.0;

	Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
	Eigen::MatrixXd S_ = M_;
	S_.setZero();

	for(int i = 0; i < sing_vals_.size(); i++)
		S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lamda_ * lamda_);
	
	M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
}