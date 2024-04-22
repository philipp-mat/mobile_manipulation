#pragma once
#include <ros/ros.h>
#include "tf/transform_datatypes.h"
#include <tf2/LinearMath/Quaternion.h>

#include <tuple>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

using namespace Eigen;

std::tuple<double, double> projection(Vector2d line1, Vector2d line2, Vector2d p);

Eigen::MatrixXd poseToMatrix(const geometry_msgs::Pose &pose);

geometry_msgs::Pose matrixTopose(const Eigen::MatrixXd &m);

std::vector<double> vec2std(const Eigen::VectorXd vec1);

void pseudoInverseTest(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped);
