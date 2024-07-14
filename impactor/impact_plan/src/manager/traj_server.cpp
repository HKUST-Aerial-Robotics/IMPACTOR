/************************************************************************
 * Date:        2023.02
 * Author:      Haokun Wang <hwangeh at connect dot ust dot hk>, 
                Aerial Robotics Group <https://uav.ust.hk>, HKUST.
 * E-mail:      hwangeh_at_connect_dot_ust_dot_hk
 * Description: This is the source file for the trajectories server, 
 *              which is used to manage the trajectory information and
 *              send the trajectory to the controller.
 * License:     GNU General Public License <http://www.gnu.org/licenses/>.
 * Project:     IMPACTOR is free software: you can redistribute it and/or 
 *              modify it under the terms of the GNU Lesser General Public 
 *              License as published by the Free Software Foundation, 
 *              either version 3 of the License, or (at your option) any 
 *              later version.
 *              IMPACTOR is distributed in the hope that it will be useful,
 *              but WITHOUT ANY WARRANTY; without even the implied warranty 
 *              of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 *              See the GNU General Public License for more details.
 * Website:     https://github.com/HKUST-Aerial-Robotics/IMPACTOR
 ************************************************************************/

#include "nav_msgs/Odometry.h"
#include "poly_traj/trajectory.hpp"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <ros/ros.h>
#include <sstream>

/* Initialization of the node */
// global info
struct globalINFO {
	ros::Publisher cmd_pub_, traj_pub_;
	std::vector<Eigen::Vector3d> pos_vec_, vel_vec_, acc_vec_, jerk_vec_;
	std::vector<double> time_vec_;
	ros::Time global_start_time_;
	double time_forward_, last_yaw_, last_yaw_dot_;
	bool receive_traj_ = false;
	bool publish_cmd_ = false;
	double pos_gain_[3] = {0, 0, 0};
	double vel_gain_[3] = {0, 0, 0};
} global_info_;
// tarjectory info
struct trajINFO {
	int traj_id_;
	double traj_duration_;
	ros::Time start_time_;
	poly_traj::Trajectory traj_;
} traj_info_;
// command info
struct CMD {
	int counter_ = 0;
	quadrotor_msgs::PositionCommand cmd_;
	std::pair<double, double> yaw_yawdot_;
	Eigen::Vector3d cmd_pos_;
	Eigen::Vector3d cmd_vel_;
	Eigen::Vector3d cmd_acc_;
	Eigen::Vector3d cmd_jerk_;
	Eigen::Vector3d cmd_pos_f_;
} cmd_info_;

void startCallback(const std_msgs::Bool::ConstPtr& start_msg) {
	if (start_msg->data == true) {
		ROS_WARN("Start Trajectory");
		global_info_.global_start_time_ = ros::Time::now();
	}
}

void finishCallback(const std_msgs::Bool::ConstPtr& finish_msg) {
	if (finish_msg->data == true) {
		ROS_WARN("Finish Trajectory");
	}
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d& pos,
                                        ros::Time& time_now,
                                        ros::Time& time_last) {
	constexpr double PI = 3.1415926;
	constexpr double YAW_DOT_MAX_PER_SEC = PI;
	// constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
	std::pair<double, double> yaw_yawdot(0, 0);
	double yaw = 0;
	double yawdot = 0;

	Eigen::Vector3d dir =
	    t_cur + global_info_.time_forward_ <= traj_info_.traj_duration_
	        ? traj_info_.traj_.getPos(t_cur + global_info_.time_forward_) -
	              cmd_info_.cmd_pos_
	        : traj_info_.traj_.getPos(traj_info_.traj_duration_) -
	              cmd_info_.cmd_pos_;
	double yaw_temp =
	    dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : global_info_.last_yaw_;
	double max_yaw_change =
	    YAW_DOT_MAX_PER_SEC * (time_now - time_last).toSec();
	if (yaw_temp - global_info_.last_yaw_ > PI) {
		if (yaw_temp - global_info_.last_yaw_ - 2 * PI < -max_yaw_change) {
			yaw = global_info_.last_yaw_ - max_yaw_change;
			if (yaw < -PI)
				yaw += 2 * PI;

			yawdot = -YAW_DOT_MAX_PER_SEC;
		} else {
			yaw = yaw_temp;
			if (yaw - global_info_.last_yaw_ > PI)
				yawdot = -YAW_DOT_MAX_PER_SEC;
			else
				yawdot = (yaw_temp - global_info_.last_yaw_) /
				         (time_now - time_last).toSec();
		}
	} else if (yaw_temp - global_info_.last_yaw_ < -PI) {
		if (yaw_temp - global_info_.last_yaw_ + 2 * PI > max_yaw_change) {
			yaw = global_info_.last_yaw_ + max_yaw_change;
			if (yaw > PI)
				yaw -= 2 * PI;

			yawdot = YAW_DOT_MAX_PER_SEC;
		} else {
			yaw = yaw_temp;
			if (yaw - global_info_.last_yaw_ < -PI)
				yawdot = YAW_DOT_MAX_PER_SEC;
			else
				yawdot = (yaw_temp - global_info_.last_yaw_) /
				         (time_now - time_last).toSec();
		}
	} else {
		if (yaw_temp - global_info_.last_yaw_ < -max_yaw_change) {
			yaw = global_info_.last_yaw_ - max_yaw_change;
			if (yaw < -PI)
				yaw += 2 * PI;

			yawdot = -YAW_DOT_MAX_PER_SEC;
		} else if (yaw_temp - global_info_.last_yaw_ > max_yaw_change) {
			yaw = global_info_.last_yaw_ + max_yaw_change;
			if (yaw > PI)
				yaw -= 2 * PI;

			yawdot = YAW_DOT_MAX_PER_SEC;
		} else {
			yaw = yaw_temp;
			if (yaw - global_info_.last_yaw_ > PI)
				yawdot = -YAW_DOT_MAX_PER_SEC;
			else if (yaw - global_info_.last_yaw_ < -PI)
				yawdot = YAW_DOT_MAX_PER_SEC;
			else
				yawdot = (yaw_temp - global_info_.last_yaw_) /
				         (time_now - time_last).toSec();
		}
	}

	if (fabs(yaw - global_info_.last_yaw_) <= max_yaw_change)
		yaw = 0.5 * global_info_.last_yaw_ + 0.5 * yaw; // nieve LPF
	yawdot = 0.5 * global_info_.last_yaw_dot_ + 0.5 * yawdot;
	global_info_.last_yaw_ = yaw;
	global_info_.last_yaw_dot_ = yawdot;

	yaw_yawdot.first = yaw;
	yaw_yawdot.second = yawdot;

	return yaw_yawdot;
}

void cmdCallback(const ros::TimerEvent& e) {
	// 
	if (!global_info_.receive_traj_)
		return;

	ros::Time time_now_ = ros::Time::now();
	double t_cur_ = (time_now_ - traj_info_.start_time_).toSec();

	static ros::Time time_last_ = ros::Time::now();
	if (global_info_.publish_cmd_ == false) {
		global_info_.publish_cmd_ = true;
	} else {
		cmd_info_.counter_++;
	}
	if (t_cur_ < traj_info_.traj_duration_ && t_cur_ >= 0.0) {
		cmd_info_.cmd_pos_ = traj_info_.traj_.getPos(t_cur_);
		cmd_info_.cmd_vel_ = traj_info_.traj_.getVel(t_cur_);
		cmd_info_.cmd_acc_ = traj_info_.traj_.getAcc(t_cur_);
		cmd_info_.cmd_jerk_ = traj_info_.traj_.getJer(t_cur_);

		/*** calculate yaw ***/
		cmd_info_.yaw_yawdot_ =
		    calculate_yaw(t_cur_, cmd_info_.cmd_pos_, time_now_, time_last_);

		double tf_ = std::min(traj_info_.traj_duration_, t_cur_ + 2.0);
		cmd_info_.cmd_pos_f_ = traj_info_.traj_.getPos(tf_);
	} else if (t_cur_ >= traj_info_.traj_duration_) {
		/* hover when finish trajectory */
		cmd_info_.cmd_pos_ = traj_info_.traj_.getPos(traj_info_.traj_duration_);
		cmd_info_.cmd_vel_.setZero();
		cmd_info_.cmd_acc_.setZero();

		cmd_info_.yaw_yawdot_.first = global_info_.last_yaw_;
		cmd_info_.yaw_yawdot_.second = 0.0;

		cmd_info_.cmd_pos_f_ = cmd_info_.cmd_pos_;
	} else {
		ROS_WARN("[traj_server] time_cur_ < 0");
	}
	time_last_ = time_now_;
	global_info_.time_vec_.push_back(
	    (ros::Time::now() - global_info_.global_start_time_).toSec());
	global_info_.pos_vec_.push_back(cmd_info_.cmd_pos_);
	global_info_.vel_vec_.push_back(cmd_info_.cmd_vel_);
	global_info_.acc_vec_.push_back(cmd_info_.cmd_acc_);
	global_info_.jerk_vec_.push_back(cmd_info_.cmd_jerk_);

	cmd_info_.cmd_.header.stamp = time_now_;
	cmd_info_.cmd_.header.frame_id = "world";
	cmd_info_.cmd_.trajectory_flag =
	    quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
	cmd_info_.cmd_.trajectory_id = traj_info_.traj_id_;

	cmd_info_.cmd_.position.x = cmd_info_.cmd_pos_(0);
	cmd_info_.cmd_.position.y = cmd_info_.cmd_pos_(1);
	cmd_info_.cmd_.position.z = cmd_info_.cmd_pos_(2);

	cmd_info_.cmd_.velocity.x = cmd_info_.cmd_vel_(0);
	cmd_info_.cmd_.velocity.y = cmd_info_.cmd_vel_(1);
	cmd_info_.cmd_.velocity.z = cmd_info_.cmd_vel_(2);

	cmd_info_.cmd_.acceleration.x = cmd_info_.cmd_acc_(0);
	cmd_info_.cmd_.acceleration.y = cmd_info_.cmd_acc_(1);
	cmd_info_.cmd_.acceleration.z = cmd_info_.cmd_acc_(2);

	cmd_info_.cmd_.yaw = cmd_info_.yaw_yawdot_.first;
	cmd_info_.cmd_.yaw_dot = cmd_info_.yaw_yawdot_.second;

	global_info_.last_yaw_ = cmd_info_.cmd_.yaw;

	global_info_.cmd_pub_.publish(cmd_info_.cmd_);
	ROS_WARN("send cmd.");
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "traj_server");
	// ros::NodeHandle node;
	ros::NodeHandle nh("~");

	ros::Subscriber reached_sub =
	    nh.subscribe("planning/finish", 10, finishCallback);
	ros::Subscriber start_sub =
	    nh.subscribe("planning/start", 10, startCallback);
	global_info_.cmd_pub_ =
	    nh.advertise<quadrotor_msgs::PositionCommand>("position_cmd", 50);

	ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

	/* control parameter */
	cmd_info_.cmd_.kx[0] = global_info_.pos_gain_[0];
	cmd_info_.cmd_.kx[1] = global_info_.pos_gain_[1];
	cmd_info_.cmd_.kx[2] = global_info_.pos_gain_[2];

	cmd_info_.cmd_.kv[0] = global_info_.vel_gain_[0];
	cmd_info_.cmd_.kv[1] = global_info_.vel_gain_[1];
	cmd_info_.cmd_.kv[2] = global_info_.vel_gain_[2];

	/* other parameter */
	cmd_info_.yaw_yawdot_.first = 0;
	cmd_info_.yaw_yawdot_.second = 0;
	cmd_info_.cmd_pos_ = Eigen::Vector3d(0, 0, 0);
	cmd_info_.cmd_vel_ = Eigen::Vector3d(0, 0, 0);
	cmd_info_.cmd_acc_ = Eigen::Vector3d(0, 0, 0);
	cmd_info_.cmd_jerk_ = Eigen::Vector3d(0, 0, 0);
	cmd_info_.cmd_pos_f_ = Eigen::Vector3d(0, 0, 0);

	nh.param("traj_server/time_forward", global_info_.time_forward_, -1.0);
	global_info_.last_yaw_ = 0.0;
	global_info_.last_yaw_dot_ = 0.0;

	ros::Duration(1.0).sleep();

	ROS_WARN("[Traj server]: ready.");

	ros::spin();

	return 0;
}
