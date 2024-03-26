/************************************************************************
 * Date:        2023.02
 * Author:      Haokun Wang <hwangeh at connect dot ust dot hk>, 
                Aerial Robotics Group <https://uav.ust.hk>, HKUST.
 * E-mail:      hwangeh_at_connect_dot_ust_dot_hk
 * Description: This is the source file for the GlobalPlanner class, which
 *              is used to generate the global plan for the UAV with
 *              suspended payloads.
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
 
#include "manager/global_plan.h"

namespace manager {
void GlobalPlanner::init(ros::NodeHandle& nh) {
	// init planner parameters
	planner_params_.trigger_flag = false;
	planner_params_.have_map = false;
	planner_params_.trigger_topic = "/move_base_simple/goal";
	planner_params_.traj_topic = "/planning/trajectory_info";
	// load user defined parameters
	nh.param("global_plan/map_topic", planner_params_.map_topic,
	         std::string("/env_generator/global_cloud"));
	nh.param("manager/start_pos/x", planner_params_.start_pos(0), 0.0);
	nh.param("manager/start_pos/y", planner_params_.start_pos(1), 0.0);
	nh.param("manager/start_pos/z", planner_params_.start_pos(2), 0.0);
	nh.param("manager/goal_pos/x", planner_params_.goal_pos(0), 8.0);
	nh.param("manager/goal_pos/y", planner_params_.goal_pos(1), 0.0);
	nh.param("manager/goal_pos/z", planner_params_.goal_pos(2), 0.0);
	// init planner manager
	plan_manager_.reset(new PlanManager);
	plan_manager_->init(nh);
	// init publisher and subscriber
	trigger_sub_ = nh.subscribe(planner_params_.trigger_topic, 1,
	                            &GlobalPlanner::triggerCallback, this);
	map_sub_ = nh.subscribe(planner_params_.map_topic, 1,
	                        &GlobalPlanner::mapCallback, this);
	traj_info_ = nh.advertise<impact_plan::PolynomialTraj>(
	    planner_params_.traj_topic, 10);
	// init planner timer
	exec_timer_ = nh.createTimer(ros::Duration(0.01),
	                             &GlobalPlanner::execPlanCallback, this);
}

void GlobalPlanner::polyTraj2ROSMsg(impact_plan::PolynomialTraj& msg) {
	msg.trajectory_id = 1;
	msg.action = msg.ACTION_ADD;
	int piece_num = plan_manager_->load_traj_->getPieceNum();
	Eigen::MatrixXd cMat(6, 8);

	for (int i = 0; i < piece_num; ++i) {
		impact_plan::PolynomialMatrix piece;
		piece.num_dim = 6;
		piece.num_order = 7;
		piece.duration = plan_manager_->load_traj_->getPiece(i).getDuration();
		auto cMat1 = plan_manager_->load_traj_->getPiece(i).getCoeffMat();
		auto cMat2 = plan_manager_->quad_traj_->getPiece(i).getCoeffMat();
		cMat.topRows(3) = cMat1;
		cMat.bottomRows(3) = cMat2;
		piece.data.assign(cMat.data(), cMat.data() + cMat.rows() * cMat.cols());
		msg.trajectory.emplace_back(piece);
	}
	msg.header.stamp = ros::Time::now(); // start time;
	return;
}

void GlobalPlanner::triggerCallback(const geometry_msgs::PoseStampedPtr& msg) {
	ROS_WARN("Start up!");
	planner_params_.trigger_flag = true;
}

void GlobalPlanner::mapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
	if (!planner_params_.have_map) {
		planner_params_.have_map = true;
		ROS_WARN("Recieve map!");
	}
}

void GlobalPlanner::execPlanCallback(const ros::TimerEvent& e) {
	exec_timer_.stop();
	if (planner_params_.trigger_flag && planner_params_.have_map) {
		planner_params_.trigger_flag = false;

		bool success = plan_manager_->optimalTraj(planner_params_.start_pos,
		                                          planner_params_.goal_pos);
		if (success) {
			impact_plan::PolynomialTraj traj_msg;
			polyTraj2ROSMsg(traj_msg);
			traj_info_.publish(traj_msg);
		}
	}
	exec_timer_.start();
}
} // namespace manager
