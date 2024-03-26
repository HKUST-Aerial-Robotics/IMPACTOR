/************************************************************************
 * Date:        2023.02
 * Author:      Haokun Wang <hwangeh at connect dot ust dot hk>, 
                Aerial Robotics Group <https://uav.ust.hk>, HKUST.
 * E-mail:      hwangeh_at_connect_dot_ust_dot_hk
 * Description: This is the source file for the PlanManager class, which
 *              is used to manage the trajectory planning.
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
 
#include <manager/plan_manager.h>

namespace manager {
void PlanManager::init(ros::NodeHandle& nh) {
	// load optimizer parameters
	manager_parameters_.polynomial_order = 7;
	manager_parameters_.state_dimension = 6;
	manager_parameters_.use_esdf_check = true;
	nh.param("manager/waypoints_num", manager_parameters_.waypoints_num, 0);
	nh.param("physical_params/cable_length", manager_parameters_.cable_length,
	         0.644);
	nh.param("manager/safe_margin", manager_parameters_.safe_margin, 0.2);

	Eigen::VectorXd wpt_;
	for (int i = 0; i < manager_parameters_.waypoints_num; i++) {
		wpt_.resize(6);
		nh.param("manager/waypoint_" + std::to_string(i) + "/load/x", wpt_(0),
		         0.0);
		nh.param("manager/waypoint_" + std::to_string(i) + "/load/y", wpt_(1),
		         0.0);
		nh.param("manager/waypoint_" + std::to_string(i) + "/load/z", wpt_(2),
		         0.0);
		nh.param("manager/waypoint_" + std::to_string(i) + "/quad/x", wpt_(3),
		         0.0);
		nh.param("manager/waypoint_" + std::to_string(i) + "/quad/y", wpt_(4),
		         0.0);
		nh.param("manager/waypoint_" + std::to_string(i) + "/quad/z", wpt_(5),
		         0.0);
		manager_parameters_.waypoints.push_back(wpt_);
	}
	// std::cout << "waypoints num: " << manager_parameters_.waypoints.size()
	//           << std::endl;
	/* initialize visualizer */
	visualizer_.reset(new utils::Visualizer);
	visualizer_->init(nh);
	/* initialize map modules */
	grid_map_.reset(new GridMap);
	grid_map_->initMap(nh);
	/* initialize optimizer */
	hybrid_opt_.reset(new alm_opt::HybridOPT);
	hybrid_opt_->init(nh);
	hybrid_opt_->initGridMap(grid_map_);
	/* initialize front-end*/
	a_star_.reset(new AStar);
	a_star_->initGridMap(grid_map_, Eigen::Vector3i(800, 200, 40));
}

void PlanManager::findPath(const Eigen::Vector3d& start_pos,
                           const Eigen::Vector3d& target_pos,
                           vector<Eigen::VectorXd>& kino_path) {
	vector<Eigen::Vector3d> load_path;
	Eigen::VectorXd state(6);
	for (int i = 1; i < manager_parameters_.waypoints_num; i++) {
		std::cout << "waypoint " << i - 1 << ": "
		          << manager_parameters_.waypoints[i - 1].transpose()
		          << std::endl;
		load_path.clear();
		load_path = a_star_->astarSearchAndGetSimplePath(
		    grid_map_->getResolution(),
		    manager_parameters_.waypoints[i - 1].head(3),
		    manager_parameters_.waypoints[i].head(3),
		    manager_parameters_.use_esdf_check);
		state.setZero();
		state = manager_parameters_.waypoints[i - 1];
		kino_path.push_back(state);
		for (int j = 1; j < load_path.size() - 1; j++) {
			state.setZero();
			state.head(3) << load_path[j];
			state.tail(3) << load_path[j];
			state(5) += manager_parameters_.safe_margin;
			kino_path.push_back(state);
		}
	}
	// std::cout << "waypoint " << manager_parameters_.waypoints_num - 1 << ": "
	//           << manager_parameters_
	//                  .waypoints[manager_parameters_.waypoints_num - 1]
	//                  .transpose()
	//           << std::endl;
	state.setZero();
	state =
	    manager_parameters_.waypoints[manager_parameters_.waypoints_num - 1];
	kino_path.push_back(state);

	visualizer_->displayWaypoints(manager_parameters_.waypoints, 0.2, 0);
}

bool PlanManager::optimalTraj(const Eigen::Vector3d& start_pos,
                              const Eigen::Vector3d& target_pos) {
	if ((start_pos - target_pos).norm() < 0.2) {
		cout << "Start and end position are too close" << endl;
		return false;
	}

	ros::Time t1, t2;
	manager_parameters_.time_search = 0.0;
	manager_parameters_.time_optimize = 0.0;

	// front-end path finding
	t1 = ros::Time::now();
	findPath(start_pos, target_pos, manager_parameters_.init_path);
	manager_parameters_.time_search = (ros::Time::now() - t1).toSec();

	visualizer_->displayAStarPath(manager_parameters_.init_path, 0.1, 0);

	// back-end optimization
	t2 = ros::Time::now();
	if (!hybrid_opt_->setup(manager_parameters_.init_path)) {
		cout << "Setup failed." << endl;
		return false;
	}

	visualizer_->displayInitPath(hybrid_opt_->info_params_.inner_pts, 0.2, 0);

	if (std::isinf(hybrid_opt_->optimize(visualizer_))) {
		cout << "Optimization failed." << endl;
		return false;
	}
	manager_parameters_.time_optimize = (ros::Time::now() - t2).toSec();

	visualizer_->visualizeDoubleball(hybrid_opt_->load_traj_,
	                                 hybrid_opt_->quad_traj_, 10, 0.1, 0.2,
	                                 9.81, 0.054, true);

	load_traj_ = hybrid_opt_->load_traj_;
	quad_traj_ = hybrid_opt_->quad_traj_;

	ROS_INFO("Path searching cost: %f, optimization cost: %f",
	         manager_parameters_.time_search,
	         manager_parameters_.time_optimize);
	return true;
}
} // namespace manager
