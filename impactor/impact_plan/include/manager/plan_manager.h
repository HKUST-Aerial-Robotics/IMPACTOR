/************************************************************************
 * Date:        2023.02
 * Author:      Haokun Wang <hwangeh at connect dot ust dot hk>, 
                Aerial Robotics Group <https://uav.ust.hk>, HKUST.
 * E-mail:      hwangeh_at_connect_dot_ust_dot_hk
 * Description: This is the header file for the PlanManager class, which
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

#ifndef _PLAN_MANAGER_H_
#define _PLAN_MANAGER_H_

#include "map/grid_map.h"
#include "path_finder/dyn_a_star.h"
#include "alm/alm_opt.hpp"
#include "utils/visualizer.hpp"

#include <ros/ros.h>

namespace manager
{
    class PlanManager
    {
    public:
        /* main planning interface */
        void init(ros::NodeHandle &nh);
        bool optimalTraj(const Eigen::Vector3d &start_state,
                         const Eigen::Vector3d &final_state);

        poly_traj::Trajectory::Ptr load_traj_;
        poly_traj::Trajectory::Ptr quad_traj_;

        ros::Time start_time_;

        typedef unique_ptr<PlanManager> Ptr;

    private:
        void findPath(const Eigen::Vector3d &start_pos,
                      const Eigen::Vector3d &target_pos,
                      vector<Eigen::VectorXd> &kino_path);

        /* Planner modules */
        ros::NodeHandle nh_;
        alm_opt::HybridOPT::Ptr hybrid_opt_;
        utils::Visualizer::Ptr visualizer_;

        // voxel_map::VoxelMap voxel_map_;
        GridMap::Ptr grid_map_;
        AStar::Ptr a_star_;

        struct ManagerParameters
        {
            /* manager parameters */
            int polynomial_order;
            int state_dimension;
            double cable_length;
            double safe_margin;
            bool use_esdf_check;
            int waypoints_num;
            vector<Eigen::VectorXd> waypoints;
            vector<Eigen::VectorXd> init_path;
            /* processing time */
            double time_search;
            double time_optimize;
        } manager_parameters_;
    };
}

#endif