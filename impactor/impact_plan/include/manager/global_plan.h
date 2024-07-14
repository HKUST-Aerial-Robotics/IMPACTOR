/************************************************************************
 * Date:        2023.02
 * Author:      Haokun Wang <hwangeh at connect dot ust dot hk>, 
                Aerial Robotics Group <https://uav.ust.hk>, HKUST.
 * E-mail:      hwangeh_at_connect_dot_ust_dot_hk
 * Description: This is the header file for the GlobalPlanner class, which 
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

#ifndef IMPACTOR_MANAGER_GLOBAL_PLAN_H_
#define IMPACTOR_MANAGER_GLOBAL_PLAN_H_

#include <vector>
#include <iostream>
#include <algorithm>

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

#include "impact_plan/PolynomialTraj.h"
#include "impact_plan/PolynomialMatrix.h"
#include "manager/plan_manager.h"
#include "utils/visualizer.hpp"

namespace manager
{
    class GlobalPlanner
    {

    private:
        struct PlannerParams
        {
            std::string trigger_topic, map_topic, traj_topic;
            bool trigger_flag, have_map;
            Eigen::Vector3d start_pos, goal_pos;
        } planner_params_;

        PlanManager::Ptr plan_manager_;

        ros::Timer exec_timer_;
        ros::Subscriber trigger_sub_, map_sub_;
        ros::Publisher traj_info_;

        void polyTraj2ROSMsg(impact_plan::PolynomialTraj &msg);
        void triggerCallback(const geometry_msgs::PoseStampedPtr &msg);
        void mapCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
        void execPlanCallback(const ros::TimerEvent &e);

    public:
        GlobalPlanner(/* args */) {}
        ~GlobalPlanner() {}

        void init(ros::NodeHandle &nh);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

} // namespace manager

#endif