/************************************************************************
 * Date:        2023.02
 * Author:      Haokun Wang <hwangeh at connect dot ust dot hk>, 
                Aerial Robotics Group <https://uav.ust.hk>, HKUST.
 * E-mail:      hwangeh_at_connect_dot_ust_dot_hk
 * Description: This is the header file for the Visualizer class, which is
 *              used to visualize the trajectory and the environment.
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

#ifndef _VISUALIZER_HPP
#define _VISUALIZER_HPP

#include "poly_traj/trajectory.hpp"
#include "utils/flatness.hpp"

#include <iostream>
#include <memory>
#include <chrono>
#include <cmath>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Visualizer for the planner
using std::vector;
namespace utils
{
    class Visualizer
    {
    private:
        // These are publishers for path, waypoints on the trajectory,
        // the entire trajectory, the mesh of free-space polytopes,
        // the edge of free-space polytopes, and spheres for safety radius
        ros::Publisher a_star_path_pub1, a_star_path_pub2;
        ros::Publisher init_path1, init_path2;
        ros::Publisher waypoint_pub1, waypoint_pub2;

        ros::Publisher ellipsoidPub;
        ros::Publisher tensionPub;
        // ros::Publisher accXPub, accYPub, accZPub;
        ros::Publisher distancePub;
        ros::Publisher loadPosPub, quadPosPub;
        ros::Publisher loadVelPub, quadVelPub;
        ros::Publisher loadAccPub, quadAccPub;
        ros::Publisher loadJerPub, quadJerPub;
        ros::Publisher loadSnaPub, quadSnaPub;
        ros::Publisher omgPub, anglePub, thrustPub;
        ros::Publisher meshPub, edgePub;
        // attitude representation transformation
        const double rad2deg = 180.0 / M_PI;
        const double deg2rad = M_PI / 180.0;

    public:
        typedef std::shared_ptr<Visualizer> Ptr;

        void init(ros::NodeHandle &nh);
        // Visualize the trajectory and its front-end path
        void displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                               Eigen::Vector4d color, int id, bool show_sphere = true, bool show_line = true);

        void visualizeDoubleball(const poly_traj::Trajectory::Ptr &trajL,
                                 const poly_traj::Trajectory::Ptr &trajQ,
                                 const int samples, const double &PayloadSize, const double &QuadrotorSize,
                                 const double &gravity, const double &load_mass, const bool &real_time = false);
        // Visualize all spheres with centers sphs and the same radius
        void displayAStarPath(vector<Eigen::VectorXd> init_pts, const double scale, int id);
        void displayInitPath(Eigen::MatrixXd init_pts, const double scale, int id);
        void displayWaypoints(vector<Eigen::VectorXd> waypoints, const double scale, int id);
        void quat2EulerRad(const Eigen::Vector4d &q, Eigen::Vector3d &euler);
        void normalizeAngles(const Eigen::Vector3d &angles, Eigen::Vector3d &normalized_angles);
        double normalizeAngle(double angle);
        void B2W(const Eigen::Vector3d &euler, Eigen::Vector3d &omg);
    };
}
#endif