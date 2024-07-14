/************************************************************************
 * Date:        2023.02
 * Author:      Haokun Wang <hwangeh at connect dot ust dot hk>, 
                Aerial Robotics Group <https://uav.ust.hk>, HKUST.
 * E-mail:      hwangeh_at_connect_dot_ust_dot_hk
 * Description: This is the header file for the RayCaster class, which is
 *              used to cast rays in the 3D space. Modified from the
 *              original code in project Fast-Planner 
 *              <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
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

#ifndef RAYCAST_H_
#define RAYCAST_H_

#include <cmath>
#include <vector>
#include <iostream>

#include <Eigen/Eigen>

double signum(double x);

double mod(double value, double modulus);

double intbound(double s, double ds);

void Raycast(const Eigen::Vector3d &start, const Eigen::Vector3d &end, const Eigen::Vector3d &min,
             const Eigen::Vector3d &max, int &output_points_cnt, Eigen::Vector3d *output);

void Raycast(const Eigen::Vector3d &start, const Eigen::Vector3d &end, const Eigen::Vector3d &min,
             const Eigen::Vector3d &max, std::vector<Eigen::Vector3d> *output);

class RayCaster
{
private:
    /* data */
    Eigen::Vector3d start_;
    Eigen::Vector3d end_;
    Eigen::Vector3d direction_;
    Eigen::Vector3d min_;
    Eigen::Vector3d max_;
    int x_;
    int y_;
    int z_;
    int endX_;
    int endY_;
    int endZ_;
    double maxDist_;
    double dx_;
    double dy_;
    double dz_;
    int stepX_;
    int stepY_;
    int stepZ_;
    double tMaxX_;
    double tMaxY_;
    double tMaxZ_;
    double tDeltaX_;
    double tDeltaY_;
    double tDeltaZ_;
    double dist_;

    int step_num_;

public:
    RayCaster(/* args */)
    {
    }
    ~RayCaster()
    {
    }

    bool setInput(const Eigen::Vector3d &start,
                  const Eigen::Vector3d &end /* , const Eigen::Vector3d& min,
                  const Eigen::Vector3d& max */
    );

    bool step(Eigen::Vector3d &ray_pt);
};

#endif // RAYCAST_H_