/************************************************************************
 * Date:        2023.02
 * Author:      Haokun Wang <hwangeh at connect dot ust dot hk>, 
                Aerial Robotics Group <https://uav.ust.hk>, HKUST.
 * E-mail:      hwangeh_at_connect_dot_ust_dot_hk
 * Description: This is the header file for the MINCO class, which is
 *              used to solve the minimum energy trajectory optimization.
 *              The MINCO class is based on the paper: 
 *              @article{WANG2022GCOPTER,
 *              title={Geometrically Constrained Trajectory Optimization for Multicopters}, 
 *              author={Wang, Zhepei and Zhou, Xin and Xu, Chao and Gao, Fei}, 
 *              journal={IEEE Transactions on Robotics}, 
 *              year={2022}, 
 *              volume={38}, 
 *              number={5}, 
 *              pages={3259-3278}, 
 *              doi={10.1109/TRO.2022.3160022}
 *              }
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

#ifndef _MINCO_HPP
#define _MINCO_HPP

#include "poly_traj/trajectory.hpp"

#include <Eigen/Eigen>

#include <cmath>
#include <vector>
#include <memory>

namespace minco
{

    // The banded system class is used for solving
    // banded linear system Ax=b efficiently.
    // A is an N*N band matrix with lower band width lowerBw
    // and upper band width upperBw.
    // Banded LU factorization has O(N) time complexity.
    class BandedSystem
    {
    public:
        // The size of A, as well as the lower/upper
        // banded width p/q are needed
        void create(const int &n, const int &p, const int &q);
        void destroy(void);

    private:
        int N;
        int lowerBw;
        int upperBw;
        // Compulsory nullptr initialization here
        double *ptrData = nullptr;

    public:
        // Reset the matrix to zero
        void reset(void);
        // The band matrix is stored as suggested in "Matrix Computation"
        const double &operator()(const int &i, const int &j) const { return ptrData[(i - j + upperBw) * N + j]; }
        double &operator()(const int &i, const int &j) { return ptrData[(i - j + upperBw) * N + j]; }
        // This function conducts banded LU factorization in place
        // Note that NO PIVOT is applied on the matrix "A" for efficiency!!!
        void factorizeLU(void);
        // This function solves Ax=b, then stores x in b
        // The input b is required to be N*m, i.e.,
        // m vectors to be solved.
        template <typename EIGENMAT>
        void solve(EIGENMAT &b);
        // This function solves ATx=b, then stores x in b
        // The input b is required to be N*m, i.e.,
        // m vectors to be solved.
        template <typename EIGENMAT>
        void solveAdj(EIGENMAT &b);
    };

    class MINCO
    {
    public:
        ~MINCO() { A.destroy(); };
        MINCO(const int &order_, const int &dim_);

        typedef std::shared_ptr<MINCO> Ptr;

        int N;
        int state_dim_;
        int coefficient_num_;
        int traj_order_;
        int bound_conditions_num_;

    private:
        Eigen::MatrixXd headPVAJ;
        Eigen::MatrixXd tailPVAJ;
        BandedSystem A;
        Eigen::MatrixXd b;
        Eigen::MatrixXd t_;

    public:
        double factorial(double n);
        void get_q(const int i, const int j, const int s,
                   int &order, double &result);
        void get_r(const int i, const int j, const int s,
                   int &order, double &result);
        void get_order(const int &order, const int &total_order,
                       Eigen::VectorXd &coeff_list);
        void setConditions(const Eigen::Matrix<double, 6, 4> &kHeadState,
                           const Eigen::Matrix<double, 6, 4> &kTailState,
                           const int &kPieceNum);
        void setParameters(const Eigen::MatrixXd &kInnerPoints,
                           const Eigen::VectorXd &kTimeAllocation);
        void getTrajectory(poly_traj::Trajectory::Ptr &load_traj, poly_traj::Trajectory::Ptr &quad_traj);
        void getEnergy(double &energy);
        const Eigen::MatrixXd &getCoeffs(void) const { return b; }
        void getEnergyPartialGradByCoeffs(Eigen::MatrixXd &gdC);
        void getEnergyPartialGradByTimes(Eigen::VectorXd &gdT);
        void propogateGrad(const Eigen::MatrixXd &partialGradByCoeffs, const Eigen::VectorXd &partialGradByTimes,
                           Eigen::MatrixXd &gradByPoints, Eigen::VectorXd &gradByTimes);
    };
}

#endif