/************************************************************************
 * Date:        2023.02
 * Author:      Haokun Wang <hwangeh at connect dot ust dot hk>, 
                Aerial Robotics Group <https://uav.ust.hk>, HKUST.
 * E-mail:      hwangeh_at_connect_dot_ust_dot_hk
 * Description: This is the header file for the ALM optimization module.
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

#ifndef _HYBRID_GCOPTER_HPP
#define _HYBRID_GCOPTER_HPP

#include "alm/minco.hpp"
#include "alm/lbfgs.hpp"
#include "utils/flatness.hpp"
#include "utils/visualizer.hpp"
#include "map/grid_map.h"

#include <std_msgs/Float64.h>
#include <ros/ros.h>

#include <Eigen/Eigen>

#include <cmath>
#include <cfloat>
#include <iostream>
#include <vector>
#include <fstream>

namespace alm_opt
{
    class Optimizer
    {
    protected:
        static inline void forwardT(const Eigen::VectorXd &tau, Eigen::VectorXd &T);

        template <typename EIGENVEC>
        static inline void backwardT(const Eigen::VectorXd &T, EIGENVEC &tau);

        template <typename EIGENVEC>
        static inline void backwardGradT(const Eigen::VectorXd &tau,
                                         const Eigen::VectorXd &gradT,
                                         EIGENVEC &gradTau);
        static inline void forwardP(const Eigen::VectorXd &xi,
                                    const Eigen::VectorXi &vIdx,
                                    const vector<Eigen::Matrix3Xd> &vPolys,
                                    Eigen::Matrix3Xd &P);
        static inline bool smoothedL1(const double &x,
                                      const double &mu,
                                      double &f,
                                      double &df);
    };

    class HybridOPT : public Optimizer
    {
    public:
        void init(ros::NodeHandle &nh);
        void initGridMap(const GridMap::Ptr &grid_map);
        // void initFlatMap(void);
        bool setup(const vector<Eigen::VectorXd> &waypoints);
        double optimize(utils::Visualizer::Ptr vis);
        static inline void forwardT(const Eigen::VectorXd &tau, Eigen::VectorXd &T) { Optimizer::forwardT(tau, T); }
        template <typename EIGENVEC>
        static inline void backwardT(const Eigen::VectorXd &T, EIGENVEC &tau) { Optimizer::backwardT(T, tau); }
        template <typename EIGENVEC>
        static inline void backwardGradT(const Eigen::VectorXd &tau,
                                         const Eigen::VectorXd &gradT,
                                         EIGENVEC &gradTau) { Optimizer::backwardGradT(tau, gradT, gradTau); }
        static inline bool smoothedL1(const double &x,
                                      const double &mu,
                                      double &f,
                                      double &df) { return Optimizer::smoothedL1(x, mu, f, df); }
        static bool fischer_burmeister(const double &a, const double &b,
                                       double &f, double &dfa, double &dfb);

        typedef shared_ptr<HybridOPT> Ptr;

    private:
        void augmentedLagrangianFunctional(const Eigen::VectorXd &T,
                                           const Eigen::MatrixXd &coeffs,
                                           const int piece_nums,
                                           int &index, double &cost,
                                           Eigen::VectorXd &gradT, Eigen::MatrixXd &gradC);
        void augmentedLagrangianFunctional(const Eigen::VectorXd &T,
                                           const Eigen::MatrixXd &coeffs,
                                           const int piece_nums,
                                           int &index, double &cost,
                                           Eigen::VectorXd &gradT, Eigen::MatrixXd &gradC, Eigen::MatrixXd &grad_cts);
        void updateTrajectories(void);
        void initPoints(const vector<Eigen::VectorXd> &init_path,
                        Eigen::MatrixXd &inner_points);
        void initTimeAllocation(Eigen::MatrixXd &inner_points,
                                Eigen::VectorXd &time_allocation);
        static double dynamicCostFunctional(void *func_data,
                                            const Eigen::VectorXd &xi,
                                            Eigen::VectorXd &grad);
        void dynamicOptimization(void);
        void initScalingW(const Eigen::VectorXd &xi);
        void updateMINCO(void);
        void updateALM(void);
        void updateLambda(void);
        void updateRho(void);
        void check_optimal(void);
        static int innerLoopDebugFunctional(void *func_data,
                                            const Eigen::VectorXd &xi,
                                            const Eigen::VectorXd &grad,
                                            const double f,
                                            const double step,
                                            const int index,
                                            const int err_code);
        static double costFunctional(void *ptr, const Eigen::VectorXd &xi, Eigen::VectorXd &grad_xi);

    private:
        /* optimization modules*/
        GridMap::Ptr grid_map_;

        /* optimization parameters*/
        lbfgs::lbfgs_parameter_t lbfgs_params_, dynamic_opt_lbfgs_params_;

        /* differential flatness mapping*/
        flatness::FlatnessMap flatmap_;

        utils::Visualizer::Ptr visualizer_;

        ros::Publisher debugPub1_, debugPub2_, debugPub3_, debugPub4_;
        ros::Publisher debug_obs_quad_pub_, debug_obs_load_pub_;
        ros::Publisher debug_vel_pub_, debug_acc_pub_, debug_jer_pub_;
        ros::Publisher debug_acc_load_pub_;
        ros::Publisher debug_theta_pub_, debug_omg_pub_, debug_thrust_pub_;
        ros::Publisher debug_dist_pub_, debug_tension_pub_;
        ros::Publisher debug_dyn_pub_, debug_comp_pub_;
        ros::Publisher debug_energy_pub_, debug_time_pub_;
        std_msgs::Float64 data1_, data2_, data3_, data4_;
        std_msgs::Float64 data_obs_quad_, data_obs_load_;
        std_msgs::Float64 data_vel_, data_acc_, data_jer_;
        std_msgs::Float64 data_acc_load_;
        std_msgs::Float64 data_theta_, data_omg_, data_thrust_;
        std_msgs::Float64 data_dist_, data_tension_;
        std_msgs::Float64 data_dyn_, data_comp_;
        std_msgs::Float64 data_energy_, data_time_;

    public:
        poly_traj::Trajectory::Ptr load_traj_, quad_traj_;
        struct DebugParams
        {
            int count = 0;
            ros::Time t1, t2, t3, t4, t5, t6, t7;
            double time1 = 0, time2 = 0, time3 = 0, time4 = 0, time5 = 0, time6 = 0, time7 = 0;
            double init_cost_obs_quad = 0, init_cost_obs_load = 0;
            double init_cost_vel = 0, init_cost_acc = 0, init_cost_jer = 0;
            double init_cost_theta = 0, init_cost_omg = 0, init_cost_thrust = 0;
            double init_cost_dist = 0, init_cost_tension = 0;
            double init_cost_dyn = 0, init_cost_comp = 0;
            double cost_obs_quad = 0, cost_obs_load = 0;
            double cost_vel = 0, cost_acc = 0, cost_jer = 0;
            double cost_acc_load = 0;
            double cost_theta = 0, cost_omg = 0, cost_thrust = 0;
            double cost_dist = 0, cost_tension = 0;
            double cost_dyn = 0, cost_comp = 0;
            double cost_energy = 0, cost_time = 0;
        } debug_params_;
        struct InfoParams
        {
            // debug
            bool debug_flag;
            // minco
            minco::MINCO::Ptr minco;
            // trajectory info
            int traj_order;
            int sample_num;
            // initial guess info
            int pieces_num;
            double init_vel;
            Eigen::Matrix<double, 6, 4> init_state_mat;
            Eigen::Matrix<double, 6, 4> goal_state_mat;
            // deceision variables info
            int state_dim;
            int decision_variable_num;
            Eigen::MatrixXd inner_pts;
            Eigen::VectorXd durations;
            // physical parameters
            double gravity;
            double quad_mass;
            double load_mass;
            double cable_length;
            double safe_margin;
            // debug cost time recording
            double inner_opt_cost;
            double update_info_cost;
            double check_opt_cost;
            double total_opt_cost;
        } info_params_;
        struct OptParams
        {
            // optimization parameters
            int integral_resolution;
            int equality_constraint_num, inequality_constraint_num;
            double cost;
            Eigen::VectorXd xi, grad;
            // ALM parameters
            int k;
            int max_iter;
            double scaling_wf;
            Eigen::VectorXd scaling_wc;
            double scaling_wf_min, scaling_wc_min;
            Eigen::VectorXd cx, lambda;
            Eigen::VectorXi cx_flag;
            double rho, rho_max, gamma;
            double hx_inf, gx_inf, grad_inf;
            double eq_cons_eps, ineq_cons_eps, prec_eps;
            bool stop_flag;
        } opt_params_;
        struct ConstraintParams
        {
            // magnitude bounds
            double max_vel_quad;
            double max_acc_quad;
            double max_jer_quad;
            double max_acc_load;
            double max_theta;
            double max_omg;
            double max_thr, min_thr;
            double max_tension;
            double min_tension;
            double min_distance;
            // penalty weights
            double weight_t;
            double weight_energy;
            double weight_obs;
            double weight_pos;
            double weight_vel_quad;
            double weight_acc_quad;
            double weight_jer_quad;
            double weight_acc_load;
            double weight_theta;
            double weight_omg;
            double weight_thr;
            double weight_ten;
            double weight_dis;
            double weight_dyn;
            double weight_com;
            int flag_obs;
            int flag_pos;
            int flag_vel_quad;
            int flag_acc_quad;
            int flag_jer_quad;
            int flag_acc_load;
            int flag_theta;
            int flag_omg;
            int flag_thr;
            int flag_ten;
            int flag_dis;
            int flag_dyn;
            int flag_com;
        } cst_params_;

    private:
        inline void inequalityConstraintObs(const double &wc, const double &mu_i, const double &rho,
                                            const double &safe_margin,
                                            const Eigen::Vector3d &pos,
                                            double &constraint_violat, Eigen::Vector3d &c_partial_pos_,
                                            double &augmented_term, Eigen::Vector3d &augmented_term_partial_pos)
        {
            double violat_(0.0);
            c_partial_pos_.setZero();

            // esdf params
            double map_resolution_ = grid_map_->getResolution();
            double esdf_dist_ = grid_map_->getDistance(pos);
            Eigen::Vector3d esdf_dist_grad_(0.0, 0.0, 0.0);

            // reset the output value
            augmented_term = 0.0;
            augmented_term_partial_pos.setZero();

            // compute the violation of collision constraint
            violat_ = safe_margin + map_resolution_ - esdf_dist_;
            if (violat_ > 0)
            {
                grid_map_->evaluateEDTWithGrad(pos, esdf_dist_, esdf_dist_grad_);
                violat_ = safe_margin - esdf_dist_;
            }

            constraint_violat = violat_;

            if ((mu_i + rho * wc * violat_) > 0)
            {
                c_partial_pos_ = (-1) * esdf_dist_grad_;
                augmented_term = wc * violat_ * (mu_i + 0.5 * rho * wc * violat_);
                augmented_term_partial_pos = (mu_i + rho * wc * violat_) * wc * c_partial_pos_;
            }
            else
            {
                augmented_term = -0.5 * mu_i * mu_i / rho;
                augmented_term_partial_pos.setZero();
            }
        };
        inline void inequalityConstraintVel(const double &wc, const double &mu_i, const double &rho,
                                            const double &sqr_bound,
                                            const Eigen::Vector3d &vel,
                                            double &constraint_violat, Eigen::Vector3d &c_partial_vel_,
                                            double &augmented_term, Eigen::Vector3d &augmented_term_partial_vel)
        {
            double violat_ = 0.0;
            double smt_violat_ = 0.0;
            double smt_violat_grad_ = 0.0;
            c_partial_vel_.setZero();

            augmented_term = 0.0;
            augmented_term_partial_vel.setZero();

            violat_ = vel.squaredNorm() - sqr_bound;
            smoothedL1(violat_, 0.001, smt_violat_, smt_violat_grad_);
            smt_violat_ = smt_violat_;
            smt_violat_grad_ = smt_violat_grad_ * 1.0;

            constraint_violat = smt_violat_;

            if ((mu_i + rho * wc * smt_violat_) > 0)
            {
                c_partial_vel_ = smt_violat_grad_ * 2.0 * vel;
                augmented_term = wc * smt_violat_ * (mu_i + 0.5 * rho * wc * smt_violat_);
                augmented_term_partial_vel = (mu_i + rho * wc * smt_violat_) * wc * c_partial_vel_;
            }
            else
            {
                augmented_term = -0.5 * mu_i * mu_i / rho;
                augmented_term_partial_vel.setZero();
            }
        };

        inline void inequalityConstraintAcc(const double &wc, const double &mu_i, const double &rho,
                                            const double &sqr_bound,
                                            const Eigen::Vector3d &acc,
                                            double &constraint_violat, Eigen::Vector3d &c_partial_acc_,
                                            double &augmented_term, Eigen::Vector3d &augmented_term_partial_acc)
        {
            double violat_ = 0.0;
            double smt_violat_ = 0.0;
            double smt_violat_grad_ = 0.0;
            c_partial_acc_.setZero();

            augmented_term = 0.0;
            augmented_term_partial_acc.setZero();

            violat_ = acc.squaredNorm() - sqr_bound;
            smoothedL1(violat_, 0.001, smt_violat_, smt_violat_grad_);
            smt_violat_ = smt_violat_;
            smt_violat_grad_ = smt_violat_grad_ * 1.0;

            constraint_violat = smt_violat_;

            if ((mu_i + rho * wc * smt_violat_) > 0)
            {
                c_partial_acc_ = smt_violat_grad_ * 2.0 * acc;
                augmented_term = wc * smt_violat_ * (mu_i + 0.5 * rho * wc * smt_violat_);
                augmented_term_partial_acc = (mu_i + rho * wc * smt_violat_) * wc * c_partial_acc_;
            }
            else
            {
                augmented_term = -0.5 * mu_i * mu_i / rho;
                augmented_term_partial_acc.setZero();
            }
        };
        inline void inequalityConstraintJer(const double &wc, const double &mu_i, const double &rho,
                                            const double &sqr_bound,
                                            const Eigen::Vector3d &jer,
                                            double &constraint_violat, Eigen::Vector3d &c_partial_jer_,
                                            double &augmented_term, Eigen::Vector3d &augmented_term_partial_jer)
        {
            double violat_ = 0.0;
            double smt_violat_ = 0.0;
            double smt_violat_grad_ = 0.0;
            c_partial_jer_.setZero();

            augmented_term = 0.0;
            augmented_term_partial_jer.setZero();

            violat_ = jer.squaredNorm() - sqr_bound;
            smoothedL1(violat_, 0.001, smt_violat_, smt_violat_grad_);

            constraint_violat = smt_violat_;

            if ((mu_i + rho * wc * smt_violat_) > 0)
            {
                c_partial_jer_ = smt_violat_grad_ * 2.0 * jer;
                augmented_term = wc * smt_violat_ * (mu_i + 0.5 * rho * wc * smt_violat_);
                augmented_term_partial_jer = (mu_i + rho * wc * smt_violat_) * wc * c_partial_jer_;
            }
            else
            {
                augmented_term = -0.5 * mu_i * mu_i / rho;
                augmented_term_partial_jer.setZero();
            }
        };
        inline void inequalityConstraintTheta(const double &wc, const double &mu_i, const double &rho,
                                              const double &bound,
                                              const Eigen::Vector4d &quat,
                                              double &constraint_violat,
                                              Eigen::Vector3d &c_partial_load_acc_, Eigen::Vector3d &c_partial_load_jer_,
                                              Eigen::Vector3d &c_partial_quad_acc_, Eigen::Vector3d &c_partial_quad_jer_,
                                              double &augmented_term,
                                              Eigen::Vector3d &augmented_term_partial_load_acc, Eigen::Vector3d &augmented_term_partial_load_jer,
                                              Eigen::Vector3d &augmented_term_partial_quad_acc, Eigen::Vector3d &augmented_term_partial_quad_jer)
        {
            double violat_ = 0.0;
            double smt_violat_ = 0.0;
            double smt_violat_grad_ = 0.0;
            double cos_theta_ = 1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2));
            Eigen::Vector4d quat_d_(0.0, 0.0, 0.0, 0.0);
            Eigen::Vector3d z_d_(0.0, 0.0, 0.0);
            double psi_grad_ = 0.0, dpsi_grad_ = 0.0;
            c_partial_load_acc_.setZero();
            c_partial_load_jer_.setZero();
            c_partial_quad_acc_.setZero();
            c_partial_quad_jer_.setZero();

            augmented_term = 0.0;
            augmented_term_partial_load_acc.setZero();
            augmented_term_partial_load_jer.setZero();
            augmented_term_partial_quad_acc.setZero();
            augmented_term_partial_quad_jer.setZero();

            violat_ = acos(cos_theta_) - bound;
            smoothedL1(violat_, 0.001, smt_violat_, smt_violat_grad_);

            constraint_violat = smt_violat_;

            if ((mu_i + rho * wc * smt_violat_) > 0)
            {
                quat_d_ = smt_violat_grad_ / sqrt(1.0 - cos_theta_ * cos_theta_) * 4.0 * Eigen::Vector4d(0.0, quat(1), quat(2), 0.0);
                // z_d_ = quat_d_.transpose() * flatmap_.quat_partial_z;
                // c_partial_load_acc_ = z_d_.transpose() * flatmap_.z_partial_load_acc;
                // c_partial_load_jer_.setZero();
                // c_partial_quad_acc_ = z_d_.transpose() * flatmap_.z_partial_quad_acc;
                // c_partial_quad_jer_.setZero();
                flatmap_.backward(0.0, quat_d_, Eigen::Vector3d(0.0, 0.0, 0.0),
                                  c_partial_load_acc_, c_partial_load_jer_, c_partial_quad_acc_, c_partial_quad_jer_,
                                  psi_grad_, dpsi_grad_);
                augmented_term = wc * smt_violat_ * (mu_i + 0.5 * rho * wc * smt_violat_);
                augmented_term_partial_load_acc = (mu_i + rho * wc * smt_violat_) * wc * c_partial_load_acc_;
                augmented_term_partial_load_jer = (mu_i + rho * wc * smt_violat_) * wc * c_partial_load_jer_;
                augmented_term_partial_quad_acc = (mu_i + rho * wc * smt_violat_) * wc * c_partial_quad_acc_;
                augmented_term_partial_quad_jer = (mu_i + rho * wc * smt_violat_) * wc * c_partial_quad_jer_;
            }
            else
            {
                augmented_term = -0.5 * mu_i * mu_i / rho;
                augmented_term_partial_load_acc.setZero();
                augmented_term_partial_load_jer.setZero();
                augmented_term_partial_quad_acc.setZero();
                augmented_term_partial_quad_jer.setZero();
            }
        };
        inline void inequalityConstraintThrust(const double &wc, const double &mu_i, const double &rho,
                                               const double &mean, const double &radi,
                                               const double &thrust,
                                               double &constraint_violat,
                                               Eigen::Vector3d &c_partial_load_acc_, Eigen::Vector3d &c_partial_load_jer_,
                                               Eigen::Vector3d &c_partial_quad_acc_, Eigen::Vector3d &c_partial_quad_jer_,
                                               double &augmented_term,
                                               Eigen::Vector3d &augmented_term_partial_load_acc, Eigen::Vector3d &augmented_term_partial_load_jer,
                                               Eigen::Vector3d &augmented_term_partial_quad_acc, Eigen::Vector3d &augmented_term_partial_quad_jer)
        {
            double violat_ = 0.0;
            double smt_violat_ = 0.0;
            double smt_violat_grad_ = 0.0;
            double thr_grad_;
            double psi_grad_ = 0.0, dpsi_grad_ = 0.0;
            c_partial_load_acc_.setZero();
            c_partial_load_jer_.setZero();
            c_partial_quad_acc_.setZero();
            c_partial_quad_jer_.setZero();

            augmented_term = 0.0;
            augmented_term_partial_load_acc.setZero();
            augmented_term_partial_load_jer.setZero();
            augmented_term_partial_quad_acc.setZero();
            augmented_term_partial_quad_jer.setZero();

            violat_ = (thrust - mean) * (thrust - mean) - radi * radi;
            smoothedL1(violat_, 0.001, smt_violat_, smt_violat_grad_);

            constraint_violat = smt_violat_;

            if ((mu_i + rho * wc * smt_violat_) > 0)
            {
                thr_grad_ = smt_violat_grad_ * 2.0 * (thrust - mean);
                flatmap_.backward(thr_grad_, Eigen::Vector4d(0.0, 0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0),
                                  c_partial_load_acc_, c_partial_load_jer_, c_partial_quad_acc_, c_partial_quad_jer_,
                                  psi_grad_, dpsi_grad_);
                augmented_term = wc * smt_violat_ * (mu_i + 0.5 * rho * wc * smt_violat_);
                augmented_term_partial_load_acc = (mu_i + rho * wc * smt_violat_) * wc * c_partial_load_acc_;
                augmented_term_partial_load_jer = (mu_i + rho * wc * smt_violat_) * wc * c_partial_load_jer_;
                augmented_term_partial_quad_acc = (mu_i + rho * wc * smt_violat_) * wc * c_partial_quad_acc_;
                augmented_term_partial_quad_jer = (mu_i + rho * wc * smt_violat_) * wc * c_partial_quad_jer_;
            }
            else
            {
                augmented_term = -0.5 * mu_i * mu_i / rho;
                augmented_term_partial_load_acc.setZero();
                augmented_term_partial_load_jer.setZero();
                augmented_term_partial_quad_acc.setZero();
                augmented_term_partial_quad_jer.setZero();
            }
        };
        inline void inequalityConstraintOmg(const double &wc, const double &mu_i, const double &rho,
                                            const double &sqr_bound,
                                            const Eigen::Vector3d &omg,
                                            double &constraint_violat,
                                            Eigen::Vector3d &c_partial_load_acc_, Eigen::Vector3d &c_partial_load_jer_,
                                            Eigen::Vector3d &c_partial_quad_acc_, Eigen::Vector3d &c_partial_quad_jer_,
                                            double &augmented_term,
                                            Eigen::Vector3d &augmented_term_partial_load_acc, Eigen::Vector3d &augmented_term_partial_load_jer,
                                            Eigen::Vector3d &augmented_term_partial_quad_acc, Eigen::Vector3d &augmented_term_partial_quad_jer)
        {
            double violat_ = 0.0;
            double smt_violat_ = 0.0;
            double smt_violat_grad_ = 0.0;
            Eigen::Vector3d omg_grad_;
            double psi_grad_ = 0.0, dpsi_grad_ = 0.0;
            c_partial_load_acc_.setZero();
            c_partial_load_jer_.setZero();
            c_partial_quad_acc_.setZero();
            c_partial_quad_jer_.setZero();

            augmented_term = 0.0;
            augmented_term_partial_load_acc.setZero();
            augmented_term_partial_load_jer.setZero();
            augmented_term_partial_quad_acc.setZero();
            augmented_term_partial_quad_jer.setZero();

            violat_ = omg.squaredNorm() - sqr_bound;
            smoothedL1(violat_, 0.001, smt_violat_, smt_violat_grad_);

            constraint_violat = smt_violat_;

            if ((mu_i + rho * wc * smt_violat_) > 0)
            {
                omg_grad_ = smt_violat_grad_ * 2.0 * omg;
                flatmap_.backward(0.0, Eigen::Vector4d(0.0, 0.0, 0.0, 0.0), omg_grad_,
                                  c_partial_load_acc_, c_partial_load_jer_, c_partial_quad_acc_, c_partial_quad_jer_,
                                  psi_grad_, dpsi_grad_);
                augmented_term = wc * smt_violat_ * (mu_i + 0.5 * rho * wc * smt_violat_);
                augmented_term_partial_load_acc = (mu_i + rho * wc * smt_violat_) * wc * c_partial_load_acc_;
                augmented_term_partial_load_jer = (mu_i + rho * wc * smt_violat_) * wc * c_partial_load_jer_;
                augmented_term_partial_quad_acc = (mu_i + rho * wc * smt_violat_) * wc * c_partial_quad_acc_;
                augmented_term_partial_quad_jer = (mu_i + rho * wc * smt_violat_) * wc * c_partial_quad_jer_;
            }
            else
            {
                augmented_term = -0.5 * mu_i * mu_i / rho;
                augmented_term_partial_load_acc.setZero();
                augmented_term_partial_load_jer.setZero();
                augmented_term_partial_quad_acc.setZero();
                augmented_term_partial_quad_jer.setZero();
            }
        };
        inline void inequalityConstraintTen(const double &wc, const double &mu_i, const double &rho,
                                            const double &mean, const double &radi,
                                            const Eigen::Vector3d &load_acc,
                                            double &constraint_violat, Eigen::Vector3d &c_partial_load_acc_,
                                            double &augmented_term, Eigen::Vector3d &augmented_term_partial_load_acc)
        {
            double violat_ = 0.0;
            double smt_violat_ = 0.0;
            double smt_violat_grad_ = 0.0;
            Eigen::Vector3d gravity_(0.0, 0.0, info_params_.gravity);
            c_partial_load_acc_.setZero();

            augmented_term = 0.0;
            augmented_term_partial_load_acc.setZero();

            violat_ = ((info_params_.load_mass * (load_acc + gravity_).norm() - mean) *
                           (info_params_.load_mass * (load_acc + gravity_).norm() - mean) -
                       radi * radi);
            smt_violat_ = violat_;
            smt_violat_grad_ = 1.0;

            constraint_violat = smt_violat_;

            if ((mu_i + rho * wc * smt_violat_) > 0)
            {
                c_partial_load_acc_ = smt_violat_grad_ * (2.0) * (info_params_.load_mass * (load_acc + gravity_).norm() - mean) * info_params_.load_mass / (load_acc + gravity_).norm() * (load_acc + gravity_);
                augmented_term = wc * smt_violat_ * (mu_i + 0.5 * rho * wc * smt_violat_);
                augmented_term_partial_load_acc = (mu_i + rho * wc * smt_violat_) * wc * c_partial_load_acc_;
            }
            else
            {
                augmented_term = -0.5 * mu_i * mu_i / rho;
            }
        };
        inline void inequalityConstraintDis(const double &wc, const double &mu_i, const double &rho,
                                            const double &mean, const double &radi,
                                            const Eigen::Vector3d &load_pos, const Eigen::Vector3d &quad_pos,
                                            double &constraint_violat, Eigen::Vector3d &c_partial_load_pos_, Eigen::Vector3d &c_partial_quad_pos_,
                                            double &augmented_term,
                                            Eigen::Vector3d &augmented_term_partial_load_pos, Eigen::Vector3d &augmented_term_partial_quad_pos)
        {
            double violat_ = 0.0;
            double smt_violat_ = 0.0;
            double smt_violat_grad_ = 0.0;
            Eigen::Vector3d dist_(0.0, 0.0, 0.0);
            c_partial_quad_pos_.setZero();
            c_partial_load_pos_.setZero();

            augmented_term = 0.0;
            augmented_term_partial_load_pos.setZero();
            augmented_term_partial_quad_pos.setZero();
            dist_ = load_pos - quad_pos;

            violat_ = (dist_.norm() - mean) * (dist_.norm() - mean) - radi * radi;
            // violat_ = dist_.squaredNorm() - pow((mean + radi), 2);
            smoothedL1(violat_, 0.001, smt_violat_, smt_violat_grad_);
            // smt_violat_ = violat_;
            // smt_violat_grad_ = 1.0;

            constraint_violat = smt_violat_;

            if ((mu_i + rho * wc * smt_violat_) > 0)
            {
                // std::cout << smt_violat_ << std::endl;
                c_partial_load_pos_ = smt_violat_grad_ * (2.0) * (dist_.norm() - mean) / dist_.norm() * dist_;
                c_partial_quad_pos_ = smt_violat_grad_ * (-2.0) * (dist_.norm() - mean) / dist_.norm() * dist_;
                augmented_term = wc * smt_violat_ * (mu_i + 0.5 * rho * wc * smt_violat_);
                augmented_term_partial_load_pos = (mu_i + rho * wc * smt_violat_) * wc * c_partial_load_pos_;
                augmented_term_partial_quad_pos = (mu_i + rho * wc * smt_violat_) * wc * c_partial_quad_pos_;
            }
            else
            {
                augmented_term = -0.5 * mu_i * mu_i / rho;
                augmented_term_partial_load_pos.setZero();
                augmented_term_partial_quad_pos.setZero();
            }
        };
        inline void equalityConstraintDyn(const double &wc, const double &lambda_i, const double &rho,
                                          const Eigen::Vector3d &load_pos, const Eigen::Vector3d &load_acc, const Eigen::Vector3d &quad_pos,
                                          double &constraint_violat,
                                          Eigen::Vector3d &c_partial_load_pos_, Eigen::Vector3d &c_partial_load_acc_, Eigen::Vector3d &c_partial_quad_pos_,
                                          double &augmented_term,
                                          Eigen::Vector3d &augmented_term_partial_load_pos,
                                          Eigen::Vector3d &augmented_term_partial_load_acc,
                                          Eigen::Vector3d &augmented_term_partial_quad_pos)
        {
            double violat_ = 0.0;
            double smt_violat_ = 0.0;
            double smt_violat_grad_ = 0.0;

            c_partial_quad_pos_.setZero();
            c_partial_load_pos_.setZero();
            c_partial_load_acc_.setZero();
            Eigen::Vector3d gravity_(0.0, 0.0, info_params_.gravity);
            Eigen::Vector3d e_(1e-12, 1e-12, 1e-12);

            augmented_term = 0.0;
            augmented_term_partial_load_pos.setZero();
            augmented_term_partial_load_acc.setZero();
            augmented_term_partial_quad_pos.setZero();

            violat_ = ((load_acc + gravity_ + e_).norm() * (load_pos - quad_pos) + (load_pos - quad_pos + e_).norm() * (load_acc + gravity_)).squaredNorm();
            smt_violat_ = violat_;
            smt_violat_grad_ = 1.0;

            constraint_violat = smt_violat_;
            augmented_term = wc * smt_violat_ * (lambda_i + 0.5 * rho * wc * smt_violat_);

            // partial derivative of load_acc
            double t00_ = 0.0;
            Eigen::Vector3d t01_(0.0, 0.0, 0.0);
            double t02_ = 0.0;
            Eigen::Vector3d t03_(0.0, 0.0, 0.0);
            Eigen::Vector3d t04_(0.0, 0.0, 0.0);

            t00_ = (load_pos - quad_pos + e_).norm();
            t01_ = load_acc + gravity_ + e_;
            t02_ = t01_.norm();
            t03_ = load_pos - quad_pos;
            t04_ = t00_ * (load_acc + gravity_) + t02_ * t03_;

            c_partial_load_acc_ = smt_violat_grad_ * (2.0 * t00_ * t04_ + 2.0 / t02_ * t03_.transpose() * t04_ * t01_);
            augmented_term_partial_load_acc = (lambda_i + rho * wc * smt_violat_) * wc * c_partial_load_acc_;

            // partial derivative of quad_pos
            Eigen::Vector3d t10_(0.0, 0.0, 0.0);
            double t11_ = 0.0;
            Eigen::Vector3d t12_(0.0, 0.0, 0.0);
            double t13_ = 0.0;
            Eigen::Vector3d t14_(0.0, 0.0, 0.0);

            t10_ = load_pos - quad_pos + e_;
            t11_ = t10_.norm();
            t12_ = load_acc + gravity_;
            t13_ = (load_acc + gravity_ + e_).norm();
            t14_ = t11_ * t12_ + t13_ * (load_pos - quad_pos);

            c_partial_quad_pos_ = smt_violat_grad_ * (-(2.0 / t11_ * t12_.transpose() * t14_ * t10_ + 2.0 * t13_ * t14_));
            augmented_term_partial_quad_pos = (lambda_i + rho * wc * smt_violat_) * wc * c_partial_quad_pos_;

            // partial derivative of load_pos
            Eigen::Vector3d t20_(0.0, 0.0, 0.0);
            double t21_ = 0.0;
            Eigen::Vector3d t22_(0.0, 0.0, 0.0);
            double t23_ = 0.0;
            Eigen::Vector3d t24_(0.0, 0.0, 0.0);

            t20_ = load_pos - quad_pos + e_;
            t21_ = t20_.norm();
            t22_ = load_acc + gravity_;
            t23_ = (load_acc + gravity_ + e_).norm();
            t24_ = t21_ * t22_ + t23_ * (load_pos - quad_pos);

            c_partial_load_pos_ = smt_violat_grad_ * (2.0 / t21_ * t22_.transpose() * t24_ * t20_ + 2.0 * t23_ * t24_);
            augmented_term_partial_load_pos = (lambda_i + rho * wc * smt_violat_) * wc * c_partial_load_pos_;
        };
        // inline void equalityConstraintCom(const double &wc, const double &lambda_i, const double &rho,
        //                                   const Eigen::Vector3d &load_pos, const Eigen::Vector3d &load_acc, const Eigen::Vector3d &quad_pos,
        //                                   double &constraint_violat,
        //                                   Eigen::Vector3d &c_partial_load_pos_, Eigen::Vector3d &c_partial_load_acc_, Eigen::Vector3d &c_partial_quad_pos_,
        //                                   double &augmented_term,
        //                                   Eigen::Vector3d &augmented_term_partial_load_pos,
        //                                   Eigen::Vector3d &augmented_term_partial_load_acc,
        //                                   Eigen::Vector3d &augmented_term_partial_quad_pos)
        // {
        //     double violat_(0.0);
        //     double smt_violat_(0.0);
        //     double smt_violat_grad_(0.0);
        //     c_partial_quad_pos_.setZero();
        //     c_partial_load_pos_.setZero();
        //     c_partial_load_acc_.setZero();
        //     Eigen::Vector3d gravity_(0.0, 0.0, info_params_.gravity);
        //     Eigen::Vector3d e_(0.0001, 0.0001, 0.0001);

        //     augmented_term = 0.0;
        //     augmented_term_partial_load_pos.setZero();
        //     augmented_term_partial_load_acc.setZero();
        //     augmented_term_partial_quad_pos.setZero();

        //     violat_ = ((info_params_.cable_length - (load_pos - quad_pos + e_).norm()) +                                                                                             // (l0 - l) +
        //             info_params_.load_mass * (load_acc + gravity_ + e_).norm() -                                                                                                  // (a + g) -
        //             sqrt(pow((info_params_.cable_length - (load_pos - quad_pos + e_).norm()), 2) + pow(info_params_.load_mass * (load_acc + gravity_ + e_).norm(), 2) + 0.0001)); // sqrt((l0 - l)^2 + (a + g)^2)
        //     smoothedL1(violat_, 0.001, smt_violat_, smt_violat_grad_);
        //     // smt_violat_ = violat_;
        //     // smt_violat_grad_ = 1.0;

        //     constraint_violat = smt_violat_;
        //     augmented_term = wc * smt_violat_ * (lambda_i + 0.5 * rho * wc * smt_violat_);

        //     // partial derivative of load position
        //     Eigen::Vector3d t00_(0.0, 0.0, 0.0);
        //     double t01_(0.0);
        //     double t02_(0.0);

        //     t00_ = load_pos - quad_pos + e_;
        //     t01_ = t00_.norm();
        //     t02_ = info_params_.cable_length - t01_;

        //     c_partial_load_pos_ = smt_violat_grad_ * (t02_ / (sqrt(0.0001 + t02_ * t02_ + info_params_.load_mass * info_params_.load_mass * (load_acc + gravity_).squaredNorm()) * t01_) * t00_ - 1.0 / t01_ * t00_);
        //     augmented_term_partial_load_pos = (lambda_i + rho * wc * smt_violat_) * wc * c_partial_load_pos_;

        //     // partial derivative of quad position
        //     Eigen::Vector3d t10_(0.0, 0.0, 0.0);
        //     double t11_(0.0);
        //     double t12_(0.0);

        //     t10_ = load_pos - quad_pos + e_;
        //     t11_ = t10_.norm();
        //     t12_ = info_params_.cable_length - t11_;

        //     c_partial_quad_pos_ = smt_violat_grad_ * (1.0 / t11_ * t10_ - t12_ / (sqrt(0.0001 + t12_ * t12_ + info_params_.load_mass * info_params_.load_mass * (load_acc + gravity_).squaredNorm()) * t11_) * t10_);
        //     augmented_term_partial_quad_pos = (lambda_i + rho * wc * smt_violat_) * wc * c_partial_quad_pos_;

        //     // partial derivative of load acceleration
        //     Eigen::Vector3d t20_(0.0, 0.0, 0.0);
        //     double t21_(0.0);
        //     double t22_(0.0);

        //     t20_ = load_acc + gravity_ + e_;
        //     t21_ = info_params_.load_mass * info_params_.load_mass;
        //     t22_ = t20_.norm();

        //     c_partial_load_acc_ = smt_violat_grad_ * (info_params_.load_mass / t22_ * t20_ - t21_ / sqrt(0.0001 + pow(info_params_.cable_length - (load_pos - quad_pos).norm(), 2) + t21_ * t22_ * t22_) * t20_);
        //     augmented_term_partial_load_acc = (lambda_i + rho * wc * smt_violat_) * wc * c_partial_load_acc_;
        // };
        inline void equalityConstraintCom(const double &wc, const double &lambda_i, const double &rho,
                                          const Eigen::Vector3d &load_pos, const Eigen::Vector3d &load_acc, const Eigen::Vector3d &quad_pos,
                                          double &constraint_violat,
                                          Eigen::Vector3d &c_partial_load_pos_, Eigen::Vector3d &c_partial_load_acc_, Eigen::Vector3d &c_partial_quad_pos_,
                                          double &augmented_term,
                                          Eigen::Vector3d &augmented_term_partial_load_pos,
                                          Eigen::Vector3d &augmented_term_partial_load_acc,
                                          Eigen::Vector3d &augmented_term_partial_quad_pos)
        {
            double violat_ = 0.0;
            double smt_violat_ = 0.0;
            double smt_violat_grad_ = 0.0;
            c_partial_quad_pos_.setZero();
            c_partial_load_pos_.setZero();
            c_partial_load_acc_.setZero();
            Eigen::Vector3d gravity_(0.0, 0.0, info_params_.gravity);
            Eigen::Vector3d e_(0.0001, 0.0001, 0.0001);

            augmented_term = 0.0;
            augmented_term_partial_load_pos.setZero();
            augmented_term_partial_load_acc.setZero();
            augmented_term_partial_quad_pos.setZero();

            violat_ = ((info_params_.cable_length - (load_pos - quad_pos + e_).norm()) +                                                                                             // (l0 - l) +
                       info_params_.load_mass * (load_acc + gravity_ + e_).norm() -                                                                                                  // (a + g) -
                       sqrt(pow((info_params_.cable_length - (load_pos - quad_pos + e_).norm()), 2) + pow(info_params_.load_mass * (load_acc + gravity_ + e_).norm(), 2) + 0.0001)); // sqrt((l0 - l)^2 + (a + g)^2)
            smt_violat_ = pow(violat_, 2);
            smt_violat_grad_ = 1.0;

            constraint_violat = smt_violat_;
            augmented_term = wc * smt_violat_ * (lambda_i + 0.5 * rho * wc * smt_violat_);

            // partial derivative of load position
            Eigen::Vector3d t00_(0.0, 0.0, 0.0);
            double t01_ = 0.0;
            double t02_ = 0.0;
            double t03_ = 0.0;
            double t04_ = 0.0;
            double t05_ = 0.0;

            t00_ = load_pos - quad_pos + e_;
            t01_ = t00_.norm();
            t02_ = info_params_.cable_length - t01_;
            t03_ = (load_acc + gravity_ + e_).norm();
            t04_ = sqrt(pow(t02_, 2) + pow(info_params_.load_mass * t03_, 2) + 0.0001);
            t05_ = t02_ + info_params_.load_mass * t03_ - t04_;

            c_partial_load_pos_ = smt_violat_grad_ * ((2.0 * t02_ * t05_) / (t04_ * t01_) * t00_ - (2.0 * t05_) / t01_ * t00_);
            augmented_term_partial_load_pos = (lambda_i + rho * wc * smt_violat_) * wc * c_partial_load_pos_;

            // partial derivative of quad position
            Eigen::Vector3d t10_(0.0, 0.0, 0.0);
            double t11_ = 0.0;
            double t12_ = 0.0;
            double t13_ = 0.0;
            double t14_ = 0.0;
            double t15_ = 0.0;

            t10_ = load_pos - quad_pos + e_;
            t11_ = t10_.norm();
            t12_ = info_params_.cable_length - t11_;
            t13_ = (load_acc + gravity_ + e_).norm();
            t14_ = sqrt(pow(t12_, 2) + pow(info_params_.load_mass * t13_, 2) + 0.0001);
            t15_ = t12_ + info_params_.load_mass * t13_ - t14_;

            c_partial_quad_pos_ = smt_violat_grad_ * ((2.0 * t15_) / t11_ * t10_ - (2.0 * t12_ * t15_) / (t14_ * t11_) * t10_);
            augmented_term_partial_quad_pos = (lambda_i + rho * wc * smt_violat_) * wc * c_partial_quad_pos_;

            // partial derivative of load acceleration
            double t20_ = 0.0;
            Eigen::Vector3d t21_(0.0, 0.0, 0.0);
            double t22_ = 0.0;
            double t23_ = 0.0;
            double t24_ = 0.0;
            double t25_ = 0.0;

            t20_ = info_params_.cable_length - (load_pos - quad_pos + e_).norm();
            t21_ = load_acc + gravity_ + e_;
            t22_ = t21_.norm();
            t23_ = pow(info_params_.load_mass, 2);
            t24_ = sqrt(pow(t20_, 2) + t23_ * pow(t22_, 2) + 0.0001);
            t25_ = t20_ + info_params_.load_mass * t22_ - t24_;

            c_partial_load_acc_ = smt_violat_grad_ * (2.0 * info_params_.load_mass * t25_ / t22_ * t21_ - (2.0 * t23_ * t25_) / t24_ * t21_);
            augmented_term_partial_load_acc = (lambda_i + rho * wc * smt_violat_) * wc * c_partial_load_acc_;
        };
        inline void get_grad_CT(const double &node, const double &alpha, const double &step,
                                const int index, const int traj_order,
                                const Eigen::Matrix<double, 8, 1> &beta0,
                                const Eigen::Matrix<double, 8, 1> &beta1,
                                const Eigen::Matrix<double, 8, 1> &beta2,
                                const Eigen::Matrix<double, 8, 1> &beta3,
                                const Eigen::Vector3d &load_vel, const Eigen::Vector3d &load_acc, const Eigen::Vector3d &load_jer, const Eigen::Vector3d &load_sna,
                                const Eigen::Vector3d &quad_vel, const Eigen::Vector3d &quad_acc, const Eigen::Vector3d &quad_jer, const Eigen::Vector3d &quad_sna,
                                const Eigen::VectorXd &total_grad_load_pos, const Eigen::VectorXd &total_grad_load_vel,
                                const Eigen::VectorXd &total_grad_load_acc, const Eigen::VectorXd &total_grad_load_jer,
                                const Eigen::VectorXd &total_grad_quad_pos, const Eigen::VectorXd &total_grad_quad_vel,
                                const Eigen::VectorXd &total_grad_quad_acc, const Eigen::VectorXd &total_grad_quad_jer,
                                Eigen::VectorXd &gradT, Eigen::MatrixXd &gradC)
        {
            gradC.block(index * (traj_order + 1), 0, traj_order + 1, 3) += (beta0 * total_grad_load_pos.transpose() +
                                                                            beta1 * total_grad_load_vel.transpose() +
                                                                            beta2 * total_grad_load_acc.transpose() +
                                                                            beta3 * total_grad_load_jer.transpose()) *
                                                                           node * step;
            gradC.block(index * (traj_order + 1), 3, traj_order + 1, 3) += (beta0 * total_grad_quad_pos.transpose() +
                                                                            beta1 * total_grad_quad_vel.transpose() +
                                                                            beta2 * total_grad_quad_acc.transpose() +
                                                                            beta3 * total_grad_quad_jer.transpose()) *
                                                                           node * step;
            gradT(index) += (total_grad_load_pos.dot(load_vel) +
                             total_grad_load_vel.dot(load_acc) +
                             total_grad_load_acc.dot(load_jer) +
                             total_grad_load_jer.dot(load_sna) +
                             total_grad_quad_pos.dot(quad_vel) +
                             total_grad_quad_vel.dot(quad_acc) +
                             total_grad_quad_acc.dot(quad_jer) +
                             total_grad_quad_jer.dot(quad_sna)) *
                            alpha * node * step;
        };
    };
}

#endif
