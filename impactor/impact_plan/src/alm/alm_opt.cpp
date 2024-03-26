/************************************************************************
 * Date:        2023.02
 * Author:      Haokun Wang <hwangeh at connect dot ust dot hk>, 
                Aerial Robotics Group <https://uav.ust.hk>, HKUST.
 * E-mail:      hwangeh_at_connect_dot_ust_dot_hk
 * Description: This is the source file for the ALM optimization class,
 *              which is used to solve the trajectory optimization problem
 *              using the Augmented Lagrangian Method.
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

#include "alm/alm_opt.hpp"

namespace alm_opt {
void Optimizer::forwardT(const Eigen::VectorXd& tau, Eigen::VectorXd& T) {
	T.resize(tau.size());
	T.setZero();
	for (int i = 0; i < tau.size(); i++) {
		T(i) = tau(i) > 0.0 ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0)
		                    : 1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
	}
	return;
}

template <typename EIGENVEC>
void Optimizer::backwardT(const Eigen::VectorXd& T, EIGENVEC& tau) {
	tau.resize(T.size());
	for (int i = 0; i < T.size(); i++) {
		tau(i) = T(i) > 1.0 ? (sqrt(2.0 * T(i) - 1.0) - 1.0)
		                    : (1.0 - sqrt(2.0 / T(i) - 1.0));
	}
	return;
}

template <typename EIGENVEC>
void Optimizer::backwardGradT(const Eigen::VectorXd& tau,
                              const Eigen::VectorXd& gradT, EIGENVEC& gradTau) {
	gradTau.resize(tau.size());
	double denSqrt;
	for (int i = 0; i < tau.size(); i++) {
		if (tau(i) > 0) {
			gradTau(i) = gradT(i) * (tau(i) + 1.0);
		} else {
			denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
			gradTau(i) = gradT(i) * (1.0 - tau(i)) / (denSqrt * denSqrt);
		}
	}
	return;
}

bool Optimizer::smoothedL1(const double& x, const double& mu, double& f,
                           double& df) {
	if (x < 0.0) {
		f = x;
		df = 1.0;
		return false;
	} else if (x > mu) {
		f = x - 0.5 * mu;
		df = 1.0;
		return true;
	} else {
		const double xdmu = x / mu;
		const double sqrxdmu = xdmu * xdmu;
		const double mumxd2 = mu - 0.5 * x;
		f = mumxd2 * sqrxdmu * xdmu;
		df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
		return true;
	}
}

void HybridOPT::init(ros::NodeHandle& nh) {
	nh.param("alm/debug_flag", info_params_.debug_flag, false);
	if (info_params_.debug_flag) {
		debugPub1_ =
		    nh.advertise<std_msgs::Float64>("/debug/total_cost", 1000); // cost
		debugPub2_ =
		    nh.advertise<std_msgs::Float64>("/debug/hx_norm", 1000); // hx norm
		debugPub3_ =
		    nh.advertise<std_msgs::Float64>("/debug/gx_norm", 1000); // gx norm
		debugPub4_ = nh.advertise<std_msgs::Float64>("/debug/grad_norm",
		                                             1000); // grad norm
		debug_energy_pub_ =
		    nh.advertise<std_msgs::Float64>("/debug/energy", 1000);
		debug_time_pub_ = nh.advertise<std_msgs::Float64>("/debug/time", 1000);
		debug_obs_quad_pub_ =
		    nh.advertise<std_msgs::Float64>("/debug/obs_quad", 1000);
		debug_obs_load_pub_ =
		    nh.advertise<std_msgs::Float64>("/debug/obs_load", 1000);
		debug_vel_pub_ = nh.advertise<std_msgs::Float64>("/debug/vel", 1000);
		debug_acc_pub_ = nh.advertise<std_msgs::Float64>("/debug/acc", 1000);
		debug_jer_pub_ = nh.advertise<std_msgs::Float64>("/debug/jer", 1000);
		debug_theta_pub_ =
		    nh.advertise<std_msgs::Float64>("/debug/theta", 1000);
		debug_omg_pub_ = nh.advertise<std_msgs::Float64>("/debug/omg", 1000);
		debug_thrust_pub_ =
		    nh.advertise<std_msgs::Float64>("/debug/thrust", 1000);
		debug_tension_pub_ =
		    nh.advertise<std_msgs::Float64>("/debug/tension", 1000);
		debug_dist_pub_ =
		    nh.advertise<std_msgs::Float64>("/debug/distance", 1000);
		debug_dyn_pub_ =
		    nh.advertise<std_msgs::Float64>("/debug/dynamic", 1000);
		debug_comp_pub_ =
		    nh.advertise<std_msgs::Float64>("/debug/complementarity", 1000);
		debug_acc_load_pub_ =
		    nh.advertise<std_msgs::Float64>("/debug/accload", 1000);
	}
	// physical parameters
	nh.param("physical_params/gravity", info_params_.gravity, -1.0);
	nh.param("physical_params/quadrotor_mass", info_params_.quad_mass, -1.0);
	nh.param("physical_params/payload_mass", info_params_.load_mass, -1.0);
	nh.param("physical_params/cable_length", info_params_.cable_length, -1.0);
	/* constraint parameters */
	nh.param("alm/init_vel", info_params_.init_vel, 0.5);
	// limitations
	nh.param("alm/max_vel_quad", cst_params_.max_vel_quad, -1.0);
	nh.param("alm/max_acc_quad", cst_params_.max_acc_quad, -1.0);
	nh.param("alm/max_jer_quad", cst_params_.max_jer_quad, -1.0);
	nh.param("alm/max_acc_load", cst_params_.max_acc_load, -1.0);
	nh.param("alm/max_theta", cst_params_.max_theta, -1.0);
	nh.param("alm/max_thrust", cst_params_.max_thr, -1.0);
	nh.param("alm/min_thrust", cst_params_.min_thr, -1.0);
	nh.param("alm/max_omg", cst_params_.max_omg, -1.0);
	nh.param("alm/max_tension", cst_params_.max_tension, -1.0);
	nh.param("alm/min_tension", cst_params_.min_tension, 0.0);
	nh.param("alm/min_distance", cst_params_.min_distance, 0.3);
	// flags and weights
	nh.param("alm/flag_obs", cst_params_.flag_obs, 0);
	nh.param("alm/flag_vel_quad", cst_params_.flag_vel_quad, 0);
	nh.param("alm/flag_acc_quad", cst_params_.flag_acc_quad, 0);
	nh.param("alm/flag_jer_quad", cst_params_.flag_jer_quad, 0);
	nh.param("alm/flag_acc_load", cst_params_.flag_acc_load, 0);
	nh.param("alm/flag_theta", cst_params_.flag_theta, 0);
	nh.param("alm/flag_thrust", cst_params_.flag_thr, 0);
	nh.param("alm/flag_omg", cst_params_.flag_omg, 0);
	nh.param("alm/flag_ten", cst_params_.flag_ten, 0);
	nh.param("alm/flag_dis", cst_params_.flag_dis, 0);
	nh.param("alm/flag_dyn", cst_params_.flag_dyn, 0);
	nh.param("alm/flag_com", cst_params_.flag_com, 0);

	nh.param("alm/weight_energy", cst_params_.weight_energy, -1.0);
	nh.param("alm/weight_t", cst_params_.weight_t, -1.0);
	nh.param("alm/weight_obs", cst_params_.weight_obs, -1.0);
	nh.param("alm/weight_vel_quad", cst_params_.weight_vel_quad, -1.0);
	nh.param("alm/weight_acc_quad", cst_params_.weight_acc_quad, -1.0);
	nh.param("alm/weight_jer_quad", cst_params_.weight_jer_quad, -1.0);
	nh.param("alm/weight_acc_load", cst_params_.weight_acc_load, -1.0);
	nh.param("alm/weight_theta", cst_params_.weight_theta, -1.0);
	nh.param("alm/weight_thrust", cst_params_.weight_thr, -1.0);
	nh.param("alm/weight_omg", cst_params_.weight_omg, -1.0);
	nh.param("alm/weight_ten", cst_params_.weight_ten, -1.0);
	nh.param("alm/weight_dis", cst_params_.weight_dis, -1.0);
	nh.param("alm/weight_dyn", cst_params_.weight_dyn, -1.0);
	nh.param("alm/weight_com", cst_params_.weight_com, -1.0);

	/* optimizer parameters */
	// fixed params
	info_params_.traj_order = 7;
	info_params_.state_dim = 6;
	info_params_.safe_margin = 0.2;
	// optimization params
	nh.param("alm/sample_num", info_params_.sample_num, 1);
	nh.param("alm/integral_intervs", opt_params_.integral_resolution, 0);
	nh.param("alm/opt_rho", opt_params_.rho, 1.0);
	nh.param("alm/opt_gamma", opt_params_.gamma, 1.0);
	nh.param("alm/opt_rho_max", opt_params_.rho_max, -1.0);
	nh.param("alm/opt_max_iter", opt_params_.max_iter, 1);
	nh.param("alm/opt_eq_constraints_eps", opt_params_.eq_cons_eps, 1.0e-2);
	nh.param("alm/opt_ineq_constraints_eps", opt_params_.ineq_cons_eps, 1.0e-2);
	nh.param("alm/opt_precision_eps", opt_params_.prec_eps, 1.0e-3);
	nh.param("alm/opt_scaling_wf_min", opt_params_.scaling_wf_min, 0.1);
	nh.param("alm/opt_scaling_wc_min", opt_params_.scaling_wc_min, 0.05);
	// line search parameters
	nh.param("lbfgs/mem_size", lbfgs_params_.mem_size, 16);
	nh.param("lbfgs/past", lbfgs_params_.past, 0);
	nh.param("lbfgs/delta", lbfgs_params_.delta, 1e-5);
	nh.param("lbfgs/min_step", lbfgs_params_.min_step, 1e-32);
	nh.param("lbfgs/g_epsilon", lbfgs_params_.g_epsilon, 1e-6);
	nh.param("lbfgs/max_linesearch", lbfgs_params_.max_linesearch, 128);
	nh.param("lbfgs/s_curv_coeff", lbfgs_params_.s_curv_coeff, 0.99);
	nh.param("lbfgs/f_dec_coeff", lbfgs_params_.f_dec_coeff, 1e-4);

	return;
}

void HybridOPT::initGridMap(const GridMap::Ptr& grid_map) {
	grid_map_ = grid_map;
	return;
}

void HybridOPT::initPoints(const vector<Eigen::VectorXd>& init_path,
                           Eigen::MatrixXd& inner_points) {
	double piece_num_ = init_path.size() - 1;
	if (piece_num_ > 1) {
		inner_points.resize(info_params_.state_dim, piece_num_ - 1);
		inner_points.setZero();
		for (int i = 1; i < piece_num_; i++) {
			inner_points.col(i - 1) = init_path[i];
		}
	} else {
		piece_num_ = 2;
		inner_points.resize(info_params_.state_dim, 1);
		inner_points.setZero();
		inner_points.col(0) = (init_path[0] + init_path[1]) / 2;
	}
	return;
}

void HybridOPT::initTimeAllocation(Eigen::MatrixXd& inner_points,
                                   Eigen::VectorXd& time_allocation) {
	Eigen::VectorXd start_pt = info_params_.init_state_mat.col(0);
	Eigen::VectorXd end_pt = info_params_.goal_state_mat.col(0);

	time_allocation.resize(inner_points.cols() + 1);
	time_allocation.setZero();

	info_params_.minco.reset(
	    new minco::MINCO(info_params_.traj_order, info_params_.state_dim));
	info_params_.minco->setConditions(info_params_.init_state_mat,
	                                  info_params_.goal_state_mat,
	                                  time_allocation.size());

	poly_traj::Trajectory::Ptr load_traj_, quad_traj_;
	load_traj_.reset(new poly_traj::Trajectory);
	load_traj_->init(info_params_.traj_order);
	quad_traj_.reset(new poly_traj::Trajectory);
	quad_traj_->init(info_params_.traj_order);

	double des_vel_ = info_params_.init_vel;
	int count = 0;
	do {
		time_allocation(0) =
		    (inner_points.col(0).tail(3) - start_pt.tail(3)).norm() / des_vel_;
		for (int i = 1; i < inner_points.cols(); ++i) {
			time_allocation(i) =
			    (inner_points.col(i).tail(3) - inner_points.col(i - 1).tail(3))
			        .norm() /
			    des_vel_;
		}
		time_allocation(inner_points.cols()) =
		    (end_pt.tail(3) - inner_points.col(inner_points.cols() - 1).tail(3))
		        .norm() /
		    des_vel_;
		info_params_.minco->setParameters(inner_points, time_allocation);
		info_params_.minco->getTrajectory(load_traj_, quad_traj_);
		des_vel_ *= 0.8;
		count++;
	} while (quad_traj_->getMaxVelRate() > cst_params_.max_vel_quad &&
	         count < 10);

	int piece_num_ = info_params_.sample_num + 1;
	inner_points.resize(info_params_.state_dim, piece_num_ - 1);
	time_allocation.resize(piece_num_);
	info_params_.minco->getTrajectory(load_traj_, quad_traj_);

	double dt_ = load_traj_->getTotalDuration() / piece_num_;
	std::cout << "Initial duration per piece: " << dt_ << std::endl;
	for (int i = 1; i < piece_num_; i++) {
		inner_points.col(i - 1).head(3) = load_traj_->getPos(dt_ * i);
		inner_points.col(i - 1).tail(3) = quad_traj_->getPos(dt_ * i);
		time_allocation(i - 1) = dt_;
	}
	time_allocation(piece_num_ - 1) = dt_;
}

double HybridOPT::dynamicCostFunctional(void* func_data,
                                        const Eigen::VectorXd& xi,
                                        Eigen::VectorXd& grad) {
	HybridOPT& obj = *(HybridOPT*)func_data;
	Eigen::Map<const Eigen::MatrixXd> inner_pts(
	    xi.data(), obj.info_params_.inner_pts.rows(),
	    obj.info_params_.inner_pts.cols());
	Eigen::Map<const Eigen::VectorXd> tau(xi.data() +
	                                          obj.info_params_.inner_pts.size(),
	                                      obj.info_params_.durations.size());
	Eigen::Map<Eigen::MatrixXd> grad_inner_pts(
	    grad.data(), obj.info_params_.inner_pts.rows(),
	    obj.info_params_.inner_pts.cols());
	Eigen::Map<Eigen::VectorXd> grad_tau(grad.data() +
	                                         obj.info_params_.inner_pts.size(),
	                                     obj.info_params_.durations.size());

	Eigen::MatrixXd grad_pts;
	Eigen::MatrixXd partial_grad_by_coeffs((obj.info_params_.traj_order + 1) *
	                                           obj.info_params_.pieces_num,
	                                       obj.info_params_.state_dim);
	Eigen::VectorXd partial_grad_by_durations(
	    obj.info_params_.durations.size());
	Eigen::VectorXd durations(obj.info_params_.durations.size());
	Eigen::VectorXd grad_by_durations(obj.info_params_.durations.size());

	forwardT(tau, durations);
	obj.info_params_.inner_pts = inner_pts;
	obj.info_params_.durations = durations;
	obj.updateMINCO();

	grad_tau.setZero();
	grad_inner_pts.setZero();
	grad_by_durations.setZero();
	partial_grad_by_coeffs.setZero();
	partial_grad_by_durations.setZero();

	double cost(0);

	Eigen::MatrixXd coeffs;
	Eigen::VectorXd pos_, vel_, acc_, jer_;
	Eigen::Vector3d load_pos_, load_vel_, load_acc_, load_jer_;
	Eigen::Vector3d quad_pos_, quad_vel_;
	Eigen::Vector3d term_partial_load_pos_, term_partial_load_acc_,
	    term_partial_quad_pos_;
	Eigen::Vector3d total_grad_load_pos_, total_grad_load_acc_,
	    total_grad_quad_pos_;
	Eigen::Vector3d alm_partial_load_pos_, alm_partial_load_acc_,
	    alm_partial_quad_pos_;
	double step_, alpha_;
	double node_, pena_, alm_pena_;
	double s1, s2, s3, s4, s5, s6, s7;
	Eigen::Matrix<double, 8, 1> beta0, beta1, beta2, beta3, beta4, beta5, beta6;
	const double integra_frac_ = 1.0 / obj.opt_params_.integral_resolution;
	const double traj_order_ = obj.info_params_.traj_order;

	double dyn_pena_;
	double dis_mean_ =
	    0.5 * (obj.info_params_.cable_length + obj.cst_params_.min_distance);
	double dis_radi_ = 0.5 * fabs(obj.info_params_.cable_length -
	                              obj.cst_params_.min_distance);
	double obs_pena_quad_, obs_pena_load_;

	coeffs = obj.info_params_.minco->getCoeffs();

	for (int i = 0; i < obj.info_params_.pieces_num; i++) {
		Eigen::MatrixXd c_;
		c_.resize(traj_order_ + 1, 6);
		c_ = coeffs.block(i * (traj_order_ + 1), 0, traj_order_ + 1, 6);

		step_ = durations(i) * integra_frac_;
		for (int j = 0; j <= obj.opt_params_.integral_resolution; j++) {
			s1 = j * step_;
			s2 = s1 * s1;
			s3 = s2 * s1;
			s4 = s3 * s1;
			s5 = s4 * s1;
			s6 = s5 * s1;
			s7 = s6 * s1;

			beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3,
			beta0(4) = s4, beta0(5) = s5, beta0(6) = s6,
			beta0(7) = s7; // beta0(8) = s8, beta0(9) = s9;
			beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1,
			beta1(3) = 3.0 * s2, beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4,
			beta1(6) = 6.0 * s5,
			beta1(7) = 7.0 * s6; // beta1(8) = 8.0 * s7, beta1(9) = 9.0 * s8;
			beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1,
			beta2(4) = 12.0 * s2, beta2(5) = 20.0 * s3, beta2(6) = 30.0 * s4,
			beta2(7) = 42.0 * s5; // beta2(8) = 56.0 * s6, beta2(9) = 72.0 * s7;
			beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0,
			beta3(4) = 24.0 * s1, beta3(5) = 60.0 * s2, beta3(6) = 120.0 * s3,
			beta3(7) =
			    210.0 * s4; // beta3(8) = 336.0 * s5, beta3(9) = 504.0 * s6;
			beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0,
			beta4(4) = 24.0, beta4(5) = 120.0 * s1, beta4(6) = 360.0 * s2,
			beta4(7) =
			    840.0 * s3; // beta4(8) = 1680.0 * s4, beta4(9) = 3024.0 * s5;
			beta5(0) = 0.0, beta5(1) = 0.0, beta5(2) = 0.0, beta5(3) = 0.0,
			beta5(4) = 0.0, beta5(5) = 120.0, beta5(6) = 720.0 * s1,
			beta5(7) =
			    2520.0 * s2; // beta5(8) = 6720.0 * s3, beta5(9) = 15120.0 * s4;
			beta6(0) = 0.0, beta6(1) = 0.0, beta6(2) = 0.0, beta6(3) = 0.0,
			beta6(4) = 0.0, beta6(5) = 0.0, beta6(6) = 720.0,
			beta6(7) = 5040.0 *
			           s1; // beta6(8) = 20160.0 * s2, beta6(9) = 60480.0 * s3;

			pos_ = c_.transpose() * beta0;
			vel_ = c_.transpose() * beta1;
			acc_ = c_.transpose() * beta2;
			jer_ = c_.transpose() * beta3;

			load_pos_ = pos_.head(3);
			load_vel_ = vel_.head(3);
			load_acc_ = acc_.head(3);
			load_jer_ = jer_.head(3);
			quad_pos_ = pos_.tail(3);
			quad_vel_ = vel_.tail(3);

			total_grad_load_pos_.setZero();
			total_grad_load_acc_.setZero();
			total_grad_quad_pos_.setZero();

			pena_ = 0.0;
			node_ = (j == 0 || j == obj.opt_params_.integral_resolution) ? 0.5
			                                                             : 1.0;
			alpha_ = j * integra_frac_;

			if (obj.cst_params_.flag_dyn != 0) {
				obj.equalityConstraintDyn(
				    1.0, 0.0, 1.0, load_pos_, load_acc_, quad_pos_, dyn_pena_,
				    term_partial_load_pos_, term_partial_load_acc_,
				    term_partial_quad_pos_, alm_pena_, alm_partial_load_pos_,
				    alm_partial_load_acc_, alm_partial_quad_pos_);
				pena_ += dyn_pena_;
				total_grad_load_pos_ += term_partial_load_pos_;
				total_grad_load_acc_ += term_partial_load_acc_;
				total_grad_quad_pos_ += term_partial_quad_pos_;
			}

			partial_grad_by_coeffs.block(i * (traj_order_ + 1), 0,
			                             traj_order_ + 1, 3) +=
			    (beta0 * total_grad_load_pos_.transpose() +
			     beta2 * total_grad_load_acc_.transpose()) *
			    node_ * step_;
			partial_grad_by_coeffs.block(i * (traj_order_ + 1), 3,
			                             traj_order_ + 1, 3) +=
			    (beta0 * total_grad_quad_pos_.transpose()) * node_ * step_;
			partial_grad_by_durations(i) +=
			    (total_grad_load_pos_.dot(load_vel_) +
			     total_grad_load_acc_.dot(load_jer_) +
			     total_grad_quad_pos_.dot(quad_vel_)) *
			    alpha_ * node_ * step_;
			cost += pena_ * node_ * step_;
		}
	}
	// std::cout << "partial coeff grad: " <<
	// partial_grad_by_coeffs.cwiseAbs().maxCoeff() << std::endl; std::cout <<
	// "partial duration grad: " <<
	// partial_grad_by_durations.cwiseAbs().maxCoeff() << std::endl;
	obj.info_params_.minco->propogateGrad(partial_grad_by_coeffs,
	                                      partial_grad_by_durations, grad_pts,
	                                      grad_by_durations);
	grad_inner_pts = grad_pts;
	backwardGradT(tau, grad_by_durations, grad_tau);

	// std::cout << "cost: " << cost << std::endl;
	// std::cout << "grad: " << grad.cwiseAbs().maxCoeff() << std::endl;
	return cost;
}

void HybridOPT::dynamicOptimization(void) {
	Eigen::VectorXd xi(info_params_.decision_variable_num);
	xi.setZero();

	xi.head(info_params_.inner_pts.size()) = Eigen::Map<const Eigen::VectorXd>(
	    info_params_.inner_pts.data(), info_params_.inner_pts.size());
	Eigen::Map<Eigen::VectorXd> tau(xi.data() + info_params_.inner_pts.size(),
	                                info_params_.durations.size());
	backwardT(info_params_.durations, tau);

	dynamic_opt_lbfgs_params_.mem_size = 4096;
	dynamic_opt_lbfgs_params_.past = 0;
	dynamic_opt_lbfgs_params_.delta = 1e-5;
	dynamic_opt_lbfgs_params_.min_step = 1e-32;
	dynamic_opt_lbfgs_params_.g_epsilon = 1e-4;
	dynamic_opt_lbfgs_params_.max_linesearch = 256;

	int result;
	double cost = 0.0;
	result = lbfgs::lbfgs_optimize(xi, cost, &HybridOPT::dynamicCostFunctional,
	                               nullptr, nullptr, this,
	                               dynamic_opt_lbfgs_params_);
	if ((result != lbfgs::LBFGS_CONVERGENCE) && (result != lbfgs::LBFGS_STOP)) {
		ROS_ERROR("L-BFGS optimization failed! Result is : %s \n",
		          lbfgs::lbfgs_strerror(result));
	} else {
		ROS_INFO("L-BFGS optimization success! Result is : %s \n",
		         lbfgs::lbfgs_strerror(result));
	}
	info_params_.inner_pts = Eigen::Map<Eigen::MatrixXd>(
	    xi.data(), 6, info_params_.inner_pts.cols());
	tau = Eigen::Map<Eigen::VectorXd>(xi.data() + info_params_.inner_pts.size(),
	                                  info_params_.durations.size());
	forwardT(tau, info_params_.durations);
	updateMINCO();
}

void HybridOPT::updateMINCO(void) {
	info_params_.minco->setConditions(info_params_.init_state_mat,
	                                  info_params_.goal_state_mat,
	                                  info_params_.durations.size());
	info_params_.minco->setParameters(info_params_.inner_pts,
	                                  info_params_.durations);
}

void HybridOPT::initScalingW(const Eigen::VectorXd& xi) {
	int cts_num = opt_params_.inequality_constraint_num +
	              opt_params_.equality_constraint_num;
	opt_params_.scaling_wf = 1.0;
	opt_params_.scaling_wc.resize(cts_num);
	opt_params_.scaling_wc.setConstant(1.0);
	Eigen::VectorXd gradf(xi.size());
	Eigen::MatrixXd gradc(cts_num, xi.size());
	gradf.setZero();
	gradc.setZero();

	Eigen::Map<const Eigen::MatrixXd> inner_pts(xi.data(),
	                                            info_params_.inner_pts.rows(),
	                                            info_params_.inner_pts.cols());
	Eigen::Map<const Eigen::VectorXd> tau(xi.data() +
	                                          info_params_.inner_pts.size(),
	                                      info_params_.durations.size());
	Eigen::Map<Eigen::MatrixXd> grad_inner_pts(gradf.data(),
	                                           info_params_.inner_pts.rows(),
	                                           info_params_.inner_pts.cols());
	Eigen::Map<Eigen::VectorXd> grad_tau(gradf.data() +
	                                         info_params_.inner_pts.size(),
	                                     info_params_.durations.size());

	Eigen::MatrixXd grad_inpts;
	Eigen::MatrixXd partial_grad_by_coeffs;
	Eigen::VectorXd partial_grad_by_durations;
	Eigen::VectorXd durations, grad_by_durations;
	double cost = 0.0;
	int idx = 0;

	forwardT(tau, durations);
	updateMINCO();

	info_params_.minco->getEnergy(cost);
	info_params_.minco->getEnergyPartialGradByCoeffs(partial_grad_by_coeffs);
	info_params_.minco->getEnergyPartialGradByTimes(partial_grad_by_durations);

	std::cout << "energy cost: " << cost << std::endl;
	std::cout << "time cost: " << durations.sum() << std::endl;

	augmentedLagrangianFunctional(
	    durations, info_params_.minco->getCoeffs(), info_params_.pieces_num,
	    idx, cost, partial_grad_by_durations, partial_grad_by_coeffs, gradc);

	std::cout << "penalty cost: " << cost << std::endl;
	std::cout << "obs quad cost: " << debug_params_.init_cost_obs_quad
	          << std::endl;
	std::cout << "obs load cost: " << debug_params_.init_cost_obs_load
	          << std::endl;
	std::cout << "vel cost: " << debug_params_.init_cost_vel << std::endl;
	std::cout << "acc cost: " << debug_params_.init_cost_acc << std::endl;
	std::cout << "jerk cost: " << debug_params_.init_cost_jer << std::endl;
	std::cout << "theta cost: " << debug_params_.init_cost_theta << std::endl;
	std::cout << "omg cost: " << debug_params_.init_cost_omg << std::endl;
	std::cout << "thrust cost: " << debug_params_.init_cost_thrust << std::endl;
	std::cout << "dis cost: " << debug_params_.init_cost_dist << std::endl;
	std::cout << "ten cost: " << debug_params_.init_cost_tension << std::endl;
	std::cout << "dyn cost: " << debug_params_.init_cost_dyn << std::endl;
	std::cout << "comp cost: " << debug_params_.init_cost_comp << std::endl;

	info_params_.minco->propogateGrad(partial_grad_by_coeffs,
	                                  partial_grad_by_durations, grad_inpts,
	                                  grad_by_durations);

	cost += cst_params_.weight_t * durations.sum();
	grad_by_durations.array() += cst_params_.weight_t;
	grad_inner_pts = grad_inpts;
	backwardGradT(tau, grad_by_durations, grad_tau);

	// calculate scaling wf and wc
	opt_params_.scaling_wf =
	    std::max(opt_params_.scaling_wf_min,
	             1.0 / std::max(1.0, gradf.cwiseAbs().maxCoeff()));
	opt_params_.scaling_wc.resize(opt_params_.equality_constraint_num +
	                              opt_params_.inequality_constraint_num);
	for (int i = 0; i < opt_params_.scaling_wc.size(); i++) {
		opt_params_.scaling_wc(i) =
		    std::max(opt_params_.scaling_wc_min,
		             1.0 / std::max(1.0, gradc.row(i).cwiseAbs().maxCoeff()));
	}
}

bool HybridOPT::setup(const vector<Eigen::VectorXd>& init_path) {
	flatmap_.reset(info_params_.quad_mass, info_params_.load_mass,
	               info_params_.gravity, 0.0001);
	// initial state matrix
	info_params_.init_state_mat.setZero();
	info_params_.goal_state_mat.setZero();
	info_params_.init_state_mat.col(0) = init_path[0];
	info_params_.goal_state_mat.col(0) = init_path[init_path.size() - 1];
	// initial inner points
	initPoints(init_path, info_params_.inner_pts);
	// initial time allocation
	initTimeAllocation(info_params_.inner_pts, info_params_.durations);
	// inital minco
	info_params_.minco.reset(
	    new minco::MINCO(info_params_.traj_order, info_params_.state_dim));
	updateMINCO();

	/* basic information */
	info_params_.pieces_num = info_params_.durations.size();
	info_params_.decision_variable_num =
	    info_params_.inner_pts.size() + info_params_.durations.size();

	/* optimizer parameters */
	opt_params_.equality_constraint_num =
	    info_params_.pieces_num * (opt_params_.integral_resolution + 1) * 2;
	opt_params_.inequality_constraint_num =
	    info_params_.pieces_num * (opt_params_.integral_resolution + 1) * 11;
	if (info_params_.debug_flag) {
		ROS_INFO("inner pts size = %ld, durations size = %ld",
		         info_params_.inner_pts.size(), info_params_.durations.size());
		ROS_INFO("eq cst num = %d, ieq cst num = %d",
		         opt_params_.equality_constraint_num,
		         opt_params_.inequality_constraint_num);
	}
	// constraints terms
	opt_params_.cx.resize(opt_params_.equality_constraint_num +
	                      opt_params_.inequality_constraint_num);
	opt_params_.cx.setZero();
	opt_params_.cx_flag.resize(opt_params_.equality_constraint_num +
	                           opt_params_.inequality_constraint_num);
	opt_params_.cx_flag.setZero();
	// lagrangian parameters vector
	opt_params_.lambda.resize(opt_params_.equality_constraint_num +
	                          opt_params_.inequality_constraint_num);
	opt_params_.lambda.setZero();
	// Outer loop stop flag and counter
	opt_params_.stop_flag = false;
	opt_params_.k = 0;

	// inital dynamic feasibility
	dynamicOptimization();

	return true;
}

double HybridOPT::optimize(utils::Visualizer::Ptr vis) {
	info_params_.inner_opt_cost = 0.0;
	info_params_.update_info_cost = 0.0;
	info_params_.check_opt_cost = 0.0;
	info_params_.total_opt_cost = 0.0;
	ros::Time t_opt_, t_update_, t_check_, t_total_;

	visualizer_ = vis;

	t_total_ = ros::Time::now();
	/* initialize decision variables */
	// set decision variables
	Eigen::VectorXd xi;
	xi.resize(info_params_.decision_variable_num);
	xi.setZero();
	xi.head(info_params_.inner_pts.size()) = Eigen::Map<const Eigen::VectorXd>(
	    info_params_.inner_pts.data(), info_params_.inner_pts.size());
	Eigen::Map<Eigen::VectorXd> tau(xi.data() + info_params_.inner_pts.size(),
	                                info_params_.durations.size());
	backwardT(info_params_.durations, tau);

	initScalingW(xi);

	int result;
	double cost = 0.0;
	while (!opt_params_.stop_flag) {
		opt_params_.k += 1;
		// Step 1: solve an unconstrained problem to get the approximate
		// solution of the Lagrangian function
		//         line search method - lbfgs, a popular quasi-Newton method
		std::cout << "---------------Loop " << opt_params_.k
		          << "----------------" << endl;
		if (info_params_.debug_flag) {
			t_opt_ = ros::Time::now();
			result = lbfgs::lbfgs_optimize(
			    xi, cost, &HybridOPT::costFunctional, nullptr,
			    &HybridOPT::innerLoopDebugFunctional, this, lbfgs_params_);
			info_params_.inner_opt_cost += (ros::Time::now() - t_opt_).toSec();
			std::cout << "Inner loop optimal grad: "
			          << opt_params_.grad.cwiseAbs().maxCoeff() << endl;
			ROS_INFO("Detail costs: %f, %f, %f, %f, %f, %f, %f.\n",
			         debug_params_.time1 / debug_params_.count,
			         debug_params_.time2 / debug_params_.count,
			         debug_params_.time3 / debug_params_.count,
			         debug_params_.time4 / debug_params_.count,
			         debug_params_.time5 / debug_params_.count,
			         debug_params_.time6 / debug_params_.count,
			         debug_params_.time7 / debug_params_.count);
		} else {
			t_opt_ = ros::Time::now();
			result =
			    lbfgs::lbfgs_optimize(xi, cost, &HybridOPT::costFunctional,
			                          nullptr, nullptr, this, lbfgs_params_);
			info_params_.inner_opt_cost += (ros::Time::now() - t_opt_).toSec();
		}
		debug_params_.count = 0;
		// check if the optimization is successful
		if ((result != lbfgs::LBFGS_CONVERGENCE) &&
		    (result != lbfgs::LBFGS_STOP)) {
			ROS_ERROR("L-BFGS optimization failed! Result is : %s \n",
			          lbfgs::lbfgs_strerror(result));
		} else {
			ROS_INFO("L-BFGS optimization success! Result is : %s \n",
			         lbfgs::lbfgs_strerror(result));
		}
		/* Step 2: update optimal info */
		t_update_ = ros::Time::now();
		info_params_.inner_pts = Eigen::Map<Eigen::MatrixXd>(
		    xi.data(), 6, info_params_.inner_pts.cols());
		tau = Eigen::Map<Eigen::VectorXd>(xi.data() +
		                                      info_params_.inner_pts.size(),
		                                  info_params_.durations.size());
		forwardT(tau, info_params_.durations);
		updateMINCO();
		info_params_.update_info_cost += (ros::Time::now() - t_update_).toSec();
		/* Step 3: check if the solution satisfies the AKKT conditions */
		t_check_ = ros::Time::now();
		updateALM();
		check_optimal();
		info_params_.check_opt_cost += (ros::Time::now() - t_check_).toSec();

		if (opt_params_.stop_flag && (result == lbfgs::LBFGS_CONVERGENCE ||
		                              result == lbfgs::LBFGS_STOP)) {
			ROS_INFO("FIND THE OPTIMAL SOLUTION!");
			break;
		}

		if (opt_params_.k >= opt_params_.max_iter) {
			ROS_INFO("MAXIMUM ITERATION REACHED!");
			break;
		}
	}
	info_params_.total_opt_cost = (ros::Time::now() - t_total_).toSec();

	updateTrajectories();

	if (info_params_.debug_flag) {
		ROS_INFO("Inner optimization cost: %f s, average: %f s.\n",
		         info_params_.inner_opt_cost,
		         info_params_.inner_opt_cost / opt_params_.k);
		ROS_INFO("Info update cost: %f s, average: %f s.\n",
		         info_params_.update_info_cost,
		         info_params_.update_info_cost / opt_params_.k);
		ROS_INFO("Check optimal cost: %f s, average: %f s.\n",
		         info_params_.check_opt_cost,
		         info_params_.check_opt_cost / opt_params_.k);
		ROS_INFO("Total optimization cost: %f.\n", info_params_.total_opt_cost);
	}
	return true;
}

void HybridOPT::updateTrajectories(void) {
	load_traj_.reset(new poly_traj::Trajectory);
	quad_traj_.reset(new poly_traj::Trajectory);
	load_traj_->init(info_params_.traj_order);
	quad_traj_->init(info_params_.traj_order);
	info_params_.minco->getTrajectory(load_traj_, quad_traj_);
}

int HybridOPT::innerLoopDebugFunctional(void* func_data,
                                        const Eigen::VectorXd& xi,
                                        const Eigen::VectorXd& grad,
                                        const double f, const double step,
                                        const int index, const int err_code) {
	HybridOPT& obj = *(HybridOPT*)func_data;

	obj.data1_.data = f;
	obj.data2_.data = index;
	obj.data3_.data = step;
	obj.data4_.data = grad.cwiseAbs().maxCoeff();
	obj.debugPub1_.publish(obj.data1_);
	obj.debugPub2_.publish(obj.data2_);
	obj.debugPub3_.publish(obj.data3_);
	obj.debugPub4_.publish(obj.data4_);

	obj.debug_energy_pub_.publish(obj.data_energy_);
	obj.debug_time_pub_.publish(obj.data_time_);
	obj.debug_obs_quad_pub_.publish(obj.data_obs_quad_);
	obj.debug_obs_load_pub_.publish(obj.data_obs_load_);
	obj.debug_vel_pub_.publish(obj.data_vel_);
	obj.debug_acc_pub_.publish(obj.data_acc_);
	obj.debug_jer_pub_.publish(obj.data_jer_);
	obj.debug_theta_pub_.publish(obj.data_theta_);
	obj.debug_omg_pub_.publish(obj.data_omg_);
	obj.debug_thrust_pub_.publish(obj.data_thrust_);
	obj.debug_dist_pub_.publish(obj.data_dist_);
	obj.debug_tension_pub_.publish(obj.data_tension_);
	obj.debug_dyn_pub_.publish(obj.data_dyn_);
	obj.debug_comp_pub_.publish(obj.data_comp_);
	obj.debug_acc_load_pub_.publish(obj.data_acc_load_);

	obj.debug_params_.count += 1;
	if (obj.debug_params_.count % 10 == 0) {
		ros::Duration(0.05).sleep();
		obj.updateTrajectories();
		obj.visualizer_->visualizeDoubleball(obj.load_traj_, obj.quad_traj_, 3,
		                                     0.1, 0.2, obj.info_params_.gravity,
		                                     obj.info_params_.load_mass);
	}
	return 1;
}

double HybridOPT::costFunctional(void* func_data, const Eigen::VectorXd& xi,
                                 Eigen::VectorXd& grad) {
	HybridOPT& obj = *(HybridOPT*)func_data;

	obj.debug_params_.t1 = ros::Time::now();
	Eigen::Map<const Eigen::MatrixXd> inner_pts(
	    xi.data(), obj.info_params_.inner_pts.rows(),
	    obj.info_params_.inner_pts.cols());
	Eigen::Map<const Eigen::VectorXd> tau(xi.data() +
	                                          obj.info_params_.inner_pts.size(),
	                                      obj.info_params_.durations.size());
	Eigen::Map<Eigen::MatrixXd> grad_inner_pts(
	    grad.data(), obj.info_params_.inner_pts.rows(),
	    obj.info_params_.inner_pts.cols());
	Eigen::Map<Eigen::VectorXd> grad_tau(grad.data() +
	                                         obj.info_params_.inner_pts.size(),
	                                     obj.info_params_.durations.size());

	Eigen::MatrixXd grad_pts;
	Eigen::MatrixXd partial_grad_by_coeffs;
	Eigen::VectorXd partial_grad_by_durations;
	Eigen::VectorXd durations(obj.info_params_.durations.size());
	Eigen::VectorXd grad_by_durations(obj.info_params_.durations.size());

	double cost = 0.0;
	int idx = 0;

	forwardT(tau, durations);
	obj.debug_params_.time1 +=
	    (ros::Time::now() - obj.debug_params_.t1).toSec();
	obj.debug_params_.t2 = ros::Time::now();

	obj.info_params_.inner_pts = inner_pts;
	obj.info_params_.durations = durations;
	obj.updateMINCO();
	obj.debug_params_.time2 +=
	    (ros::Time::now() - obj.debug_params_.t2).toSec();
	obj.debug_params_.t3 = ros::Time::now();

	grad_by_durations.setZero();
	grad_inner_pts.setZero();

	obj.info_params_.minco->getEnergy(cost);
	obj.info_params_.minco->getEnergyPartialGradByCoeffs(
	    partial_grad_by_coeffs);
	obj.info_params_.minco->getEnergyPartialGradByTimes(
	    partial_grad_by_durations);

	obj.debug_params_.time3 +=
	    (ros::Time::now() - obj.debug_params_.t3).toSec();
	obj.debug_params_.t4 = ros::Time::now();

	cost = obj.opt_params_.scaling_wf * obj.cst_params_.weight_energy * cost;
	partial_grad_by_coeffs = obj.opt_params_.scaling_wf *
	                         obj.cst_params_.weight_energy *
	                         partial_grad_by_coeffs;
	partial_grad_by_durations = obj.opt_params_.scaling_wf *
	                            obj.cst_params_.weight_energy *
	                            partial_grad_by_durations;

	obj.debug_params_.time4 +=
	    (ros::Time::now() - obj.debug_params_.t4).toSec();
	obj.debug_params_.cost_energy = cost;
	// std::cout << "energy cost: " << cost << ", scaling: " <<
	// obj.opt_params_.scaling_wf << std::endl;
	obj.debug_params_.t5 = ros::Time::now();

	obj.augmentedLagrangianFunctional(
	    durations, obj.info_params_.minco->getCoeffs(),
	    obj.info_params_.pieces_num, idx, cost, partial_grad_by_durations,
	    partial_grad_by_coeffs);

	obj.debug_params_.time5 +=
	    (ros::Time::now() - obj.debug_params_.t5).toSec();
	// std::cout << "augmented cost: " << cost << std::endl;
	obj.debug_params_.t6 = ros::Time::now();

	obj.info_params_.minco->propogateGrad(partial_grad_by_coeffs,
	                                      partial_grad_by_durations, grad_pts,
	                                      grad_by_durations);
	cost +=
	    obj.opt_params_.scaling_wf * obj.cst_params_.weight_t * durations.sum();
	grad_by_durations.array() +=
	    obj.opt_params_.scaling_wf * obj.cst_params_.weight_t;

	obj.debug_params_.time6 +=
	    (ros::Time::now() - obj.debug_params_.t6).toSec();
	// std::cout << "duration cost: " << durations.sum() << std::endl;
	obj.debug_params_.cost_time =
	    obj.opt_params_.scaling_wf * obj.cst_params_.weight_t * durations.sum();
	obj.debug_params_.t7 = ros::Time::now();

	grad_inner_pts = grad_pts;
	backwardGradT(tau, grad_by_durations, grad_tau);

	obj.debug_params_.time7 +=
	    (ros::Time::now() - obj.debug_params_.t7).toSec();

	obj.opt_params_.grad = grad;
	obj.opt_params_.xi = xi;
	obj.opt_params_.cost = cost;
	if (obj.info_params_.debug_flag) {
		obj.data_energy_.data = obj.debug_params_.cost_energy;
		obj.data_time_.data = obj.debug_params_.cost_time;
		obj.data_obs_quad_.data = obj.debug_params_.cost_obs_quad;
		obj.data_obs_load_.data = obj.debug_params_.cost_obs_load;
		obj.data_vel_.data = obj.debug_params_.cost_vel;
		obj.data_acc_.data = obj.debug_params_.cost_acc;
		obj.data_jer_.data = obj.debug_params_.cost_jer;
		obj.data_theta_.data = obj.debug_params_.cost_theta;
		obj.data_omg_.data = obj.debug_params_.cost_omg;
		obj.data_thrust_.data = obj.debug_params_.cost_thrust;
		obj.data_dist_.data = obj.debug_params_.cost_dist;
		obj.data_tension_.data = obj.debug_params_.cost_tension;
		obj.data_dyn_.data = obj.debug_params_.cost_dyn;
		obj.data_comp_.data = obj.debug_params_.cost_comp;
		obj.data_acc_load_.data = obj.debug_params_.cost_acc_load;

		obj.debug_params_.cost_energy = 0.0;
		obj.debug_params_.cost_time = 0.0;
		obj.debug_params_.cost_obs_quad = 0.0;
		obj.debug_params_.cost_obs_load = 0.0;
		obj.debug_params_.cost_vel = 0.0;
		obj.debug_params_.cost_acc = 0.0;
		obj.debug_params_.cost_jer = 0.0;
		obj.debug_params_.cost_theta = 0.0;
		obj.debug_params_.cost_omg = 0.0;
		obj.debug_params_.cost_thrust = 0.0;
		obj.debug_params_.cost_dist = 0.0;
		obj.debug_params_.cost_tension = 0.0;
		obj.debug_params_.cost_dyn = 0.0;
		obj.debug_params_.cost_comp = 0.0;
		obj.debug_params_.cost_acc_load = 0.0;
	}

	return cost;
}

void HybridOPT::augmentedLagrangianFunctional(const Eigen::VectorXd& T,
                                              const Eigen::MatrixXd& coeffs,
                                              const int piece_nums, int& index,
                                              double& cost,
                                              Eigen::VectorXd& gradT,
                                              Eigen::MatrixXd& gradC) {
	// System info
	Eigen::VectorXd pos_, vel_, acc_, jer_, sna_, cra_, dcra_;
	Eigen::Vector3d load_pos_, load_vel_, load_acc_, load_jer_, load_sna_,
	    load_cra_, load_dcra_;
	Eigen::Vector3d quad_pos_, quad_vel_, quad_acc_, quad_jer_, quad_sna_,
	    quad_cra_, quad_dcra_;
	Eigen::Vector3d omg_;
	Eigen::Vector4d quat_;
	double thrust_;
	double step_, alpha_;
	double s1, s2, s3, s4, s5, s6, s7;
	Eigen::Matrix<double, 8, 1> beta0, beta1, beta2, beta3, beta4, beta5, beta6;
	const double integra_frac_ = 1.0 / opt_params_.integral_resolution;
	const double traj_order_ = info_params_.traj_order;

	// Gradience
	Eigen::Vector3d term_partial_quad_pos_, term_partial_quad_vel_,
	    term_partial_quad_acc_, term_partial_quad_jer_;
	Eigen::Vector3d term_partial_load_pos_, term_partial_load_vel_,
	    term_partial_load_acc_, term_partial_load_jer_;

	Eigen::Vector3d total_grad_quad_pos_, total_grad_quad_vel_,
	    total_grad_quad_acc_, total_grad_quad_jer_;
	Eigen::Vector3d total_grad_load_pos_, total_grad_load_vel_,
	    total_grad_load_acc_, total_grad_load_jer_;

	Eigen::Vector3d cts_partial_quad_pos_, cts_partial_quad_vel_,
	    cts_partial_quad_acc_, cts_partial_quad_jer_;
	Eigen::Vector3d cts_partial_load_pos_, cts_partial_load_vel_,
	    cts_partial_load_acc_, cts_partial_load_jer_;

	// Cost
	double node_, pena_;

	// Obstacle avoidance parameters
	double weight_obs_quad_ = cst_params_.weight_obs;
	double flag_obs_quad_ = cst_params_.flag_obs;
	double obs_pena_quad_;
	double weight_obs_load_ = cst_params_.weight_obs;
	double flag_obs_load_ = cst_params_.flag_obs;
	double obs_pena_load_;
	// Maximum velocity of quadrotor penalty parameters
	double weight_vel_quad_ = cst_params_.weight_vel_quad;
	double flag_vel_quad_ = cst_params_.flag_vel_quad;
	double vel_sqr_bound_quad_ =
	    cst_params_.max_vel_quad * cst_params_.max_vel_quad;
	double vel_pena_quad_;
	// Maximum acceleration of quadrotor penalty parameters
	double weight_acc_quad_ = cst_params_.weight_acc_quad;
	double flag_acc_quad_ = cst_params_.flag_acc_quad;
	double acc_sqr_bound_quad_ =
	    cst_params_.max_acc_quad * cst_params_.max_acc_quad;
	double acc_pena_quad_;
	// Maximum acceleration of quadrotor penalty parameters
	double weight_acc_load_ = cst_params_.weight_acc_load;
	double flag_acc_load_ = cst_params_.flag_acc_load;
	double acc_sqr_bound_load_ =
	    cst_params_.max_acc_load * cst_params_.max_acc_load;
	double acc_pena_load_;
	// Maximum jerk of quadrotor penalty parameters
	double weight_jer_quad_ = cst_params_.weight_jer_quad;
	double flag_jer_quad_ = cst_params_.flag_jer_quad;
	double jer_sqr_bound_quad_ =
	    cst_params_.max_jer_quad * cst_params_.max_jer_quad;
	double jer_pena_quad_;
	// Maximum body angle of quadrotor penalty parameters
	double weight_theta_ = cst_params_.weight_theta;
	double flag_theta_ = cst_params_.flag_theta;
	double theta_bound_ = cst_params_.max_theta;
	double theta_pena_;
	// Maximum body rate of quadrotor penalty parameters
	double weight_omg_ = cst_params_.weight_omg;
	double flag_omg_ = cst_params_.flag_omg;
	double omg_bound_ = cst_params_.max_omg * cst_params_.max_omg;
	double omg_pena_;
	// Maximum thrust of quadrotor penalty parameters
	double weight_thr_ = cst_params_.weight_thr;
	double flag_thr_ = cst_params_.flag_thr;
	double thr_mean_ = 0.5 * (cst_params_.max_thr + cst_params_.min_thr);
	double thr_radi_ = 0.5 * fabs(cst_params_.max_thr - cst_params_.min_thr);
	double thr_pena_;
	// Maximum tension penalty parameters
	double weight_ten_ = cst_params_.weight_ten;
	double flag_ten_ = cst_params_.flag_ten;
	double ten_mean_ =
	    0.5 * (cst_params_.max_tension + cst_params_.min_tension);
	double ten_radi_ =
	    0.5 * fabs(cst_params_.max_tension - cst_params_.min_tension);
	double tension_pena_;
	// Maximum distance penalty parameters
	double weight_dis_ = cst_params_.weight_dis;
	double flag_dis_ = cst_params_.flag_dis;
	double dis_mean_ =
	    0.5 * (info_params_.cable_length + cst_params_.min_distance);
	double dis_radi_ =
	    0.5 * fabs(info_params_.cable_length - cst_params_.min_distance);
	double distance_pena_;
	// Dynamics equality constraint parameters
	double weight_dyn_ = cst_params_.weight_dyn;
	double flag_dyn_ = cst_params_.flag_dyn;
	double dyn_pena_;
	// Complementarity constraint parameters
	double weight_com_ = cst_params_.weight_com;
	double flag_com_ = cst_params_.flag_com;
	double com_pena_;
	// Tension cost parameters

	for (int i = 0; i < piece_nums; i++) {
		Eigen::MatrixXd c_;
		c_.resize(traj_order_ + 1, 6);
		c_ = coeffs.block(i * (traj_order_ + 1), 0, traj_order_ + 1, 6);

		step_ = T(i) * integra_frac_;
		for (int j = 0; j <= opt_params_.integral_resolution; j++) {
			s1 = j * step_;
			s2 = s1 * s1;
			s3 = s2 * s1;
			s4 = s3 * s1;
			s5 = s4 * s1;
			s6 = s5 * s1;
			s7 = s6 * s1;

			beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3,
			beta0(4) = s4, beta0(5) = s5, beta0(6) = s6,
			beta0(7) = s7; // beta0(8) = s8, beta0(9) = s9;
			beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1,
			beta1(3) = 3.0 * s2, beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4,
			beta1(6) = 6.0 * s5,
			beta1(7) = 7.0 * s6; // beta1(8) = 8.0 * s7, beta1(9) = 9.0 * s8;
			beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1,
			beta2(4) = 12.0 * s2, beta2(5) = 20.0 * s3, beta2(6) = 30.0 * s4,
			beta2(7) = 42.0 * s5; // beta2(8) = 56.0 * s6, beta2(9) = 72.0 * s7;
			beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0,
			beta3(4) = 24.0 * s1, beta3(5) = 60.0 * s2, beta3(6) = 120.0 * s3,
			beta3(7) =
			    210.0 * s4; // beta3(8) = 336.0 * s5, beta3(9) = 504.0 * s6;
			beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0,
			beta4(4) = 24.0, beta4(5) = 120.0 * s1, beta4(6) = 360.0 * s2,
			beta4(7) =
			    840.0 * s3; // beta4(8) = 1680.0 * s4, beta4(9) = 3024.0 * s5;
			beta5(0) = 0.0, beta5(1) = 0.0, beta5(2) = 0.0, beta5(3) = 0.0,
			beta5(4) = 0.0, beta5(5) = 120.0, beta5(6) = 720.0 * s1,
			beta5(7) =
			    2520.0 * s2; // beta5(8) = 6720.0 * s3, beta5(9) = 15120.0 * s4;
			beta6(0) = 0.0, beta6(1) = 0.0, beta6(2) = 0.0, beta6(3) = 0.0,
			beta6(4) = 0.0, beta6(5) = 0.0, beta6(6) = 720.0,
			beta6(7) = 5040.0 *
			           s1; // beta6(8) = 20160.0 * s2, beta6(9) = 60480.0 * s3;

			pos_ = c_.transpose() * beta0;
			vel_ = c_.transpose() * beta1;
			acc_ = c_.transpose() * beta2;
			jer_ = c_.transpose() * beta3;
			sna_ = c_.transpose() * beta4;
			cra_ = c_.transpose() * beta5;
			dcra_ = c_.transpose() * beta6;

			load_pos_ = pos_.head(3);
			load_vel_ = vel_.head(3);
			load_acc_ = acc_.head(3);
			load_jer_ = jer_.head(3);
			load_sna_ = sna_.head(3);
			load_cra_ = cra_.head(3);
			load_dcra_ = dcra_.head(3);

			quad_pos_ = pos_.tail(3);
			quad_vel_ = vel_.tail(3);
			quad_acc_ = acc_.tail(3);
			quad_jer_ = jer_.tail(3);
			quad_sna_ = sna_.tail(3);
			quad_cra_ = cra_.tail(3);
			quad_dcra_ = dcra_.tail(3);

			term_partial_load_pos_.setZero();
			term_partial_load_vel_.setZero();
			term_partial_load_acc_.setZero();
			term_partial_load_jer_.setZero();
			term_partial_quad_pos_.setZero();
			term_partial_quad_vel_.setZero();
			term_partial_quad_acc_.setZero();
			term_partial_quad_jer_.setZero();

			total_grad_load_pos_.setZero();
			total_grad_load_vel_.setZero();
			total_grad_load_acc_.setZero();
			total_grad_load_jer_.setZero();
			total_grad_quad_pos_.setZero();
			total_grad_quad_vel_.setZero();
			total_grad_quad_acc_.setZero();
			total_grad_quad_jer_.setZero();

			pena_ = 0.0;
			node_ =
			    (j == 0 || j == opt_params_.integral_resolution) ? 0.5 : 1.0;
			alpha_ = j * integra_frac_;

			flatmap_.forward(load_acc_, load_jer_, quad_acc_, quad_jer_, 0.0,
			                 0.0, thrust_, quat_, omg_);

			/* obstacle avoidance penalty for quadrotor */
			if (flag_obs_quad_ != 0) {
				inequalityConstraintObs(
				    opt_params_.scaling_wc(index), opt_params_.lambda(index),
				    opt_params_.rho, info_params_.safe_margin, quad_pos_,
				    opt_params_.cx(index), cts_partial_quad_pos_,
				    obs_pena_quad_, term_partial_quad_pos_);
				pena_ += weight_obs_quad_ * obs_pena_quad_;
				total_grad_quad_pos_ +=
				    weight_obs_quad_ * term_partial_quad_pos_;
				if (opt_params_.cx(index) > 0.0)
					debug_params_.cost_obs_quad += opt_params_.cx(index);
			}
			index += 1;

			/* obstacle avoidance penalty for payload */
			if (flag_obs_load_ != 0) {
				inequalityConstraintObs(
				    opt_params_.scaling_wc(index), opt_params_.lambda(index),
				    opt_params_.rho, info_params_.safe_margin, load_pos_,
				    opt_params_.cx(index), cts_partial_load_pos_,
				    obs_pena_load_, term_partial_load_pos_);
				pena_ += weight_obs_load_ * obs_pena_load_;
				total_grad_load_pos_ +=
				    weight_obs_load_ * term_partial_load_pos_;
				if (opt_params_.cx(index) > 0.0)
					debug_params_.cost_obs_load += opt_params_.cx(index);
			}
			index += 1;

			/* velocity feasibility constraint penalty of quadrotor */
			if (flag_vel_quad_ != 0) {
				inequalityConstraintVel(
				    opt_params_.scaling_wc(index), opt_params_.lambda(index),
				    opt_params_.rho, vel_sqr_bound_quad_, quad_vel_,
				    opt_params_.cx(index), cts_partial_quad_vel_,
				    vel_pena_quad_, term_partial_quad_vel_);
				pena_ += weight_vel_quad_ * vel_pena_quad_;
				total_grad_quad_vel_ +=
				    weight_vel_quad_ * term_partial_quad_vel_;
				if (opt_params_.cx(index) > 0.0)
					debug_params_.cost_vel += opt_params_.cx(index);
			}
			index += 1;

			/* accleration feasibility constraint penalty of quadrotor */
			if (flag_acc_quad_ != 0) {
				inequalityConstraintAcc(
				    opt_params_.scaling_wc(index), opt_params_.lambda(index),
				    opt_params_.rho, acc_sqr_bound_quad_, quad_acc_,
				    opt_params_.cx(index), cts_partial_quad_acc_,
				    acc_pena_quad_, term_partial_quad_acc_);
				pena_ += weight_acc_quad_ * acc_pena_quad_;
				total_grad_quad_acc_ +=
				    weight_acc_quad_ * term_partial_quad_acc_;
				if (opt_params_.cx(index) > 0.0)
					debug_params_.cost_acc += opt_params_.cx(index);
			}
			index += 1;

			/* accleration feasibility constraint penalty of quadrotor */
			if (flag_acc_load_ != 0) {
				inequalityConstraintAcc(
				    opt_params_.scaling_wc(index), opt_params_.lambda(index),
				    opt_params_.rho, acc_sqr_bound_load_, load_acc_,
				    opt_params_.cx(index), cts_partial_load_acc_,
				    acc_pena_load_, term_partial_load_acc_);
				pena_ += weight_acc_load_ * acc_pena_load_;
				total_grad_load_acc_ +=
				    weight_acc_load_ * term_partial_load_acc_;
				if (opt_params_.cx(index) > 0.0)
					debug_params_.cost_acc_load += opt_params_.cx(index);
			}
			index += 1;

			/* jerk feasibility constraint penalty of quadrotor */
			if (flag_jer_quad_ != 0) {
				inequalityConstraintJer(
				    opt_params_.scaling_wc(index), opt_params_.lambda(index),
				    opt_params_.rho, jer_sqr_bound_quad_, quad_jer_,
				    opt_params_.cx(index), cts_partial_quad_jer_,
				    jer_pena_quad_, term_partial_quad_jer_);
				pena_ += weight_jer_quad_ * jer_pena_quad_;
				total_grad_quad_jer_ +=
				    weight_jer_quad_ * term_partial_quad_jer_;
				if (opt_params_.cx(index) > 0.0)
					debug_params_.cost_jer += opt_params_.cx(index);
			}
			index += 1;

			/* body angle feasibility constraint penalty of quad */
			if (flag_theta_ != 0) {
				inequalityConstraintTheta(
				    opt_params_.scaling_wc(index), opt_params_.lambda(index),
				    opt_params_.rho, theta_bound_, quat_, opt_params_.cx(index),
				    cts_partial_load_acc_, cts_partial_load_jer_,
				    cts_partial_quad_acc_, cts_partial_quad_jer_, theta_pena_,
				    term_partial_load_acc_, term_partial_load_jer_,
				    term_partial_quad_acc_, term_partial_quad_jer_);
				pena_ += weight_theta_ * theta_pena_;
				total_grad_load_acc_ += weight_theta_ * term_partial_load_acc_;
				total_grad_load_jer_ += weight_theta_ * term_partial_load_jer_;
				total_grad_quad_acc_ += weight_theta_ * term_partial_quad_acc_;
				total_grad_quad_jer_ += weight_theta_ * term_partial_quad_jer_;
				if (opt_params_.cx(index) > 0.0)
					debug_params_.cost_theta += opt_params_.cx(index);
			}
			index += 1;

			/* thrust feasibility constraint penalty of quad */
			if (flag_thr_ != 0) {
				inequalityConstraintThrust(
				    opt_params_.scaling_wc(index), opt_params_.lambda(index),
				    opt_params_.rho, thr_mean_, thr_radi_, thrust_,
				    opt_params_.cx(index), cts_partial_load_acc_,
				    cts_partial_load_jer_, cts_partial_quad_acc_,
				    cts_partial_quad_jer_, thr_pena_, term_partial_load_acc_,
				    term_partial_load_jer_, term_partial_quad_acc_,
				    term_partial_quad_jer_);
				pena_ += weight_thr_ * thr_pena_;
				total_grad_load_acc_ += weight_thr_ * term_partial_load_acc_;
				total_grad_load_jer_ += weight_thr_ * term_partial_load_jer_;
				total_grad_quad_acc_ += weight_thr_ * term_partial_quad_acc_;
				total_grad_quad_jer_ += weight_thr_ * term_partial_quad_jer_;
				if (opt_params_.cx(index) > 0.0)
					debug_params_.cost_thrust += opt_params_.cx(index);
			}
			index += 1;

			/* body rate feasibility constraint penalty of quad */
			if (flag_omg_ != 0) {
				inequalityConstraintOmg(
				    opt_params_.scaling_wc(index), opt_params_.lambda(index),
				    opt_params_.rho, omg_bound_, omg_, opt_params_.cx(index),
				    cts_partial_load_acc_, cts_partial_load_jer_,
				    cts_partial_quad_acc_, cts_partial_quad_jer_, omg_pena_,
				    term_partial_load_acc_, term_partial_load_jer_,
				    term_partial_quad_acc_, term_partial_quad_jer_);
				pena_ += weight_omg_ * omg_pena_;
				total_grad_load_acc_ += weight_omg_ * term_partial_load_acc_;
				total_grad_load_jer_ += weight_omg_ * term_partial_load_jer_;
				total_grad_quad_acc_ += weight_omg_ * term_partial_quad_acc_;
				total_grad_quad_jer_ += weight_omg_ * term_partial_quad_jer_;
				if (opt_params_.cx(index) > 0.0)
					debug_params_.cost_omg += opt_params_.cx(index);
			}
			index += 1;

			/* tension feasibility constraint penalty */
			if (flag_ten_ != 0) {
				inequalityConstraintTen(
				    opt_params_.scaling_wc(index), opt_params_.lambda(index),
				    opt_params_.rho, ten_mean_, ten_radi_, load_acc_,
				    opt_params_.cx(index), cts_partial_load_acc_, tension_pena_,
				    term_partial_load_acc_);
				pena_ += weight_ten_ * tension_pena_;
				total_grad_load_acc_ += weight_ten_ * term_partial_load_acc_;
				if (opt_params_.cx(index) > 0.0)
					debug_params_.cost_tension += opt_params_.cx(index);
			}
			index += 1;

			/* distance feasibility constraint penalty */
			if (flag_dis_ != 0) {
				inequalityConstraintDis(
				    opt_params_.scaling_wc(index), opt_params_.lambda(index),
				    opt_params_.rho, dis_mean_, dis_radi_, load_pos_, quad_pos_,
				    opt_params_.cx(index), cts_partial_load_pos_,
				    cts_partial_quad_pos_, distance_pena_,
				    term_partial_load_pos_, term_partial_quad_pos_);
				pena_ += weight_dis_ * distance_pena_;
				total_grad_load_pos_ += weight_dis_ * term_partial_load_pos_;
				total_grad_quad_pos_ += weight_dis_ * term_partial_quad_pos_;
				if (opt_params_.cx(index) > 0.0)
					debug_params_.cost_dist += opt_params_.cx(index);
			}
			index += 1;

			/* dynamic equality constraint penalty */
			if (flag_dyn_ != 0) {
				equalityConstraintDyn(
				    opt_params_.scaling_wc(index), opt_params_.lambda(index),
				    opt_params_.rho, load_pos_, load_acc_, quad_pos_,
				    opt_params_.cx(index), cts_partial_load_pos_,
				    cts_partial_load_acc_, cts_partial_quad_pos_, dyn_pena_,
				    term_partial_load_pos_, term_partial_load_acc_,
				    term_partial_quad_pos_);
				pena_ += weight_dyn_ * dyn_pena_;
				total_grad_load_pos_ += weight_dyn_ * term_partial_load_pos_;
				total_grad_load_acc_ += weight_dyn_ * term_partial_load_acc_;
				total_grad_quad_pos_ += weight_dyn_ * term_partial_quad_pos_;
				debug_params_.cost_dyn += opt_params_.cx(index);
			}
			index += 1;

			/* complementary equality constraint penalty */
			if (flag_com_ != 0) {
				equalityConstraintCom(
				    opt_params_.scaling_wc(index), opt_params_.lambda(index),
				    opt_params_.rho, load_pos_, load_acc_, quad_pos_,
				    opt_params_.cx(index), cts_partial_load_pos_,
				    cts_partial_load_acc_, cts_partial_quad_pos_, com_pena_,
				    term_partial_load_pos_, term_partial_load_acc_,
				    term_partial_quad_pos_);
				pena_ += weight_com_ * com_pena_;
				total_grad_load_pos_ += weight_com_ * term_partial_load_pos_;
				total_grad_quad_pos_ += weight_com_ * term_partial_quad_pos_;
				total_grad_load_acc_ += weight_com_ * term_partial_load_acc_;
				debug_params_.cost_comp += opt_params_.cx(index);
			}
			index += 1;

			// calculate totoal grad
			gradC.block(i * (traj_order_ + 1), 0, traj_order_ + 1, 3) +=
			    (beta0 * total_grad_load_pos_.transpose() +
			     beta1 * total_grad_load_vel_.transpose() +
			     beta2 * total_grad_load_acc_.transpose() +
			     beta3 * total_grad_load_jer_.transpose()) *
			    node_ * step_;
			gradC.block(i * (traj_order_ + 1), 3, traj_order_ + 1, 3) +=
			    (beta0 * total_grad_quad_pos_.transpose() +
			     beta1 * total_grad_quad_vel_.transpose() +
			     beta2 * total_grad_quad_acc_.transpose() +
			     beta3 * total_grad_quad_jer_.transpose()) *
			    node_ * step_;
			gradT(i) += (total_grad_load_pos_.dot(load_vel_) +
			             total_grad_load_vel_.dot(load_acc_) +
			             total_grad_load_acc_.dot(load_jer_) +
			             total_grad_load_jer_.dot(load_sna_) +
			             total_grad_quad_pos_.dot(quad_vel_) +
			             total_grad_quad_vel_.dot(quad_acc_) +
			             total_grad_quad_acc_.dot(quad_jer_) +
			             total_grad_quad_jer_.dot(quad_sna_)) *
			            alpha_ * node_ * step_;
			cost += pena_ * node_ * step_;
		}
	}
}

void HybridOPT::augmentedLagrangianFunctional(
    const Eigen::VectorXd& T, const Eigen::MatrixXd& coeffs,
    const int piece_nums, int& index, double& cost, Eigen::VectorXd& gradT,
    Eigen::MatrixXd& gradC, Eigen::MatrixXd& grad_cts) {
	// System info
	Eigen::VectorXd pos_, vel_, acc_, jer_, sna_, cra_, dcra_;
	Eigen::Vector3d load_pos_, load_vel_, load_acc_, load_jer_, load_sna_,
	    load_cra_, load_dcra_;
	Eigen::Vector3d quad_pos_, quad_vel_, quad_acc_, quad_jer_, quad_sna_,
	    quad_cra_, quad_dcra_;
	Eigen::Vector3d omg_;
	Eigen::Vector4d quat_;
	double thrust_;
	double step_, alpha_;
	double s1, s2, s3, s4, s5, s6, s7;
	Eigen::Matrix<double, 8, 1> beta0, beta1, beta2, beta3, beta4, beta5, beta6;
	const double integra_frac_ = 1.0 / opt_params_.integral_resolution;
	const double traj_order_ = info_params_.traj_order;

	// Gradience
	Eigen::Vector3d term_partial_quad_pos_, term_partial_quad_vel_,
	    term_partial_quad_acc_, term_partial_quad_jer_;
	Eigen::Vector3d term_partial_load_pos_, term_partial_load_vel_,
	    term_partial_load_acc_, term_partial_load_jer_;

	Eigen::Vector3d total_grad_quad_pos_, total_grad_quad_vel_,
	    total_grad_quad_acc_, total_grad_quad_jer_;
	Eigen::Vector3d total_grad_load_pos_, total_grad_load_vel_,
	    total_grad_load_acc_, total_grad_load_jer_;

	Eigen::Vector3d cts_partial_quad_pos_, cts_partial_quad_vel_,
	    cts_partial_quad_acc_, cts_partial_quad_jer_;
	Eigen::Vector3d cts_partial_load_pos_, cts_partial_load_vel_,
	    cts_partial_load_acc_, cts_partial_load_jer_;

	Eigen::VectorXd partial_T_(piece_nums);
	Eigen::MatrixXd partial_coeffs_((info_params_.traj_order + 1) * piece_nums,
	                                info_params_.state_dim);

	Eigen::MatrixXd grad_inner_pts_;
	Eigen::VectorXd grad_durations_, grad_tau_;

	// Cost
	double node_, pena_;

	// Obstacle avoidance parameters
	double flag_obs_quad_ = cst_params_.flag_obs;
	double obs_pena_quad_;

	double flag_obs_load_ = cst_params_.flag_obs;
	double obs_pena_load_;
	// Maximum velocity of quadrotor penalty parameters
	double flag_vel_quad_ = cst_params_.flag_vel_quad;
	double vel_sqr_bound_quad_ =
	    cst_params_.max_vel_quad * cst_params_.max_vel_quad;
	double vel_pena_quad_;
	// Maximum acceleration of quadrotor penalty parameters
	double flag_acc_quad_ = cst_params_.flag_acc_quad;
	double acc_sqr_bound_quad_ =
	    cst_params_.max_acc_quad * cst_params_.max_acc_quad;
	double acc_pena_quad_;
	// Maximum acceleration of quadrotor penalty parameters
	double flag_acc_load_ = cst_params_.flag_acc_load;
	double acc_sqr_bound_load_ =
	    cst_params_.max_acc_load * cst_params_.max_acc_load;
	double acc_pena_load_;
	// Maximum jerk of quadrotor penalty parameters
	double flag_jer_quad_ = cst_params_.flag_jer_quad;
	double jer_sqr_bound_quad_ =
	    cst_params_.max_jer_quad * cst_params_.max_jer_quad;
	double jer_pena_quad_;
	// Maximum body angle of quadrotor penalty parameters
	double flag_theta_ = cst_params_.flag_theta;
	double theta_bound_ = cst_params_.max_theta;
	double theta_pena_;
	// Maximum body rate of quadrotor penalty parameters
	double flag_omg_ = cst_params_.flag_omg;
	double omg_bound_ = cst_params_.max_omg;
	double omg_pena_;
	// Maximum thrust of quadrotor penalty parameters
	double flag_thr_ = cst_params_.flag_thr;
	double thr_mean_ = 0.5 * (cst_params_.max_thr + cst_params_.min_thr);
	double thr_radi_ = 0.5 * (cst_params_.max_thr - cst_params_.min_thr);
	double thr_pena_;
	// Maximum tension penalty parameters
	double flag_ten_ = cst_params_.flag_ten;
	double ten_mean_ =
	    0.5 * (cst_params_.max_tension + cst_params_.min_tension);
	double ten_radi_ =
	    0.5 * fabs(cst_params_.max_tension - cst_params_.min_tension);
	double tension_pena_;
	// Maximum distance penalty parameters
	double flag_dis_ = cst_params_.flag_dis;
	double dis_mean_ =
	    0.5 * (info_params_.cable_length + cst_params_.min_distance);
	double dis_radi_ =
	    0.5 * fabs(info_params_.cable_length - cst_params_.min_distance);
	double distance_pena_;
	// Dynamics equality constraint parameters
	double flag_dyn_ = cst_params_.flag_dyn;
	double dyn_pena_;
	// Complementarity constraint parameters
	double flag_com_ = cst_params_.flag_com;
	double com_pena_;

	std::cout << piece_nums << std::endl;

	for (int i = 0; i < piece_nums; i++) {
		Eigen::MatrixXd c_;
		c_.resize(traj_order_ + 1, 6);
		c_ = coeffs.block(i * (traj_order_ + 1), 0, traj_order_ + 1, 6);

		step_ = T(i) * integra_frac_;
		for (int j = 0; j <= opt_params_.integral_resolution; j++) {
			s1 = j * step_;
			s2 = s1 * s1;
			s3 = s2 * s1;
			s4 = s3 * s1;
			s5 = s4 * s1;
			s6 = s5 * s1;
			s7 = s6 * s1;

			beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3,
			beta0(4) = s4, beta0(5) = s5, beta0(6) = s6,
			beta0(7) = s7; // beta0(8) = s8, beta0(9) = s9;
			beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1,
			beta1(3) = 3.0 * s2, beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4,
			beta1(6) = 6.0 * s5,
			beta1(7) = 7.0 * s6; // beta1(8) = 8.0 * s7, beta1(9) = 9.0 * s8;
			beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1,
			beta2(4) = 12.0 * s2, beta2(5) = 20.0 * s3, beta2(6) = 30.0 * s4,
			beta2(7) = 42.0 * s5; // beta2(8) = 56.0 * s6, beta2(9) = 72.0 * s7;
			beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0,
			beta3(4) = 24.0 * s1, beta3(5) = 60.0 * s2, beta3(6) = 120.0 * s3,
			beta3(7) =
			    210.0 * s4; // beta3(8) = 336.0 * s5, beta3(9) = 504.0 * s6;
			beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0,
			beta4(4) = 24.0, beta4(5) = 120.0 * s1, beta4(6) = 360.0 * s2,
			beta4(7) =
			    840.0 * s3; // beta4(8) = 1680.0 * s4, beta4(9) = 3024.0 * s5;
			beta5(0) = 0.0, beta5(1) = 0.0, beta5(2) = 0.0, beta5(3) = 0.0,
			beta5(4) = 0.0, beta5(5) = 120.0, beta5(6) = 720.0 * s1,
			beta5(7) =
			    2520.0 * s2; // beta5(8) = 6720.0 * s3, beta5(9) = 15120.0 * s4;
			beta6(0) = 0.0, beta6(1) = 0.0, beta6(2) = 0.0, beta6(3) = 0.0,
			beta6(4) = 0.0, beta6(5) = 0.0, beta6(6) = 720.0,
			beta6(7) = 5040.0 *
			           s1; // beta6(8) = 20160.0 * s2, beta6(9) = 60480.0 * s3;

			pos_ = c_.transpose() * beta0;
			vel_ = c_.transpose() * beta1;
			acc_ = c_.transpose() * beta2;
			jer_ = c_.transpose() * beta3;
			sna_ = c_.transpose() * beta4;
			cra_ = c_.transpose() * beta5;
			dcra_ = c_.transpose() * beta6;

			load_pos_ = pos_.head(3);
			load_vel_ = vel_.head(3);
			load_acc_ = acc_.head(3);
			load_jer_ = jer_.head(3);
			load_sna_ = sna_.head(3);
			load_cra_ = cra_.head(3);
			load_dcra_ = dcra_.head(3);

			quad_pos_ = pos_.tail(3);
			quad_vel_ = vel_.tail(3);
			quad_acc_ = acc_.tail(3);
			quad_jer_ = jer_.tail(3);
			quad_sna_ = sna_.tail(3);
			quad_cra_ = cra_.tail(3);
			quad_dcra_ = dcra_.tail(3);

			term_partial_load_pos_.setZero();
			term_partial_load_vel_.setZero();
			term_partial_load_acc_.setZero();
			term_partial_load_jer_.setZero();
			term_partial_quad_pos_.setZero();
			term_partial_quad_vel_.setZero();
			term_partial_quad_acc_.setZero();
			term_partial_quad_jer_.setZero();

			total_grad_load_pos_.setZero();
			total_grad_load_vel_.setZero();
			total_grad_load_acc_.setZero();
			total_grad_load_jer_.setZero();
			total_grad_quad_pos_.setZero();
			total_grad_quad_vel_.setZero();
			total_grad_quad_acc_.setZero();
			total_grad_quad_jer_.setZero();

			pena_ = 0.0;
			node_ =
			    (j == 0 || j == opt_params_.integral_resolution) ? 0.5 : 1.0;
			alpha_ = j * integra_frac_;

			flatmap_.forward(load_acc_, load_jer_, quad_acc_, quad_jer_, 0.0,
			                 0.0, thrust_, quat_, omg_);
			/* obstacle avoidance penalty for quadrotor */
			opt_params_.cx_flag(index) = 1;
			if (flag_obs_quad_ != 0) {
				inequalityConstraintObs(
				    opt_params_.scaling_wc(index), opt_params_.lambda(index),
				    opt_params_.rho, info_params_.safe_margin, quad_pos_,
				    opt_params_.cx(index), cts_partial_quad_pos_,
				    obs_pena_quad_, term_partial_quad_pos_);
				pena_ += obs_pena_quad_;
				total_grad_quad_pos_ += term_partial_quad_pos_;
				if (opt_params_.cx(index) > 0.0)
					debug_params_.init_cost_obs_quad += opt_params_.cx(index);
			}

			// calculate the grad of constraint violation
			if (cts_partial_quad_pos_.norm() < 1e-8 || flag_obs_quad_ == 0) {
				grad_cts.row(index).setZero();
			} else {
				partial_T_.setZero();
				partial_coeffs_.setZero();
				get_grad_CT(node_, alpha_, step_, i, info_params_.traj_order,
				            beta0, beta1, beta2, beta3, load_vel_, load_acc_,
				            load_jer_, load_sna_, quad_vel_, quad_acc_,
				            quad_jer_, quad_sna_, Eigen::Vector3d::Zero(),
				            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
				            Eigen::Vector3d::Zero(), cts_partial_quad_pos_,
				            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
				            Eigen::Vector3d::Zero(), partial_T_,
				            partial_coeffs_);
				info_params_.minco->propogateGrad(partial_coeffs_, partial_T_,
				                                  grad_inner_pts_,
				                                  grad_durations_);
				backwardGradT(T, grad_durations_, grad_tau_);
				Eigen::Map<Eigen::VectorXd> grad_inner_pts_vec(
				    grad_inner_pts_.data(), grad_inner_pts_.size());
				grad_cts.row(index).head(grad_inner_pts_.size()) =
				    grad_inner_pts_vec;
				grad_cts.row(index).tail(grad_tau_.size()) = grad_tau_;
			}
			index += 1;

			/* obstacle avoidance penalty for payload */
			opt_params_.cx_flag(index) = 1;
			if (flag_obs_load_ != 0) {
				inequalityConstraintObs(
				    opt_params_.scaling_wc(index), opt_params_.lambda(index),
				    opt_params_.rho, info_params_.safe_margin, load_pos_,
				    opt_params_.cx(index), cts_partial_load_pos_,
				    obs_pena_load_, term_partial_load_pos_);
				pena_ += obs_pena_load_;
				total_grad_load_pos_ += term_partial_load_pos_;
				if (opt_params_.cx(index) > 0.0)
					debug_params_.init_cost_obs_load += opt_params_.cx(index);
			}
			// calculate the grad of constraint violation
			if (cts_partial_load_pos_.norm() < 1e-8 || flag_obs_load_ == 0) {
				grad_cts.row(index).setZero();
			} else {
				partial_T_.setZero();
				partial_coeffs_.setZero();
				get_grad_CT(node_, alpha_, step_, i, info_params_.traj_order,
				            beta0, beta1, beta2, beta3, load_vel_, load_acc_,
				            load_jer_, load_sna_, quad_vel_, quad_acc_,
				            quad_jer_, quad_sna_, cts_partial_load_pos_,
				            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
				            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
				            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
				            Eigen::Vector3d::Zero(), partial_T_,
				            partial_coeffs_);
				info_params_.minco->propogateGrad(partial_coeffs_, partial_T_,
				                                  grad_inner_pts_,
				                                  grad_durations_);
				backwardGradT(T, grad_durations_, grad_tau_);
				Eigen::Map<Eigen::VectorXd> grad_inner_pts_vec(
				    grad_inner_pts_.data(), grad_inner_pts_.size());
				grad_cts.row(index).head(grad_inner_pts_.size()) =
				    grad_inner_pts_vec;
				grad_cts.row(index).tail(grad_tau_.size()) = grad_tau_;
			}
			index += 1;

			/* velocity feasibility constraint penalty of quadrotor */
			opt_params_.cx_flag(index) = 1;
			if (flag_vel_quad_ != 0) {
				inequalityConstraintVel(
				    opt_params_.scaling_wc(index), opt_params_.lambda(index),
				    opt_params_.rho, vel_sqr_bound_quad_, quad_vel_,
				    opt_params_.cx(index), cts_partial_quad_vel_,
				    vel_pena_quad_, term_partial_quad_vel_);
				pena_ += vel_pena_quad_;
				total_grad_quad_vel_ += term_partial_quad_vel_;
				if (opt_params_.cx(index) > 0.0)
					debug_params_.init_cost_vel += opt_params_.cx(index);
			}
			// calculate the grad of constraint violation
			if (cts_partial_quad_vel_.norm() < 1e-8 || flag_vel_quad_ == 0) {
				grad_cts.row(index).setZero();
			} else {
				partial_T_.setZero();
				partial_coeffs_.setZero();
				get_grad_CT(node_, alpha_, step_, i, info_params_.traj_order,
				            beta0, beta1, beta2, beta3, load_vel_, load_acc_,
				            load_jer_, load_sna_, quad_vel_, quad_acc_,
				            quad_jer_, quad_sna_, Eigen::Vector3d::Zero(),
				            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
				            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
				            cts_partial_quad_vel_, Eigen::Vector3d::Zero(),
				            Eigen::Vector3d::Zero(), partial_T_,
				            partial_coeffs_);
				info_params_.minco->propogateGrad(partial_coeffs_, partial_T_,
				                                  grad_inner_pts_,
				                                  grad_durations_);
				backwardGradT(T, grad_durations_, grad_tau_);
				Eigen::Map<Eigen::VectorXd> grad_inner_pts_vec(
				    grad_inner_pts_.data(), grad_inner_pts_.size());
				grad_cts.row(index).head(grad_inner_pts_.size()) =
				    grad_inner_pts_vec;
				grad_cts.row(index).tail(grad_tau_.size()) = grad_tau_;
			}
			index += 1;

			/* accleration feasibility constraint penalty of quadrotor */
			opt_params_.cx_flag(index) = 1;
			if (flag_acc_quad_ != 0) {
				inequalityConstraintAcc(
				    opt_params_.scaling_wc(index), opt_params_.lambda(index),
				    opt_params_.rho, acc_sqr_bound_quad_, quad_acc_,
				    opt_params_.cx(index), cts_partial_quad_acc_,
				    acc_pena_quad_, term_partial_quad_acc_);
				pena_ += acc_pena_quad_;
				total_grad_quad_acc_ += term_partial_quad_acc_;
				if (opt_params_.cx(index) > 0.0)
					debug_params_.init_cost_acc += opt_params_.cx(index);
			}
			// calculate the grad of constraint violation
			if (cts_partial_quad_acc_.norm() < 1e-8 || flag_acc_quad_ == 0) {
				grad_cts.row(index).setZero();
			} else {
				partial_T_.setZero();
				partial_coeffs_.setZero();
				get_grad_CT(node_, alpha_, step_, i, info_params_.traj_order,
				            beta0, beta1, beta2, beta3, load_vel_, load_acc_,
				            load_jer_, load_sna_, quad_vel_, quad_acc_,
				            quad_jer_, quad_sna_, Eigen::Vector3d::Zero(),
				            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
				            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
				            Eigen::Vector3d::Zero(), cts_partial_quad_acc_,
				            Eigen::Vector3d::Zero(), partial_T_,
				            partial_coeffs_);
				info_params_.minco->propogateGrad(partial_coeffs_, partial_T_,
				                                  grad_inner_pts_,
				                                  grad_durations_);
				backwardGradT(T, grad_durations_, grad_tau_);
				Eigen::Map<Eigen::VectorXd> grad_inner_pts_vec(
				    grad_inner_pts_.data(), grad_inner_pts_.size());
				grad_cts.row(index).head(grad_inner_pts_.size()) =
				    grad_inner_pts_vec;
				grad_cts.row(index).tail(grad_tau_.size()) = grad_tau_;
			}
			index += 1;

			/* accleration feasibility constraint penalty of quadrotor */
			opt_params_.cx_flag(index) = 1;
			if (flag_acc_load_ != 0) {
				inequalityConstraintAcc(
				    opt_params_.scaling_wc(index), opt_params_.lambda(index),
				    opt_params_.rho, acc_sqr_bound_load_, load_acc_,
				    opt_params_.cx(index), cts_partial_load_acc_,
				    acc_pena_load_, term_partial_load_acc_);
				pena_ += acc_pena_load_;
				total_grad_load_acc_ += term_partial_load_acc_;
				if (opt_params_.cx(index) > 0.0)
					debug_params_.init_cost_acc += opt_params_.cx(index);
			}
			// calculate the grad of constraint violation
			if (cts_partial_load_acc_.norm() < 1e-8 || flag_acc_load_ == 0) {
				grad_cts.row(index).setZero();
			} else {
				partial_T_.setZero();
				partial_coeffs_.setZero();
				get_grad_CT(node_, alpha_, step_, i, info_params_.traj_order,
				            beta0, beta1, beta2, beta3, load_vel_, load_acc_,
				            load_jer_, load_sna_, quad_vel_, quad_acc_,
				            quad_jer_, quad_sna_, Eigen::Vector3d::Zero(),
				            Eigen::Vector3d::Zero(), cts_partial_load_acc_,
				            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
				            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
				            Eigen::Vector3d::Zero(), partial_T_,
				            partial_coeffs_);
				info_params_.minco->propogateGrad(partial_coeffs_, partial_T_,
				                                  grad_inner_pts_,
				                                  grad_durations_);
				backwardGradT(T, grad_durations_, grad_tau_);
				Eigen::Map<Eigen::VectorXd> grad_inner_pts_vec(
				    grad_inner_pts_.data(), grad_inner_pts_.size());
				grad_cts.row(index).head(grad_inner_pts_.size()) =
				    grad_inner_pts_vec;
				grad_cts.row(index).tail(grad_tau_.size()) = grad_tau_;
			}
			index += 1;

			/* jerk feasibility constraint penalty of quadrotor */
			opt_params_.cx_flag(index) = 1;
			if (flag_jer_quad_ != 0) {
				inequalityConstraintJer(
				    opt_params_.scaling_wc(index), opt_params_.lambda(index),
				    opt_params_.rho, jer_sqr_bound_quad_, quad_jer_,
				    opt_params_.cx(index), cts_partial_quad_jer_,
				    jer_pena_quad_, term_partial_quad_jer_);
				pena_ += jer_pena_quad_;
				total_grad_quad_jer_ += term_partial_quad_jer_;
				if (opt_params_.cx(index) > 0.0)
					debug_params_.init_cost_jer += opt_params_.cx(index);
			}
			// calculate the grad of constraint violation
			if (cts_partial_quad_jer_.norm() < 1e-8 || flag_jer_quad_ == 0) {
				grad_cts.row(index).setZero();
			} else {
				partial_T_.setZero();
				partial_coeffs_.setZero();
				get_grad_CT(node_, alpha_, step_, i, info_params_.traj_order,
				            beta0, beta1, beta2, beta3, load_vel_, load_acc_,
				            load_jer_, load_sna_, quad_vel_, quad_acc_,
				            quad_jer_, quad_sna_, Eigen::Vector3d::Zero(),
				            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
				            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
				            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
				            cts_partial_quad_jer_, partial_T_, partial_coeffs_);
				info_params_.minco->propogateGrad(partial_coeffs_, partial_T_,
				                                  grad_inner_pts_,
				                                  grad_durations_);
				backwardGradT(T, grad_durations_, grad_tau_);
				Eigen::Map<Eigen::VectorXd> grad_inner_pts_vec(
				    grad_inner_pts_.data(), grad_inner_pts_.size());
				grad_cts.row(index).head(grad_inner_pts_.size()) =
				    grad_inner_pts_vec;
				grad_cts.row(index).tail(grad_tau_.size()) = grad_tau_;
			}
			index += 1;

			/* theta feasibility constraint penalty of payload */
			opt_params_.cx_flag(index) = 1;
			if (flag_theta_ != 0) {
				inequalityConstraintTheta(
				    opt_params_.scaling_wc(index), opt_params_.lambda(index),
				    opt_params_.rho, theta_bound_, quat_, opt_params_.cx(index),
				    cts_partial_load_acc_, cts_partial_load_jer_,
				    cts_partial_quad_acc_, cts_partial_quad_jer_, theta_pena_,
				    term_partial_load_acc_, term_partial_load_jer_,
				    term_partial_quad_acc_, term_partial_quad_jer_);
				pena_ += theta_pena_;
				total_grad_load_acc_ += term_partial_load_acc_;
				total_grad_load_jer_ += term_partial_load_jer_;
				total_grad_quad_acc_ += term_partial_quad_acc_;
				total_grad_quad_jer_ += term_partial_quad_jer_;
				if (opt_params_.cx(index) > 0.0)
					debug_params_.init_cost_theta += opt_params_.cx(index);
			}
			// calculate the grad of constraint violation
			if ((cts_partial_load_acc_.norm() < 1e-8 &&
			     cts_partial_load_jer_.norm() < 1e-8 &&
			     cts_partial_quad_acc_.norm() < 1e-8 &&
			     cts_partial_quad_jer_.norm() < 1e-8) ||
			    flag_theta_ == 0) {
				grad_cts.row(index).setZero();
			} else {
				partial_T_.setZero();
				partial_coeffs_.setZero();
				get_grad_CT(node_, alpha_, step_, i, info_params_.traj_order,
				            beta0, beta1, beta2, beta3, load_vel_, load_acc_,
				            load_jer_, load_sna_, quad_vel_, quad_acc_,
				            quad_jer_, quad_sna_, Eigen::Vector3d::Zero(),
				            Eigen::Vector3d::Zero(), cts_partial_load_acc_,
				            cts_partial_load_jer_, Eigen::Vector3d::Zero(),
				            Eigen::Vector3d::Zero(), cts_partial_quad_acc_,
				            cts_partial_quad_jer_, partial_T_, partial_coeffs_);
				info_params_.minco->propogateGrad(partial_coeffs_, partial_T_,
				                                  grad_inner_pts_,
				                                  grad_durations_);
				backwardGradT(T, grad_durations_, grad_tau_);
				Eigen::Map<Eigen::VectorXd> grad_inner_pts_vec(
				    grad_inner_pts_.data(), grad_inner_pts_.size());
				grad_cts.row(index).head(grad_inner_pts_.size()) =
				    grad_inner_pts_vec;
				grad_cts.row(index).tail(grad_tau_.size()) = grad_tau_;
			}
			index += 1;

			/* pitch feasibility constraint penalty of quad */
			opt_params_.cx_flag(index) = 1;
			if (flag_thr_ != 0) {
				inequalityConstraintThrust(
				    opt_params_.scaling_wc(index), opt_params_.lambda(index),
				    opt_params_.rho, thr_mean_, thr_radi_, thrust_,
				    opt_params_.cx(index), cts_partial_load_acc_,
				    cts_partial_load_jer_, cts_partial_quad_acc_,
				    cts_partial_quad_jer_, thr_pena_, term_partial_load_acc_,
				    term_partial_load_jer_, term_partial_quad_acc_,
				    term_partial_quad_jer_);
				pena_ += thr_pena_;
				total_grad_load_acc_ += term_partial_load_acc_;
				total_grad_load_jer_ += term_partial_load_jer_;
				total_grad_quad_acc_ += term_partial_quad_acc_;
				total_grad_quad_jer_ += term_partial_quad_jer_;
				if (opt_params_.cx(index) > 0.0)
					debug_params_.init_cost_thrust += opt_params_.cx(index);
			}
			// calculate the grad of constraint violation
			if ((cts_partial_load_acc_.norm() < 1e-8 &&
			     cts_partial_load_jer_.norm() < 1e-8 &&
			     cts_partial_quad_acc_.norm() < 1e-8 &&
			     cts_partial_quad_jer_.norm() < 1e-8) ||
			    flag_thr_ == 0) {
				grad_cts.row(index).setZero();
			} else {
				partial_T_.setZero();
				partial_coeffs_.setZero();
				get_grad_CT(node_, alpha_, step_, i, info_params_.traj_order,
				            beta0, beta1, beta2, beta3, load_vel_, load_acc_,
				            load_jer_, load_sna_, quad_vel_, quad_acc_,
				            quad_jer_, quad_sna_, Eigen::Vector3d::Zero(),
				            Eigen::Vector3d::Zero(), cts_partial_load_acc_,
				            cts_partial_load_jer_, Eigen::Vector3d::Zero(),
				            Eigen::Vector3d::Zero(), cts_partial_quad_acc_,
				            cts_partial_quad_jer_, partial_T_, partial_coeffs_);
				info_params_.minco->propogateGrad(partial_coeffs_, partial_T_,
				                                  grad_inner_pts_,
				                                  grad_durations_);
				backwardGradT(T, grad_durations_, grad_tau_);
				Eigen::Map<Eigen::VectorXd> grad_inner_pts_vec(
				    grad_inner_pts_.data(), grad_inner_pts_.size());
				grad_cts.row(index).head(grad_inner_pts_.size()) =
				    grad_inner_pts_vec;
				grad_cts.row(index).tail(grad_tau_.size()) = grad_tau_;
			}
			index += 1;

			/* omg feasibility constraint penalty of quad */
			opt_params_.cx_flag(index) = 1;
			if (flag_omg_ != 0) {
				inequalityConstraintOmg(
				    opt_params_.scaling_wc(index), opt_params_.lambda(index),
				    opt_params_.rho, omg_bound_, omg_, opt_params_.cx(index),
				    cts_partial_load_acc_, cts_partial_load_jer_,
				    cts_partial_quad_acc_, cts_partial_quad_jer_, omg_pena_,
				    term_partial_load_acc_, term_partial_load_jer_,
				    term_partial_quad_acc_, term_partial_quad_jer_);
				pena_ += omg_pena_;
				total_grad_load_acc_ += term_partial_load_acc_;
				total_grad_load_jer_ += term_partial_load_jer_;
				total_grad_quad_acc_ += term_partial_quad_acc_;
				total_grad_quad_jer_ += term_partial_quad_jer_;
				if (opt_params_.cx(index) > 0.0)
					debug_params_.init_cost_omg += opt_params_.cx(index);
			}
			// calculate the grad of constraint violation
			if ((cts_partial_load_acc_.norm() < 1e-8 &&
			     cts_partial_load_jer_.norm() < 1e-8 &&
			     cts_partial_quad_acc_.norm() < 1e-8 &&
			     cts_partial_quad_jer_.norm() < 1e-8) ||
			    flag_omg_ == 0) {
				grad_cts.row(index).setZero();
			} else {
				partial_T_.setZero();
				partial_coeffs_.setZero();
				get_grad_CT(node_, alpha_, step_, i, info_params_.traj_order,
				            beta0, beta1, beta2, beta3, load_vel_, load_acc_,
				            load_jer_, load_sna_, quad_vel_, quad_acc_,
				            quad_jer_, quad_sna_, Eigen::Vector3d::Zero(),
				            Eigen::Vector3d::Zero(), cts_partial_load_acc_,
				            cts_partial_load_jer_, Eigen::Vector3d::Zero(),
				            Eigen::Vector3d::Zero(), cts_partial_quad_acc_,
				            cts_partial_quad_jer_, partial_T_, partial_coeffs_);
				info_params_.minco->propogateGrad(partial_coeffs_, partial_T_,
				                                  grad_inner_pts_,
				                                  grad_durations_);
				backwardGradT(T, grad_durations_, grad_tau_);
				Eigen::Map<Eigen::VectorXd> grad_inner_pts_vec(
				    grad_inner_pts_.data(), grad_inner_pts_.size());
				grad_cts.row(index).head(grad_inner_pts_.size()) =
				    grad_inner_pts_vec;
				grad_cts.row(index).tail(grad_tau_.size()) = grad_tau_;
			}
			index += 1;

			/* tension up bound feasibility constraint penalty */
			opt_params_.cx_flag(index) = 1;
			if (flag_ten_ != 0) {
				inequalityConstraintTen(
				    opt_params_.scaling_wc(index), opt_params_.lambda(index),
				    opt_params_.rho, ten_mean_, ten_radi_, load_acc_,
				    opt_params_.cx(index), cts_partial_load_acc_, tension_pena_,
				    term_partial_load_acc_);
				pena_ += tension_pena_;
				total_grad_load_acc_ += term_partial_load_acc_;
				if (opt_params_.cx(index) > 0.0)
					debug_params_.init_cost_tension += opt_params_.cx(index);
			}
			// calculate the grad of constraint violation
			if (cts_partial_load_acc_.norm() < 1e-8 || flag_ten_ == 0) {
				grad_cts.row(index).setZero();
			} else {
				partial_T_.setZero();
				partial_coeffs_.setZero();
				get_grad_CT(node_, alpha_, step_, i, info_params_.traj_order,
				            beta0, beta1, beta2, beta3, load_vel_, load_acc_,
				            load_jer_, load_sna_, quad_vel_, quad_acc_,
				            quad_jer_, quad_sna_, Eigen::Vector3d::Zero(),
				            Eigen::Vector3d::Zero(), cts_partial_load_acc_,
				            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
				            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
				            Eigen::Vector3d::Zero(), partial_T_,
				            partial_coeffs_);
				info_params_.minco->propogateGrad(partial_coeffs_, partial_T_,
				                                  grad_inner_pts_,
				                                  grad_durations_);
				backwardGradT(T, grad_durations_, grad_tau_);
				Eigen::Map<Eigen::VectorXd> grad_inner_pts_vec(
				    grad_inner_pts_.data(), grad_inner_pts_.size());
				grad_cts.row(index).head(grad_inner_pts_.size()) =
				    grad_inner_pts_vec;
				grad_cts.row(index).tail(grad_tau_.size()) = grad_tau_;
			}
			index += 1;

			/* distance up bound feasibility constraint penalty */
			opt_params_.cx_flag(index) = 1;
			if (flag_dis_ != 0) {
				inequalityConstraintDis(
				    opt_params_.scaling_wc(index), opt_params_.lambda(index),
				    opt_params_.rho, dis_mean_, dis_radi_, load_pos_, quad_pos_,
				    opt_params_.cx(index), cts_partial_load_pos_,
				    cts_partial_quad_pos_, distance_pena_,
				    term_partial_load_pos_, term_partial_quad_pos_);
				pena_ += distance_pena_;
				total_grad_load_pos_ += term_partial_load_pos_;
				total_grad_quad_pos_ += term_partial_quad_pos_;
				if (opt_params_.cx(index) > 0.0)
					debug_params_.init_cost_dist += opt_params_.cx(index);
			}
			// calculate the grad of constraint violation
			if ((cts_partial_load_pos_.norm() < 1e-8 &&
			     cts_partial_quad_pos_.norm() < 1e-8) ||
			    flag_dis_ == 0) {
				grad_cts.row(index).setZero();
			} else {
				partial_T_.setZero();
				partial_coeffs_.setZero();
				get_grad_CT(node_, alpha_, step_, i, info_params_.traj_order,
				            beta0, beta1, beta2, beta3, load_vel_, load_acc_,
				            load_jer_, load_sna_, quad_vel_, quad_acc_,
				            quad_jer_, quad_sna_, cts_partial_load_pos_,
				            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
				            Eigen::Vector3d::Zero(), cts_partial_quad_pos_,
				            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
				            Eigen::Vector3d::Zero(), partial_T_,
				            partial_coeffs_);
				info_params_.minco->propogateGrad(partial_coeffs_, partial_T_,
				                                  grad_inner_pts_,
				                                  grad_durations_);
				backwardGradT(T, grad_durations_, grad_tau_);
				Eigen::Map<Eigen::VectorXd> grad_inner_pts_vec(
				    grad_inner_pts_.data(), grad_inner_pts_.size());
				grad_cts.row(index).head(grad_inner_pts_.size()) =
				    grad_inner_pts_vec;
				grad_cts.row(index).tail(grad_tau_.size()) = grad_tau_;
			}
			index += 1;

			/* dynamic equality constraint penalty */
			opt_params_.cx_flag(index) = 0;
			if (flag_dyn_ != 0) {
				equalityConstraintDyn(
				    opt_params_.scaling_wc(index), opt_params_.lambda(index),
				    opt_params_.rho, load_pos_, load_acc_, quad_pos_,
				    opt_params_.cx(index), cts_partial_load_pos_,
				    cts_partial_load_acc_, cts_partial_quad_pos_, dyn_pena_,
				    term_partial_load_pos_, term_partial_load_acc_,
				    term_partial_quad_pos_);
				pena_ += dyn_pena_;
				total_grad_load_pos_ += term_partial_load_pos_;
				total_grad_load_acc_ += term_partial_load_acc_;
				total_grad_quad_pos_ += term_partial_quad_pos_;
				debug_params_.init_cost_dyn += opt_params_.cx(index);
			}
			// calculate the grad of constraint violation
			if ((cts_partial_load_pos_.norm() < 1e-8 &&
			     cts_partial_load_acc_.norm() < 1e-8 &&
			     cts_partial_quad_pos_.norm() < 1e-8) ||
			    flag_dyn_ == 0) {
				grad_cts.row(index).setZero();
			} else {
				partial_T_.setZero();
				partial_coeffs_.setZero();
				get_grad_CT(node_, alpha_, step_, i, info_params_.traj_order,
				            beta0, beta1, beta2, beta3, load_vel_, load_acc_,
				            load_jer_, load_sna_, quad_vel_, quad_acc_,
				            quad_jer_, quad_sna_, cts_partial_load_pos_,
				            Eigen::Vector3d::Zero(), cts_partial_load_acc_,
				            Eigen::Vector3d::Zero(), cts_partial_quad_pos_,
				            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
				            Eigen::Vector3d::Zero(), partial_T_,
				            partial_coeffs_);
				info_params_.minco->propogateGrad(partial_coeffs_, partial_T_,
				                                  grad_inner_pts_,
				                                  grad_durations_);
				backwardGradT(T, grad_durations_, grad_tau_);
				Eigen::Map<Eigen::VectorXd> grad_inner_pts_vec(
				    grad_inner_pts_.data(), grad_inner_pts_.size());
				grad_cts.row(index).head(grad_inner_pts_.size()) =
				    grad_inner_pts_vec;
				grad_cts.row(index).tail(grad_tau_.size()) = grad_tau_;
			}
			index += 1;

			/* complementary equality constraint penalty */
			opt_params_.cx_flag(index) = 0;
			if (flag_com_ != 0) {
				equalityConstraintCom(
				    opt_params_.scaling_wc(index), opt_params_.lambda(index),
				    opt_params_.rho, load_pos_, load_acc_, quad_pos_,
				    opt_params_.cx(index), cts_partial_load_pos_,
				    cts_partial_load_acc_, cts_partial_quad_pos_, com_pena_,
				    term_partial_load_pos_, term_partial_load_acc_,
				    term_partial_quad_pos_);
				pena_ += com_pena_;
				total_grad_load_pos_ += term_partial_load_pos_;
				total_grad_quad_pos_ += term_partial_quad_pos_;
				total_grad_load_acc_ += term_partial_load_acc_;
				debug_params_.init_cost_comp += opt_params_.cx(index);
			}
			// calculate the grad of constraint violation
			if ((cts_partial_load_pos_.norm() < 1e-8 &&
			     cts_partial_load_acc_.norm() < 1e-8 &&
			     cts_partial_quad_pos_.norm() < 1e-8) ||
			    flag_com_ == 0) {
				grad_cts.row(index).setZero();
			} else {
				partial_T_.setZero();
				partial_coeffs_.setZero();
				get_grad_CT(node_, alpha_, step_, i, info_params_.traj_order,
				            beta0, beta1, beta2, beta3, load_vel_, load_acc_,
				            load_jer_, load_sna_, quad_vel_, quad_acc_,
				            quad_jer_, quad_sna_, cts_partial_load_pos_,
				            Eigen::Vector3d::Zero(), term_partial_load_acc_,
				            Eigen::Vector3d::Zero(), cts_partial_quad_pos_,
				            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
				            Eigen::Vector3d::Zero(), partial_T_,
				            partial_coeffs_);
				info_params_.minco->propogateGrad(partial_coeffs_, partial_T_,
				                                  grad_inner_pts_,
				                                  grad_durations_);
				backwardGradT(T, grad_durations_, grad_tau_);
				Eigen::Map<Eigen::VectorXd> grad_inner_pts_vec(
				    grad_inner_pts_.data(), grad_inner_pts_.size());
				grad_cts.row(index).head(grad_inner_pts_.size()) =
				    grad_inner_pts_vec;
				grad_cts.row(index).tail(grad_tau_.size()) = grad_tau_;
			}
			index += 1;

			// calculate totoal grad
			gradC.block(i * (traj_order_ + 1), 0, traj_order_ + 1, 3) +=
			    (beta0 * total_grad_load_pos_.transpose() +
			     beta1 * total_grad_load_vel_.transpose() +
			     beta2 * total_grad_load_acc_.transpose() +
			     beta3 * total_grad_load_jer_.transpose()) *
			    node_ * step_;
			gradC.block(i * (traj_order_ + 1), 3, traj_order_ + 1, 3) +=
			    (beta0 * total_grad_quad_pos_.transpose() +
			     beta1 * total_grad_quad_vel_.transpose() +
			     beta2 * total_grad_quad_acc_.transpose() +
			     beta3 * total_grad_quad_jer_.transpose()) *
			    node_ * step_;
			gradT(i) += (total_grad_load_pos_.dot(load_vel_) +
			             total_grad_load_vel_.dot(load_acc_) +
			             total_grad_load_acc_.dot(load_jer_) +
			             total_grad_load_jer_.dot(load_sna_) +
			             total_grad_quad_pos_.dot(quad_vel_) +
			             total_grad_quad_vel_.dot(quad_acc_) +
			             total_grad_quad_acc_.dot(quad_jer_) +
			             total_grad_quad_jer_.dot(quad_sna_)) *
			            alpha_ * node_ * step_;
			cost += pena_ * node_ * step_;
		}
	}
}

bool HybridOPT::fischer_burmeister(const double& a, const double& b, double& f,
                                   double& dfa, double& dfb) {
	f = (a + b) - sqrt(a * a + b * b + 0.01);
	dfa = 1.0 - a / sqrt(a * a + b * b + 0.01);
	dfb = 1.0 - b / sqrt(a * a + b * b + 0.01);
	return true;
}

void HybridOPT::updateALM(void) {
	updateLambda();
	updateRho();
	// lbfgs_params_.g_epsilon = std::max(5.0e-3, lbfgs_params_.g_epsilon *
	// 0.1);
}

void HybridOPT::updateLambda(void) {
	double cts_num_ = opt_params_.equality_constraint_num +
	                  opt_params_.equality_constraint_num;
	for (int i = 0; i < cts_num_; i++) {
		if (opt_params_.cx_flag(i) == 0) {
			opt_params_.lambda(i) +=
			    opt_params_.rho * opt_params_.scaling_wc(i) * opt_params_.cx(i);
		} else {
			if (opt_params_.lambda(i) + opt_params_.rho *
			                                opt_params_.scaling_wc(i) *
			                                opt_params_.cx(i) >=
			    0) {
				opt_params_.lambda(i) += opt_params_.rho *
				                         opt_params_.scaling_wc(i) *
				                         opt_params_.cx(i);
			} else {
				opt_params_.lambda(i) = 0;
			}
		}
	}
	return;
}

void HybridOPT::updateRho(void) {
	opt_params_.rho = std::min((1 + opt_params_.gamma) * opt_params_.rho,
	                           opt_params_.rho_max);
	return;
}

void HybridOPT::check_optimal(void) {
	int cts_num_ = opt_params_.equality_constraint_num +
	               opt_params_.inequality_constraint_num;
	Eigen::VectorXd max_hx_(opt_params_.equality_constraint_num);
	Eigen::VectorXd max_gx_(opt_params_.inequality_constraint_num);
	for (int i = 0, j = 0, k = 0; i < cts_num_; i++) {
		if (opt_params_.cx_flag(i) == 0) {
			max_hx_(j) = opt_params_.scaling_wc(i) * opt_params_.cx(i);
			j++;
		} else {
			max_gx_(k) =
			    std::max(0.0, opt_params_.scaling_wc(i) * opt_params_.cx(i));
			k++;
		}
	}
	opt_params_.hx_inf = max_hx_.cwiseAbs().maxCoeff();
	opt_params_.gx_inf = max_gx_.cwiseAbs().maxCoeff();
	opt_params_.grad_inf = opt_params_.grad.cwiseAbs().maxCoeff();
	if ((opt_params_.hx_inf < opt_params_.eq_cons_eps) &&
	    (opt_params_.gx_inf < opt_params_.ineq_cons_eps) &&
	    (opt_params_.grad_inf < opt_params_.prec_eps)) {
		opt_params_.stop_flag = true;
	}
	return;
}
} // namespace alm_opt