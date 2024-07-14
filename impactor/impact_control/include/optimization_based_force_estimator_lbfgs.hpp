/**
 * @file optimization_based_force_estimator.hpp
 * @author Haojia Li (hlied@connect.ust.hk)
 * @brief optimization based extern force estimator
 * @version 1.0
 * @date 2022-10-05
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once
#include <float.h>
#include "Eigen/Eigen"
#include "mpc_params.h"
#include <math.h>
#include "deque"
#include "lbfgs.hpp"
#include "lowpassfilter2p.h"
namespace PayloadMPC
{

  class OptForceEstimator
  {
    typedef struct
    {
      Eigen::Vector3d total_force;
      Eigen::Vector3d C;
      Eigen::Vector3d B;
      Eigen::Vector3d cable;
      int USE_CONSTANT_MOMENT;

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } systemState;
    inline static double huber_loss(const double residual, double &gradient, const double delta = 1.0)
    {
      double huber_cost = 0.0;
      double abs_r = fabs(residual);

      if (abs_r <= delta)
      {
        huber_cost = 0.5 * residual * residual;
        gradient = residual;
      }
      else
      {
        huber_cost = delta * (abs_r - 0.5 * delta);
        gradient = delta * (residual > 0.0 ? 1.0 : -1.0);
      }
      return huber_cost;
    }

    static double cost_function(void *instance, const Eigen::VectorXd &x, Eigen::VectorXd &g)
    {
      double cost = 0;
      OptForceEstimator &estimator = *(OptForceEstimator *)instance;
      Eigen::Map<const Eigen::Vector3d> fq_array(x.data());
      Eigen::Map<const Eigen::Vector3d> fl_array(x.data() + 3);

      Eigen::Map<Eigen::Vector3d> fq_gradient(g.data());
      Eigen::Map<Eigen::Vector3d> fl_gradient(g.data() + 3);
      fq_gradient.setZero();
      fl_gradient.setZero();

      for (const systemState &state_ : estimator.state_buffer)
      {
        Eigen::Vector3d local_fq, local_fl;
        Eigen::Vector3d local_fq_grad(0, 0, 0), local_fl_grad(0, 0, 0);
        Eigen::Vector3d fl = fl_array;
        Eigen::Vector3d fq = fq_array;
        if (state_.USE_CONSTANT_MOMENT == 1)
        {
          local_fl[0] = fl[1] * (state_.cable[2]) - fl[2] * (state_.cable[1]);
          local_fl[1] = fl[2] * (state_.cable[0]) - fl[0] * (state_.cable[2]);
          local_fl[2] = fl[0] * (state_.cable[1]) - fl[1] * (state_.cable[0]);
          local_fq[0] = fq[0];
          local_fq[1] = fq[1];
          local_fq[2] = fq[2];
          double res = (fl[0] * (state_.cable[0]) + fl[1] * (state_.cable[1]) + fl[2] * (state_.cable[2]));
          double huber_gradient;
          cost += huber_loss(res, huber_gradient);
          fl_gradient += huber_gradient * state_.cable;
        }
        else if (state_.USE_CONSTANT_MOMENT == 2)
        {
          local_fq[0] = fq[1] * (state_.cable[2]) - fq[2] * (state_.cable[1]);
          local_fq[1] = fq[2] * (state_.cable[0]) - fq[0] * (state_.cable[2]);
          local_fq[2] = fq[0] * (state_.cable[1]) - fq[1] * (state_.cable[0]);
          local_fl[0] = fl[0];
          local_fl[1] = fl[1];
          local_fl[2] = fl[2];
          double res = fq[0] * (state_.cable[0]) + fq[1] * (state_.cable[1]) + fq[2] * (state_.cable[2]);
          double huber_gradient;
          cost += huber_loss(res, huber_gradient);
          fq_gradient += huber_gradient * state_.cable;
        }
        else
        {
          local_fq[0] = fq[0];
          local_fq[1] = fq[1];
          local_fq[2] = fq[2];
          local_fl[0] = fl[0];
          local_fl[1] = fl[1];
          local_fl[2] = fl[2];
          // residuals[9] = T(0);
          // residuals[6] = local_fq[0] * T(state_.cable[0]) + local_fq[1] * T(state_.cable[1]) + local_fq[2] * T(state_.cable[2]);
          double res = fl[0] * (state_.cable[0]) + fl[1] * (state_.cable[1]) + fl[2] * (state_.cable[2]);
          double huber_gradient;
          cost += huber_loss(res, huber_gradient);
          fl_gradient += huber_gradient * state_.cable;
        }

        Eigen::Vector3d r1 = local_fl + local_fq - state_.total_force;
        double huber_gradient;
        cost += huber_loss(r1[0], huber_gradient);
        local_fl_grad(0) += huber_gradient;
        local_fq_grad(0) += huber_gradient;
        cost += huber_loss(r1[1], huber_gradient);
        local_fl_grad(1) += huber_gradient;
        local_fq_grad(1) += huber_gradient;
        cost += huber_loss(r1[2], huber_gradient);
        local_fl_grad(2) += huber_gradient;
        local_fq_grad(2) += huber_gradient;

        double r2[3];

        r2[0] = (local_fq[2] + (state_.C[2])) * (state_.cable[1]) - (local_fq[1] + (state_.C[1])) * (state_.cable[2]);
        cost += huber_loss(r2[0], huber_gradient);
        local_fq_grad(2) += huber_gradient * state_.cable[1];
        local_fq_grad(1) -= huber_gradient * state_.cable[2];

        r2[1] = (local_fq[0] + (state_.C[0])) * (state_.cable[2]) - (local_fq[2] + (state_.C[2])) * (state_.cable[0]);
        cost += huber_loss(r2[1], huber_gradient);
        local_fq_grad(0) += huber_gradient * state_.cable[2];
        local_fq_grad(2) -= huber_gradient * state_.cable[0];

        r2[2] = (local_fq[1] + (state_.C[1])) * (state_.cable[0]) - (local_fq[0] + (state_.C[0])) * (state_.cable[1]);
        cost += huber_loss(r2[2], huber_gradient);
        local_fq_grad(1) += huber_gradient * state_.cable[0];
        local_fq_grad(0) -= huber_gradient * state_.cable[1];
        if (state_.USE_CONSTANT_MOMENT == 1)
        {
          // local_fl[0] = fl[1] * (state_.cable[2]) - fl[2] * (state_.cable[1]);
          // local_fl[1] = fl[2] * (state_.cable[0]) - fl[0] * (state_.cable[2]);
          // local_fl[2] = fl[0] * (state_.cable[1]) - fl[1] * (state_.cable[0]);
          fq_gradient += local_fq_grad;
          fl_gradient(1) += local_fl_grad(0) * state_.cable[2];
          fl_gradient(2) -= local_fl_grad(0) * state_.cable[1];
          fl_gradient(2) += local_fl_grad(1) * state_.cable[0];
          fl_gradient(0) -= local_fl_grad(1) * state_.cable[2];
          fl_gradient(0) += local_fl_grad(2) * state_.cable[1];
          fl_gradient(1) -= local_fl_grad(2) * state_.cable[0];
        }
        else if (state_.USE_CONSTANT_MOMENT == 2)
        {
          // local_fq[0] = fq[1] * (state_.cable[2]) - fq[2] * (state_.cable[1]);
          // local_fq[1] = fq[2] * (state_.cable[0]) - fq[0] * (state_.cable[2]);
          // local_fq[2] = fq[0] * (state_.cable[1]) - fq[1] * (state_.cable[0]);
          fq_gradient(1) += local_fq_grad(0) * state_.cable[2];
          fq_gradient(2) -= local_fq_grad(0) * state_.cable[1];
          fq_gradient(2) += local_fq_grad(1) * state_.cable[0];
          fq_gradient(0) -= local_fq_grad(1) * state_.cable[2];
          fq_gradient(0) += local_fq_grad(2) * state_.cable[1];
          fq_gradient(1) -= local_fq_grad(2) * state_.cable[0];
          fl_gradient += local_fl_grad;
        }
        else
        {
          fq_gradient += local_fq_grad;
          fl_gradient += local_fl_grad;
        }
      }
      return cost;
    }

    Eigen::Vector3d opt_fq_, opt_fl_;
    Eigen::VectorXd opt_var_;

  private:
    Eigen::Vector3d quad_acc_;
    Eigen::Vector3d load_acc_;
    Eigen::Vector3d cable_;
    Eigen::Vector4d rpm_;
    Eigen::Quaterniond quad_q_;
    Eigen::Vector3d Thr_;
    double T_, sqrt_kf_;

    std::deque<systemState> state_buffer;

    LowPassFilter2p<Eigen::Vector3d> fq_filter_;
    LowPassFilter2p<Eigen::Vector3d> fl_filter_;

    double mass_quad_, mass_load_, g_, imu_body_length_, l_length_, imu_load_length_, Thrust_;
    double sample_freq_fq_{333.333}, sample_freq_fl_{333.333};
    double cutoff_freq_fq_{100.0}, cutoff_freq_fl_{100.0};

    double max_force_, max_force_sqr_;
    bool use_force_estimator_;
    int USE_CONSTANT_MOMENT_;
    int queue_size_;

  public:
    OptForceEstimator(/* args */)
    {
    }

    void init(MpcParams &params)
    {
      mass_quad_ = params.dyn_params_.mass_q;
      mass_load_ = params.dyn_params_.mass_l;
      l_length_ = params.dyn_params_.l_length;
      imu_body_length_ = params.force_estimator_param_.imu_body_length;
      imu_load_length_ = l_length_ - imu_body_length_;

      sqrt_kf_ = params.force_estimator_param_.sqrt_kf;
      use_force_estimator_ = params.force_estimator_param_.use_force_estimator;
      max_force_ = params.force_estimator_param_.max_force;
      max_force_sqr_ = max_force_ * max_force_;

      sample_freq_fq_ = params.force_estimator_param_.sample_freq_fq;
      sample_freq_fl_ = params.force_estimator_param_.sample_freq_fl;
      cutoff_freq_fq_ = params.force_estimator_param_.cutoff_freq_fq;
      cutoff_freq_fl_ = params.force_estimator_param_.cutoff_freq_fl;

      USE_CONSTANT_MOMENT_ = params.force_estimator_param_.USE_CONSTANT_MOMENT;

      g_ = params.gravity_;

      fq_filter_.set_cutoff_frequency(sample_freq_fq_, cutoff_freq_fq_);
      fl_filter_.set_cutoff_frequency(sample_freq_fl_, cutoff_freq_fl_);
      fq_filter_.reset(Eigen::Vector3d::Zero());
      fl_filter_.reset(Eigen::Vector3d::Zero());

      queue_size_ = params.force_estimator_param_.max_queue;
      opt_var_.resize(6);
      opt_var_.setZero();
    }

    void enableForceEstimator()
    {
      use_force_estimator_ = true;
    }
    void disableForceEstimator()
    {
      use_force_estimator_ = false;
    }

    void setSystemState(const Eigen::Vector3d &quad_acc_body, const Eigen::Quaterniond &Rotwb, const Eigen::Vector3d &load_acc_body, const Eigen::Quaterniond &load_Rotwb, const Eigen::Vector3d cable, const Eigen::Vector4d &Rpm)
    {
      quad_acc_ = Rotwb * quad_acc_body; //include gravity
      // quad_acc_(2) += g_;
      quad_q_ = Rotwb;

      load_acc_ = load_Rotwb * load_acc_body; //include gravity
      // load_acc_(2) += g_;

      load_acc_ = (load_acc_ - quad_acc_) / imu_body_length_ * l_length_ + quad_acc_;

      cable_ = cable;

      rpm_ = Rpm;
      T_ = (sqrt_kf_ * rpm_).squaredNorm();
      Thr_ = T_ * quad_q_.toRotationMatrix().col(2);

      systemState state;
      state.C = Thr_ - mass_quad_ * quad_acc_;
      state.total_force = mass_quad_ * quad_acc_ + mass_load_ * load_acc_ - Thr_;
      state.cable = cable;
      state.USE_CONSTANT_MOMENT = USE_CONSTANT_MOMENT_;
      // std::cout<<"state.cable: "<<state.cable.transpose()<<std::endl;

      state_buffer.emplace_back(state);
      while (state_buffer.size() > queue_size_)
      {
        state_buffer.pop_front();
      }
    }

    void caculate_force(Eigen::Vector3d &fl, Eigen::Vector3d &fq)
    {
      if (use_force_estimator_)
      {
        if (state_buffer.size() <= 2)
        {
          fl = Eigen::Vector3d::Zero();
          fq = Eigen::Vector3d::Zero();
        }
        else
        {
          // auto start = std::chrono::steady_clock::now();
          lbfgs::lbfgs_parameter_t opt_param;
          opt_param.mem_size = 32;
          opt_param.g_epsilon = 1.0e-5;
          opt_param.delta = 1.0e-6;
          opt_param.past = 0;
          opt_param.max_iterations = 500;
          opt_param.max_linesearch = 128;
          double final_cost;

          int error_code = lbfgs::lbfgs_optimize(opt_var_, final_cost, OptForceEstimator::cost_function, nullptr, nullptr, this, opt_param);
          // auto end = std::chrono::steady_clock::now();
          // std::cout << "Elapsed time in microseconds: "
          // << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
          // << " µs" << std::endl;
          if (error_code < 0)
          {
            if (error_code == lbfgs::LBFGSERR_MAXIMUMITERATION)
            {
              
            }
            else
            {
              std::cout << "[Force estimator]error code: " << std::string(lbfgs::lbfgs_strerror(error_code)) << std::endl;
            }
          
          }
          Eigen::Map<Eigen::Vector3d> all_opt_fq(opt_var_.data());
          Eigen::Map<Eigen::Vector3d> all_opt_fl(opt_var_.data() + 3);

          opt_fl_ = all_opt_fl;
          opt_fq_ = all_opt_fq;
          if (opt_fl_.squaredNorm() > max_force_sqr_)
          {
            opt_fl_ = opt_fl_.normalized() * max_force_;
            all_opt_fl.setZero();
          }
          if (opt_fq_.squaredNorm() > max_force_sqr_)
          {
            opt_fq_ = opt_fq_.normalized() * max_force_;
            all_opt_fq.setZero();
          }
        }

        opt_fl_ = fl_filter_.apply(opt_fl_);
        opt_fq_ = fq_filter_.apply(opt_fq_);
        fl = opt_fl_;
        fq = opt_fq_;
      }
      else
      {
        fl = Eigen::Vector3d::Zero();
        fq = Eigen::Vector3d::Zero();
      }
    }
  };

} // namespace PayloadMPC
