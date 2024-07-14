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
#include "ceres/ceres.h"
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
    struct forceResidual
    {
      forceResidual(const systemState &state)
          : state_(state) {}

      template <typename T>
      bool operator()(const T *const fq,
                      const T *const fl,
                      T *residuals) const
      {
        T local_fq[3], local_fl[3];

        if (state_.USE_CONSTANT_MOMENT == 1)
        {
          local_fl[0] = fl[1]*T(state_.cable[2]) - fl[2]*T(state_.cable[1]);
          local_fl[1] = fl[2]*T(state_.cable[0]) - fl[0]*T(state_.cable[2]);
          local_fl[2] = fl[0]*T(state_.cable[1]) - fl[1]*T(state_.cable[0]);
          local_fq[0] = fq[0];
          local_fq[1] = fq[1];
          local_fq[2] = fq[2]; 
          residuals[6] = fl[0] * T(state_.cable[0]) + fl[1] * T(state_.cable[1]) + fl[2] * T(state_.cable[2]);
        }
        else if(state_.USE_CONSTANT_MOMENT == 2)
        {
          local_fq[0] = fq[1]*T(state_.cable[2]) - fq[2]*T(state_.cable[1]);
          local_fq[1] = fq[2]*T(state_.cable[0]) - fq[0]*T(state_.cable[2]);
          local_fq[2] = fq[0]*T(state_.cable[1]) - fq[1]*T(state_.cable[0]);
          local_fl[0] = fl[0];
          local_fl[1] = fl[1];
          local_fl[2] = fl[2]; 
          residuals[6] = fq[0] * T(state_.cable[0]) + fq[1] * T(state_.cable[1]) + fq[2] * T(state_.cable[2]);
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
          residuals[6] = fl[0] * T(state_.cable[0]) + fl[1] * T(state_.cable[1]) + fl[2] * T(state_.cable[2]);
        }

        residuals[0] = local_fq[0] + local_fl[0] - T(state_.total_force[0]);
        residuals[1] = local_fq[1] + local_fl[1] - T(state_.total_force[1]);
        residuals[2] = local_fq[2] + local_fl[2] - T(state_.total_force[2]);
        residuals[3] = (local_fq[2] + T(state_.C[2])) * T(state_.cable[1]) - (local_fq[1] + T(state_.C[1])) * T(state_.cable[2]);
        residuals[4] = (local_fq[0] + T(state_.C[0])) * T(state_.cable[2]) - (local_fq[2] + T(state_.C[2])) * T(state_.cable[0]);
        residuals[5] = (local_fq[1] + T(state_.C[1])) * T(state_.cable[0]) - (local_fq[0] + T(state_.C[0])) * T(state_.cable[1]);
        // residuals[6] = (fl[2] + T(state_.B[2])) * T(state_.cable[1]) - (fl[1] + T(state_.B[1])) * T(state_.cable[2]);
        // residuals[7] = (fl[0] + T(state_.B[0])) * T(state_.cable[2]) - (fl[2] + T(state_.B[2])) * T(state_.cable[0]);
        // residuals[8] = (fl[1] + T(state_.B[1])) * T(state_.cable[0]) - (fl[0] + T(state_.B[0])) * T(state_.cable[1]);
        // residuals[6] = (fl[0]*fl[0] + fl[1]*fl[1]+fl[2]*fl[2] +fq[0]*fq[0] + fq[1]*fq[1]+fq[2]*fq[2])*T(1.0e-3);
        return true;
      }
      static ceres::CostFunction *Create(const systemState &state)
      {
        return (new ceres::AutoDiffCostFunction<forceResidual, 7, 3, 3>(
            new forceResidual(state)));
      }
      systemState state_;
    };

    Eigen::Vector3d opt_fq_, opt_fl_;

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

    double max_force_;
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
      // state.B = - mass_load_ * load_acc_;
      state.total_force = mass_quad_ * quad_acc_ + mass_load_ *load_acc_ - Thr_;
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
          auto start = std::chrono::steady_clock::now();
          ceres::Problem problem;
          for (auto &s : state_buffer)
          {
            ceres::CostFunction *cost_function = forceResidual::Create(s);
            problem.AddResidualBlock(cost_function, new ceres::HuberLoss(1.0), opt_fq_.data(), opt_fl_.data());
          }
          for (int i = 0; i < 3; i++)
          {
            problem.SetParameterUpperBound(opt_fq_.data(), i, max_force_);
            problem.SetParameterLowerBound(opt_fq_.data(), i, -max_force_);
            problem.SetParameterUpperBound(opt_fl_.data(), i, max_force_);
            problem.SetParameterLowerBound(opt_fl_.data(), i, -max_force_);
          }

          ceres::Solver::Options options;
          options.linear_solver_type = ceres::DENSE_SCHUR;
          options.minimizer_progress_to_stdout = false;
          ceres::Solver::Summary summary;
          Solve(options, &problem, &summary);
          auto end = std::chrono::steady_clock::now();
          // std::cout << "Elapsed time in microseconds: "
          // << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
          // << " µs" << std::endl;
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
