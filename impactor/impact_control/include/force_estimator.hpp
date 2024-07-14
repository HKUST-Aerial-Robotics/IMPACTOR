/**
 * @file force_estimator.hpp
 * @author Haojia Li (hlied@connect.ust.hk)
 * @brief Estimate the extern force and moment from the payload
 * @version 1.0
 * @date 2022-10-03
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once
#include <float.h>
#include "Eigen/Eigen"
#include "mpc_params.h"
#include <math.h>
#include "lowpassfilter2p.h"
namespace PayloadMPC
{

  class ForceEstimator
  {
  private:
    Eigen::Vector3d quad_acc_;
    Eigen::Vector3d load_acc_;
    Eigen::Vector3d cable_;
    Eigen::Vector4d rpm_;
    Eigen::Quaterniond quad_q_;
    Eigen::Vector3d Thr_;
    double T_, sqrt_kf_;
    LowPassFilter2p<Eigen::Vector3d> fq_filter_;
    LowPassFilter2p<Eigen::Vector3d> fl_filter_;

    double mass_quad_, mass_load_, g_, imu_body_length_, l_length_, imu_load_length_, Thrust_;
    double sample_freq_fq_{333.333}, sample_freq_fl_{333.333};
    double cutoff_freq_fq_{100.0}, cutoff_freq_fl_{100.0};
    double max_force_sqr_, max_force_;
    bool use_force_estimator_;
    int USE_CONSTANT_MOMENT_;

  public:
    ForceEstimator(/* args */)
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

      quad_acc_ = Rotwb * quad_acc_body;
      quad_acc_(2) += g_;
      quad_q_ = Rotwb;

      load_acc_ = load_Rotwb * load_acc_body;
      load_acc_(2) += g_;

      load_acc_ = (load_acc_ - quad_acc_) / imu_body_length_ * l_length_ + quad_acc_;

      cable_ = cable;

      rpm_ = Rpm;
      T_ = (sqrt_kf_ * rpm_).squaredNorm();
      Thr_ = T_ * quad_q_.toRotationMatrix().col(2);
    }

    void caculate_force(Eigen::Vector3d &fl, Eigen::Vector3d &fq)
    {
      if (use_force_estimator_)
      {
        Eigen::Vector3d C, B;
        Eigen::Vector3d fl_local, fq_local;
        C = Thr_ - mass_quad_ * (Eigen::Vector3d(0, 0, g_) + quad_acc_);
        B = mass_quad_ * (Eigen::Vector3d(0, 0, g_) + quad_acc_) + mass_load_ * (Eigen::Vector3d(0, 0, g_) + load_acc_) - Thr_;
        // fl_local(0) = B(0) +C(0)*(cable_(0)*cable_(0)-1) + C(1)* cable_(0) *cable_(1) + C(2)* cable_(0)*cable_(2);
        // fl_local(1) = B(1) +C(1)*(cable_(1)*cable_(1)-1) + C(0)* cable_(0) *cable_(1) + C(2)* cable_(1)*cable_(2);
        // fl_local(2) = B(2) +C(2)*(cable_(2)*cable_(2)-1) + C(0)* cable_(0) *cable_(2) + C(1)* cable_(1)*cable_(2);
        // fq_local = B-fl_local;

        fq_local(0) = B(0) + (B(1) - C(1)) * cable_(0) * cable_(1) + (C(0) - B(0)) * (1 - cable_(0) * cable_(0)) + (B(2) - C(2)) * cable_(0) * cable_(2);
        fq_local(1) = C(1) + (B(0) - C(0)) * cable_(0) * cable_(1) + (B(1) - C(1)) * cable_(1) * cable_(1) + (B(2) - C(2)) * cable_(1) * cable_(2);
        fq_local(2) = C(2) + (B(0) - C(0)) * cable_(0) * cable_(2) + (B(1) - C(1)) * cable_(1) * cable_(2) + (B(2) - C(2)) * cable_(2) * cable_(2);
        fl_local = B - fq_local;

        if (fl_local.squaredNorm() > max_force_sqr_)
        {
          fl_local = fl_local.normalized() * max_force_;
        }
        if (fq_local.squaredNorm() > max_force_sqr_)
        {
          fq_local = fq_local.normalized() * max_force_;
        }
        if (USE_CONSTANT_MOMENT_ == 1)
        {
          fl_local = cable_.cross(fl_local);
        }
        else if (USE_CONSTANT_MOMENT_ == 2)
        {
          fq_local = cable_.cross(fq_local);
        }
        fl = fl_filter_.apply(fl_local);
        fq = fq_filter_.apply(fq_local);
      }
      else
      {
        fl = Eigen::Vector3d::Zero();
        fq = Eigen::Vector3d::Zero();
      }
    }
  };

} // namespace PayloadMPC
