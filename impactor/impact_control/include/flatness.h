#pragma once

#include <Eigen/Eigen>

#include <cmath>

namespace PayloadMPC
{
    class Flatness
    {
    public:
        void reset(const double &quadrotor_mass,
                   const double &payload_mass,
                   const double &l_length,
                   const double gravity = 9.81)
        {
            mass_q_ = quadrotor_mass;
            mass_l_ = payload_mass;
            l_length_ = l_length;
            grav_ = gravity;
        }

        void  qnorm(const Eigen::Vector3d &q, const Eigen::Vector3d &qd,
                   const Eigen::Vector3d &qdd, const Eigen::Vector3d &q3d,
                   Eigen::Vector3d &norm, Eigen::Vector3d &qnormd, Eigen::Vector3d &qnormdd, Eigen::Vector3d &qnorm3d)
        {
            double q_dot_q = q.squaredNorm();
            double q_norm = sqrt(q_dot_q);
            // If here need to check if q_norm is zero?
            // if (q_norm < 1e-8)
            // {
            //     norm=Eigen::Vector3d::Zero();
            //     norm.z() = -1.0;
            //     qnormd=Eigen::Vector3d::Zero();
            //     qnormdd = Eigen::Vector3d::Zero();
            //     qnorm3d = Eigen::Vector3d::Zero();
            // }
            assert(q_norm > 1e-8);

            double q_norm_3 = q_dot_q * q_norm;
            double q_norm_5 = q_dot_q * q_norm_3;
            double q_norm_7 = q_dot_q * q_norm_5;
            double q_dot_qd = q.dot(qd);
            double q_dot_qdd = q.dot(qdd);
            double qd_dot_qd = qd.dot(qd);
            double qd_dot_qdd = qd.dot(qdd);
            double q_dot_q3d = q.dot(q3d);

            double sqr_q_dot_qd = q_dot_qd * q_dot_qd;
            double q_dot_qd_3 = sqr_q_dot_qd * q_dot_qd;

            norm = q / q_norm;

            qnormd = (-q_dot_qd * q + q_dot_q * qd) / q_norm_3;

            qnormdd = (3 * (sqr_q_dot_qd)*q 
            - q_dot_q * ((q_dot_qdd + qd_dot_qd) * q 
            + 2 * q_dot_qd * qd)) / q_norm_5 + qdd / q_norm;

            qnorm3d = (-15 * q_dot_qd_3 * q 
            + 9 * q_dot_q * q_dot_qd * ((qd_dot_qd + q_dot_qdd) * q 
            + q_dot_qd * qd)) / q_norm_7 
            - ((q_dot_q3d + 3 * qd_dot_qdd) * q 
            + 3 * (qd_dot_qd + q_dot_qdd) * qd + 3 * q_dot_qd * qdd) / q_norm_3 
            + q3d / q_norm;
        }

        void normalizeFDF(const Eigen::Vector3d &x,
                             Eigen::Vector3d &xNor,
                             Eigen::Matrix3d &G)
        {
            const double a = x(0), b = x(1), c = x(2);
            const double aSqr = a * a, bSqr = b * b, cSqr = c * c;
            const double ab = a * b, bc = b * c, ca = c * a;
            const double xSqrNorm = aSqr + bSqr + cSqr;
            const double xNorm = sqrt(xSqrNorm);
            const double den = xSqrNorm * xNorm;
            xNor = x / xNorm;
            G(0, 0) = bSqr + cSqr;
            G(0, 1) = -ab;
            G(0, 2) = -ca;
            G(1, 0) = -ab;
            G(1, 1) = aSqr + cSqr;
            G(1, 2) = -bc;
            G(2, 0) = -ca;
            G(2, 1) = -bc;
            G(2, 2) = aSqr + bSqr;
            G /= den;
            // return;
        }

        // void qdotdot(const Eigen::Vector3d &q, const Eigen::Vector3d &jerk,
        //                                 Eigen::Matrix3d &G)
        // {
        //     double t0 = q.norm();
        //     double t1 = 1.0 / pow(t0, 3);
        //     double t2 = q.dot(jerk);

        //     G = -(t1 * jerk * q.transpose() + t1 * t2 * Eigen::Matrix3d::Identity() + t1 * q * jerk.transpose() - (3 * t2) * q * q.transpose() / pow(t0, 5));

        //     return;
        // }
        inline void forward_p(const Eigen::Vector3d &pos,
                            const Eigen::Vector3d &vel,
                            const Eigen::Vector3d &acc,
                            const Eigen::Vector3d &jerk,
                            const Eigen::Vector3d &snap,
                            const Eigen::Vector3d &crackle,

                            Eigen::Vector3d &pos_quad,
                            Eigen::Vector3d &vel_quad,
                            Eigen::Vector3d &acc_quad,
                            Eigen::Vector3d &jerk_quad,
                            Eigen::Vector3d &p,
                            Eigen::Vector3d &pd)
        {
            Eigen::Vector3d Tp = -acc; ////here is -Tp
            Tp(2) -= grav_;
            Eigen::Vector3d pdd, p3d;

            qnorm(Tp, -jerk, -snap, -crackle, p, pd, pdd, p3d);
            pos_quad = pos - l_length_ * p;
            vel_quad = vel - l_length_ * pd;
            acc_quad = acc - l_length_ * pdd;
            jerk_quad = jerk - l_length_ * p3d;

        }

        /*inline void forward_rot(const Eigen::Vector3d &acc,
                            const Eigen::Vector3d &jerk,
                            const Eigen::Vector3d &acc_quad,
                            const Eigen::Vector3d &jerk_quad,
                            const Eigen::Vector3d &fq,
                            const Eigen::Vector3d &fl,
                            const double &heading,
                            const double &heading_rate,

                            Eigen::Quaterniond &quat,
                            double &thr,
                            Eigen::Vector3d &omg)
        {

            Eigen::Vector3d des_f_in_world = mass_q_ * acc_quad + mass_l_ * acc + Eigen::Vector3d(0, 0, (mass_q_ + mass_l_) * grav_) - fq - fl;

            thr = des_f_in_world.norm();

            Eigen::Vector3d z_B;
            Eigen::Matrix3d G;
            normalizeFDF(des_f_in_world, z_B, G);
            double scal = sqrt(2 * (1 + z_B(2)));
            // 1 + z_B(2) shouldn't be zero

            const double c_half_psi = cos(heading * 0.5);
            const double s_half_psi = sin(heading * 0.5);
            const double s_psi = sin(heading);
            const double c_psi = cos(heading);
            Eigen::Quaterniond qz(scal * 0.5 * c_half_psi,          //w
             (-z_B[1] * c_half_psi + z_B[0] * s_half_psi) / scal,   //x 
             (z_B[0] * c_half_psi + z_B[1] * s_half_psi) / scal,    //y
             scal * 0.5 * s_half_psi);                              //z
            quat = qz.normalized();
            Eigen::Vector3d dz = G * (mass_q_ * jerk_quad + mass_l_ * jerk);

            double omg_den = 1.0 + z_B(2);
            double omg_term = dz(2) / omg_den;

            omg(0) = dz(0) * s_psi - dz(1) * c_psi -
                    (z_B(0) * s_psi - z_B(1) * c_psi) * omg_term;
            omg(1) = dz(0) * c_psi + dz(1) * s_psi -
                    (z_B(0) * c_psi + z_B(1) * s_psi) * omg_term;
            omg(2) = (z_B(1) * dz(0) - z_B(0) * dz(1)) / omg_den + heading_rate;

            return;
        }*/

    void forward_rot(const Eigen::Vector3d &acc,
                  const Eigen::Vector3d &jerk,
                  const Eigen::Vector3d &acc_quad,
                  const Eigen::Vector3d &jerk_quad,
                  const Eigen::Vector3d &fq,
                  const Eigen::Vector3d &fl,
                  const double &heading,
                  const double &heading_rate,

                  Eigen::Quaterniond &quat,
                  double &thr,
                  Eigen::Vector3d &omg)
    {

        Eigen::Vector3d des_f_in_world = mass_q_ * acc_quad + mass_l_ * acc + Eigen::Vector3d(0, 0, (mass_q_ + mass_l_) * grav_) - fq - fl;

        thr = des_f_in_world.norm();
        Eigen::Vector3d z_B;
        z_B = des_f_in_world.normalized();

        const Eigen::Quaterniond q_heading = Eigen::Quaterniond(
            Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ()));

        // Compute desired orientation
        const Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX();
        const Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY();

        const Eigen::Vector3d x_B = y_C.cross(z_B).normalized();

        const Eigen::Vector3d y_B = (z_B.cross(x_B)).normalized();

        // From the computed desired body axes we can now compose a desired attitude
        const Eigen::Matrix3d R_W_B((Eigen::Matrix3d() << x_B, y_B, z_B).finished());

        quat = Eigen::Quaterniond(R_W_B);

        // Reference body rates
        if (fabs(thr) < kAlmostZeroThrustThreshold_)
        {
            omg.x() = 0.0;
            omg.y() = 0.0;
        }
        else
        {
            omg.x() = -1.0 / thr * y_B.dot(mass_q_ * jerk_quad + mass_l_ * jerk);
            omg.y() = 1.0 / thr * x_B.dot(mass_q_ * jerk_quad + mass_l_ * jerk);
        }

        double normalize_y = (y_C.cross(z_B)).norm();

        if (fabs(normalize_y) < kAlmostZeroValueThreshold_)
        {
            omg.z() = 0.0;
        }
        else
        {
            omg.z() =
                1.0 / normalize_y *
                (heading_rate * x_C.dot(x_B) + omg.y() * y_C.dot(z_B));
        }
    }  

    private:
        double mass_q_, mass_l_, grav_, l_length_;
        const double kAlmostZeroThrustThreshold_{0.01};
        const double kAlmostZeroValueThreshold_{0.001};
    };
}
