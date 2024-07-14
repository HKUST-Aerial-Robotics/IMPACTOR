/************************************************************************
 * Date:        2023.02
 * Author:      Haokun Wang <hwangeh at connect dot ust dot hk>, 
                Aerial Robotics Group <https://uav.ust.hk>, HKUST.
 * E-mail:      hwangeh_at_connect_dot_ust_dot_hk
 * Description: This is the header file for the FlatnessMap class, which 
 *              is used to calculate the flatness map for the quadrotor 
 *              and the load.
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


#ifndef FLATNESS_HPP
#define FLATNESS_HPP

#include <Eigen/Eigen>

#include <cmath>

namespace flatness
{
    class FlatnessMap
    {
    public:
        inline void reset(const double &quad_mass,
                          const double &load_mass,
                          const double &gravitational_acceleration,
                          const double &speed_smooth_factor)
        {
            massQ = quad_mass;
            massL = load_mass;
            grav = gravitational_acceleration;
            eps = speed_smooth_factor;

            return;
        }

        inline void forward(const Eigen::Vector3d &load_acc,
                            const Eigen::Vector3d &load_jer,
                            const Eigen::Vector3d &quad_acc,
                            const Eigen::Vector3d &quad_jer,
                            const double &psi, const double &dpsi,
                            double &thr, Eigen::Vector4d &quat, Eigen::Vector3d &omg)
        {
            load_acc0 = load_acc(0);
            load_acc1 = load_acc(1);
            load_acc2 = load_acc(2);
            load_jer0 = load_jer(0);
            load_jer1 = load_jer(1);
            load_jer2 = load_jer(2);

            quad_acc0 = quad_acc(0);
            quad_acc1 = quad_acc(1);
            quad_acc2 = quad_acc(2);
            quad_jer0 = quad_jer(0);
            quad_jer1 = quad_jer(1);
            quad_jer2 = quad_jer(2);

            Psi = psi;
            dPsi = dpsi;

            zu0 = massQ * quad_acc0 + massL * load_acc0;
            zu1 = massQ * quad_acc1 + massL * load_acc1;
            zu2 = massQ * (quad_acc2 + grav) + massL * (load_acc2 + grav);
            zu01 = zu0 * zu1;
            zu12 = zu1 * zu2;
            zu02 = zu0 * zu2;
            zu_sqr0 = zu0 * zu0;
            zu_sqr1 = zu1 * zu1;
            zu_sqr2 = zu2 * zu2;
            zu_sqr_norm = zu_sqr0 + zu_sqr1 + zu_sqr2;
            zu_norm = sqrt(zu_sqr_norm);

            z0 = zu0 / zu_norm;
            z1 = zu1 / zu_norm;
            z2 = zu2 / zu_norm;

            ng_den = zu_sqr_norm * zu_norm;
            ng00 = (zu_sqr_norm - zu_sqr0) / ng_den;
            ng01 = -zu01 / ng_den;
            ng02 = -zu02 / ng_den;
            ng11 = (zu_sqr_norm - zu_sqr1) / ng_den;
            ng12 = -zu12 / ng_den;
            ng22 = (zu_sqr_norm - zu_sqr2) / ng_den;

            tilt_den = sqrt(2.0 * (1.0 + z2));
            tilt0 = 0.5 * tilt_den;
            tilt1 = -z1 / tilt_den;
            tilt2 = z0 / tilt_den;

            dz_term0 = massQ * quad_jer0 + massL * load_jer0;
            dz_term1 = massQ * quad_jer1 + massL * load_jer1;
            dz_term2 = massQ * quad_jer2 + massL * load_jer2;

            dz0 = ng00 * dz_term0 + ng01 * dz_term1 + ng02 * dz_term2;
            dz1 = ng01 * dz_term0 + ng11 * dz_term1 + ng12 * dz_term2;
            dz2 = ng02 * dz_term0 + ng12 * dz_term1 + ng22 * dz_term2;

            omg_den = z2 + 1.0;
            omg_term = dz2 / omg_den;

            c_half_psi = cos(0.5 * psi);
            s_half_psi = sin(0.5 * psi);
            c_psi = cos(psi);
            s_psi = sin(psi);

            thrust = zu_norm;

            q0 = tilt0 * c_half_psi;
            q1 = tilt1 * c_half_psi + tilt2 * s_half_psi;
            q2 = tilt2 * c_half_psi - tilt1 * s_half_psi;
            q3 = tilt0 * s_half_psi;

            // q_norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

            // quat0 = q0 / q_norm;
            // quat1 = q1 / q_norm;
            // quat2 = q2 / q_norm;
            // quat3 = q3 / q_norm;

            omg0 = dz0 * s_psi - dz1 * c_psi -
                   (z0 * s_psi - z1 * c_psi) * omg_term;
            omg1 = dz0 * c_psi + dz1 * s_psi -
                   (z0 * c_psi + z1 * s_psi) * omg_term;
            omg2 = (z1 * dz0 - z0 * dz1) / omg_den + dpsi;

            thr = thrust;
            // quat << quat0, quat1, quat2, quat3;
            quat << q0, q1, q2, q3;
            omg << omg0, omg1, omg2;
            return;
        }

        inline void backward(const double &thr_grad,
                             const Eigen::Vector4d &quat_grad,
                             const Eigen::Vector3d &omg_grad,
                             Eigen::Vector3d &load_acc_grad,
                             Eigen::Vector3d &load_jer_grad,
                             Eigen::Vector3d &quad_acc_grad,
                             Eigen::Vector3d &quad_jer_grad,
                             double &psi_total_grad,
                             double &dpsi_total_grad) const
        {
            double zu0b = 0.0, zu1b = 0.0, zu2b = 0.0;
            double zu_sqr_normb = 0.0, zu_normb = 0.0;
            double zu01b = 0.0, zu12b = 0.0, zu02b = 0.0;
            double zu_sqr0b = 0.0, zu_sqr1b = 0.0, zu_sqr2b = 0.0;
            double z0b = 0.0, z1b = 0.0, z2b = 0.0;
            double dz0b = 0.0, dz1b = 0.0, dz2b = 0.0;
            double dz_term0b = 0.0, dz_term1b = 0.0, dz_term2b = 0.0;
            double ng_denb = 0.0, ng00b = 0.0, ng01b = 0.0, ng02b = 0.0, ng11b = 0.0, ng12b = 0.0, ng22b = 0.0;
            double tilt_denb = 0.0, tilt0b = 0.0, tilt1b = 0.0, tilt2b = 0.0;
            double omg_denb = 0.0, omg_termb = 0.0;
            double q_normb = 0.0, q0b = 0.0, q1b = 0.0, q2b = 0.0, q3b = 0.0;
            double c_half_psib = 0.0, s_half_psib = 0.0, c_psib = 0.0, s_psib = 0.0;
            double tempb = 0.0;

            omg_termb = -((z0 * c_psi + z1 * s_psi) * omg_grad(1)) - (z0 * s_psi - z1 * c_psi) * omg_grad(0);
            tempb = omg_grad(2) / omg_den;
            dpsi_total_grad = omg_grad(2);

            z1b = dz0 * tempb;
            dz0b = z1 * tempb + c_psi * omg_grad(1) + s_psi * omg_grad(0);
            z0b = -(dz1 * tempb);
            dz1b = s_psi * omg_grad(1) - z0 * tempb - c_psi * omg_grad(0);
            omg_denb = -((z1 * dz0 - z0 * dz1) * tempb / omg_den) - dz2 * omg_termb / (omg_den * omg_den);
            tempb = -(omg_term * omg_grad(1));
            c_psib = dz0 * omg_grad(1) + z0 * tempb;
            s_psib = dz1 * omg_grad(1) + z1 * tempb;

            z0b = z0b + c_psi * tempb;
            z1b = z1b + s_psi * tempb;
            tempb = -(omg_term * omg_grad(0));
            s_psib = s_psib + dz0 * omg_grad(0) + z0 * tempb;
            c_psib = c_psib - dz1 * omg_grad(0) - z1 * tempb;

            z0b = z0b + s_psi * tempb;
            z1b = z1b - c_psi * tempb;
            // z0b = z0b + s_psi * tempb;
            // z1b = z1b - c_psi * tempb;
            // q_normb = -(q3 * (quat_grad(3)) / (q_norm * q_norm)) - q2 * (quat_grad(2)) / (q_norm * q_norm) -
            //           q1 * (quat_grad(1)) / (q_norm * q_norm) - q0 * (quat_grad(0)) / (q_norm * q_norm);
            // tempb = (q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3 == 0.0 ? 0.0 : q_normb / (2.0 * sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)));
            // q3b = quat_grad(3) / q_norm + 2 * q3 * tempb;
            // q2b = quat_grad(2) / q_norm + 2 * q2 * tempb;
            // q1b = quat_grad(1) / q_norm + 2 * q1 * tempb;
            // q0b = quat_grad(0) / q_norm + 2 * q0 * tempb;
            q3b = quat_grad(3);
            q2b = quat_grad(2);
            q1b = quat_grad(1);
            q0b = quat_grad(0);

            tilt0b = s_half_psi * q3b + c_half_psi * q0b;
            s_half_psib = tilt0 * q3b + tilt2 * q1b - tilt1 * q2b;
            tilt2b = c_half_psi * q2b + s_half_psi * q1b;
            c_half_psib = tilt2 * q2b + tilt1 * q1b + tilt0 * q0b;
            tilt1b = c_half_psi * q1b - s_half_psi * q2b;

            psi_total_grad = cos(Psi) * s_psib + 0.5 * cos(0.5 * Psi) * s_half_psib - sin(Psi) * c_psib -
                             0.5 * sin(0.5 * Psi) * c_half_psib;

            dz2b = omg_termb / omg_den;
            ng02b = dz_term0 * dz2b + dz_term2 * dz0b;
            dz_term0b = ng02 * dz2b + ng01 * dz1b + ng00 * dz0b;
            ng12b = dz_term1 * dz2b + dz_term2 * dz1b;
            dz_term1b = ng12 * dz2b + ng11 * dz1b + ng01 * dz0b;
            ng22b = dz_term2 * dz2b;
            dz_term2b = ng22 * dz2b + ng12 * dz1b + ng02 * dz0b;
            ng01b = dz_term0 * dz1b + dz_term1 * dz0b;
            ng11b = dz_term1 * dz1b;
            ng00b = dz_term0 * dz0b;

            quad_jer_grad(2) = massQ * dz_term2b;
            quad_jer_grad(1) = massQ * dz_term1b;
            quad_jer_grad(0) = massQ * dz_term0b;
            load_jer_grad(2) = massL * dz_term2b;
            load_jer_grad(1) = massL * dz_term1b;
            load_jer_grad(0) = massL * dz_term0b;

            z0b = z0b + tilt2b / tilt_den;
            tilt_denb = z1 * tilt1b / (tilt_den * tilt_den) - z0 * tilt2b / (tilt_den * tilt_den) + 0.5 * tilt0b;
            z2b = (2.0 * (z2 + 1.0) == 0.0 ? omg_denb : omg_denb + tilt_denb / sqrt(2.0 * (z2 + 1.0)));
            z1b = z1b - tilt1b / tilt_den;
            tempb = ng22b / ng_den;
            zu_sqr0b = tempb;
            zu_sqr1b = tempb;
            ng_denb = -((zu_sqr0 + zu_sqr1) * tempb / ng_den);
            zu12b = -(ng12b / ng_den);
            tempb = ng11b / ng_den;
            ng_denb = ng_denb + zu12 * ng12b / (ng_den * ng_den) - (zu_sqr0 + zu_sqr2) * tempb / ng_den;
            zu_sqr0b = zu_sqr0b + tempb;
            zu_sqr2b = tempb;
            zu02b = -(ng02b / ng_den);
            zu01b = -(ng01b / ng_den);
            tempb = ng00b / ng_den;
            ng_denb = ng_denb + zu02 * ng02b / (ng_den * ng_den) + zu01 * ng01b / (ng_den * ng_den) - (zu_sqr1 + zu_sqr2) * tempb / ng_den;
            zu_normb = thr_grad + zu_sqr_norm * ng_denb - zu2 * z2b / (zu_norm * zu_norm) - zu1 * z1b / (zu_norm * zu_norm) - zu0 * z0b / (zu_norm * zu_norm);
            // zu_normb = zu_sqr_norm * ng_denb - zu2 * z2b / (zu_norm * zu_norm) - zu1 * z1b / (zu_norm * zu_norm) - zu0 * z0b / (zu_norm * zu_norm);

            zu_sqr_normb = (zu_sqr_norm == 0.0 ? zu_norm * ng_denb : zu_norm * ng_denb + zu_normb / (2.0 * sqrt(zu_sqr_norm)));
            zu_sqr1b = zu_sqr1b + tempb + zu_sqr_normb;
            zu_sqr2b = zu_sqr2b + tempb + zu_sqr_normb;
            zu2b = z2b / zu_norm + 2 * zu2 * zu_sqr2b + zu0 * zu02b + zu1 * zu12b;
            zu1b = z1b / zu_norm + 2 * zu1 * zu_sqr1b + zu2 * zu12b + zu0 * zu01b;
            zu_sqr0b = zu_sqr0b + zu_sqr_normb;
            zu0b = z0b / zu_norm + 2 * zu0 * zu_sqr0b + zu2 * zu02b + zu1 * zu01b;

            quad_acc_grad(2) = massQ * zu2b;
            quad_acc_grad(1) = massQ * zu1b;
            quad_acc_grad(0) = massQ * zu0b;
            load_acc_grad(2) = massL * zu2b;
            load_acc_grad(1) = massL * zu1b;
            load_acc_grad(0) = massL * zu0b;

            return;
        }

    private:
        // Parameters
        double massQ, massL, grav, eps;
        // forward params
        double load_acc0 = 0, load_acc1 = 0, load_acc2 = 0;
        double quad_acc0 = 0, quad_acc1 = 0, quad_acc2 = 0;
        double load_jer0 = 0, load_jer1 = 0, load_jer2 = 0;
        double quad_jer0 = 0, quad_jer1 = 0, quad_jer2 = 0;
        double Psi = 0, dPsi = 0;
        double thrust = 0;
        double q0 = 0, q1 = 0, q2 = 0, q3 = 0; // q_norm = 0;
        // double quat0 = 0, quat1 = 0, quat2 = 0, quat3 = 0;
        double omg0 = 0, omg1 = 0, omg2 = 0;
        double zu0 = 0, zu1 = 0, zu2 = 0, zu_sqr_norm = 0, zu_norm = 0;
        double zu01 = 0, zu12 = 0, zu02 = 0;
        double zu_sqr0 = 0, zu_sqr1 = 0, zu_sqr2 = 0;
        double z0 = 0, z1 = 0, z2 = 0, z_term = 0;
        double dz0 = 0, dz1 = 0, dz2 = 0;
        double dz_term0 = 0, dz_term1 = 0, dz_term2 = 0;
        double ng_den = 0;
        double ng00 = 0, ng01 = 0, ng02 = 0, ng10 = 0, ng11, ng12 = 0, ng20 = 0, ng21 = 0, ng22 = 0;
        double tilt_den = 0, tilt0 = 0, tilt1 = 0, tilt2 = 0;
        double omg_den = 0, omg_term = 0;
        double c_half_psi = 0, c_psi = 0;
        double s_half_psi = 0, s_psi = 0;
    };
}

#endif
