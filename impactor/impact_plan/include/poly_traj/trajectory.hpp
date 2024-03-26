/************************************************************************
 * Date:        2023.02
 * Author:      Haokun Wang <hwangeh at connect dot ust dot hk>, 
                Aerial Robotics Group <https://uav.ust.hk>, HKUST.
 * E-mail:      hwangeh_at_connect_dot_ust_dot_hk
 * Description: This is the header file for the Trajectory class, which is
 *              used to generate the trajectory. Modified from the
 *              original code in project GCOPTER:
 *              <https://github.com/ZJU-FAST-Lab/GCOPTER>.
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

#ifndef _TRAJECTORY_HPP
#define _TRAJECTORY_HPP

#include "poly_traj/root_finder.hpp"

#include <Eigen/Eigen>

#include <iostream>
#include <cmath>
#include <cfloat>
#include <vector>
#include <memory>

namespace poly_traj
{

    class Piece
    {
    public:
        typedef Eigen::MatrixXd CoefficientMat;
        typedef Eigen::MatrixXd VelCoefficientMat;
        typedef Eigen::MatrixXd AccCoefficientMat;

        double dimension, degree;

    private:
        double duration;
        CoefficientMat coeffMat;

    public:
        Piece() = default;

        Piece(double dur, const CoefficientMat &cMat)
            : duration(dur), coeffMat(cMat), dimension(cMat.rows()), degree(cMat.cols() - 1) {}

        inline int getDim() const
        {
            return 3;
        }

        inline int getDegree() const
        {
            return degree;
        }

        inline double getDuration() const
        {
            return duration;
        }

        inline const CoefficientMat &getCoeffMat() const
        {
            return coeffMat;
        }

        inline Eigen::Vector3d getPos(const double &t) const
        {
            Eigen::Vector3d pos(0.0, 0.0, 0.0);
            double tn = 1.0;
            for (int i = degree; i >= 0; i--)
            {
                pos += tn * coeffMat.col(i);
                tn *= t;
            }
            return pos;
        }

        inline Eigen::Vector3d getVel(const double &t) const
        {
            Eigen::Vector3d vel(0.0, 0.0, 0.0);
            double tn = 1.0;
            int n = 1;
            for (int i = degree - 1; i >= 0; i--)
            {
                vel += n * tn * coeffMat.col(i);
                tn *= t;
                n++;
            }
            return vel;
        }

        inline Eigen::Vector3d getAcc(const double &t) const
        {
            Eigen::Vector3d acc(0.0, 0.0, 0.0);
            double tn = 1.0;
            int m = 1;
            int n = 2;
            for (int i = degree - 2; i >= 0; i--)
            {
                acc += m * n * tn * coeffMat.col(i);
                tn *= t;
                m++;
                n++;
            }
            return acc;
        }

        inline Eigen::Vector3d getJer(const double &t) const
        {
            Eigen::Vector3d jer(0.0, 0.0, 0.0);
            double tn = 1.0;
            int l = 1;
            int m = 2;
            int n = 3;
            for (int i = degree - 3; i >= 0; i--)
            {
                jer += l * m * n * tn * coeffMat.col(i);
                tn *= t;
                l++;
                m++;
                n++;
            }
            return jer;
        }

        inline Eigen::Vector3d getSna(const double &t) const
        {
            Eigen::Vector3d sna(0.0, 0.0, 0.0);
            double tn = 1.0;
            int k = 1;
            int l = 2;
            int m = 3;
            int n = 4;
            for (int i = degree - 3; i >= 0; i--)
            {
                sna += k * l * m * n * tn * coeffMat.col(i);
                tn *= t;
                k++;
                l++;
                m++;
                n++;
            }
            return sna;
        }

        inline Eigen::Vector3d getCra(const double &t) const
        {
            Eigen::Vector3d cra(0.0, 0.0, 0.0);
            double tn = 1.0;
            int j = 1;
            int k = 2;
            int l = 3;
            int m = 4;
            int n = 5;
            for (int i = degree - 3; i >= 0; i--)
            {
                cra += j * k * l * m * n * tn * coeffMat.col(i);
                tn *= t;
                j++;
                k++;
                l++;
                m++;
                n++;
            }
            return cra;
        }

        inline CoefficientMat normalizePosCoeffMat() const
        {
            CoefficientMat nPosCoeffsMat;
            double t = 1.0;
            nPosCoeffsMat.resize(dimension, degree + 1);
            for (int i = degree; i >= 0; i--)
            {
                nPosCoeffsMat.col(i) = coeffMat.col(i) * t;
                t *= duration;
            }
            return nPosCoeffsMat;
        }

        inline VelCoefficientMat normalizeVelCoeffMat() const
        {
            VelCoefficientMat nVelCoeffMat;
            int n = 1;
            double t = duration;
            nVelCoeffMat.resize(dimension, degree);
            for (int i = degree - 1; i >= 0; i--)
            {
                nVelCoeffMat.col(i) = n * coeffMat.col(i) * t;
                t *= duration;
                n++;
            }
            return nVelCoeffMat;
        }

        inline AccCoefficientMat normalizeAccCoeffMat() const
        {
            AccCoefficientMat nAccCoeffMat;
            int n = 2;
            int m = 1;
            double t = duration * duration;
            nAccCoeffMat.resize(dimension, degree - 1);
            for (int i = degree - 2; i >= 0; i--)
            {
                nAccCoeffMat.col(i) = n * m * coeffMat.col(i) * t;
                n++;
                m++;
                t *= duration;
            }
            return nAccCoeffMat;
        }

        inline double getMaxVelRate() const
        {
            VelCoefficientMat nVelCoeffMat = normalizeVelCoeffMat();
            Eigen::VectorXd coeff = RootFinder::polySqr(nVelCoeffMat.row(0)) +
                                    RootFinder::polySqr(nVelCoeffMat.row(1)) +
                                    RootFinder::polySqr(nVelCoeffMat.row(2));
            int N = coeff.size();
            int n = N - 1;

            for (int i = 0; i < N; i++)
            {
                coeff(i) *= n;
                n--;
            }
            if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON)
            {
                return getVel(0.0).norm();
            }
            else
            {
                double l = -0.0625;
                double r = 1.0625;
                while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON)
                {
                    l = 0.5 * l;
                }
                while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON)
                {
                    r = 0.5 * (r + 1.0);
                }
                std::set<double> candidates = RootFinder::solvePolynomial(coeff.head(N - 1), l, r,
                                                                          FLT_EPSILON / duration);
                candidates.insert(0.0);
                candidates.insert(1.0);
                double maxVelRateSqr = -INFINITY;
                double tempNormSqr;
                for (std::set<double>::const_iterator it = candidates.begin();
                     it != candidates.end();
                     it++)
                {
                    if (0.0 <= *it && 1.0 >= *it)
                    {
                        tempNormSqr = getVel((*it) * duration).squaredNorm();
                        maxVelRateSqr = maxVelRateSqr < tempNormSqr ? tempNormSqr : maxVelRateSqr;
                    }
                }
                return sqrt(maxVelRateSqr);
            }
        }

        inline double getMaxAccRate() const
        {
            AccCoefficientMat nAccCoeffMat = normalizeAccCoeffMat();
            Eigen::VectorXd coeff = RootFinder::polySqr(nAccCoeffMat.row(0)) +
                                    RootFinder::polySqr(nAccCoeffMat.row(1)) +
                                    RootFinder::polySqr(nAccCoeffMat.row(2));
            int N = coeff.size();
            int n = N - 1;
            for (int i = 0; i < N; i++)
            {
                coeff(i) *= n;
                n--;
            }
            if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON)
            {
                return getAcc(0.0).norm();
            }
            else
            {
                double l = -0.0625;
                double r = 1.0625;
                while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON)
                {
                    l = 0.5 * l;
                }
                while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON)
                {
                    r = 0.5 * (r + 1.0);
                }
                std::set<double> candidates = RootFinder::solvePolynomial(coeff.head(N - 1), l, r,
                                                                          FLT_EPSILON / duration);
                candidates.insert(0.0);
                candidates.insert(1.0);
                double maxAccRateSqr = -INFINITY;
                double tempNormSqr;
                for (std::set<double>::const_iterator it = candidates.begin();
                     it != candidates.end();
                     it++)
                {
                    if (0.0 <= *it && 1.0 >= *it)
                    {
                        tempNormSqr = getAcc((*it) * duration).squaredNorm();
                        maxAccRateSqr = maxAccRateSqr < tempNormSqr ? tempNormSqr : maxAccRateSqr;
                    }
                }
                return sqrt(maxAccRateSqr);
            }
        }

        inline bool checkMaxVelRate(const double &maxVelRate) const
        {
            double sqrMaxVelRate = maxVelRate * maxVelRate;
            if (getVel(0.0).squaredNorm() >= sqrMaxVelRate ||
                getVel(duration).squaredNorm() >= sqrMaxVelRate)
            {
                return false;
            }
            else
            {
                VelCoefficientMat nVelCoeffMat = normalizeVelCoeffMat();
                Eigen::VectorXd coeff = RootFinder::polySqr(nVelCoeffMat.row(0)) +
                                        RootFinder::polySqr(nVelCoeffMat.row(1)) +
                                        RootFinder::polySqr(nVelCoeffMat.row(2));
                double t2 = duration * duration;
                coeff.tail<1>()(0) -= sqrMaxVelRate * t2;
                return RootFinder::countRoots(coeff, 0.0, 1.0) == 0;
            }
        }

        inline bool checkMaxAccRate(const double &maxAccRate) const
        {
            double sqrMaxAccRate = maxAccRate * maxAccRate;
            if (getAcc(0.0).squaredNorm() >= sqrMaxAccRate ||
                getAcc(duration).squaredNorm() >= sqrMaxAccRate)
            {
                return false;
            }
            else
            {
                AccCoefficientMat nAccCoeffMat = normalizeAccCoeffMat();
                Eigen::VectorXd coeff = RootFinder::polySqr(nAccCoeffMat.row(0)) +
                                        RootFinder::polySqr(nAccCoeffMat.row(1)) +
                                        RootFinder::polySqr(nAccCoeffMat.row(2));
                double t2 = duration * duration;
                double t4 = t2 * t2;
                coeff.tail<1>()(0) -= sqrMaxAccRate * t4;
                return RootFinder::countRoots(coeff, 0.0, 1.0) == 0;
            }
        }
    };

    class Trajectory
    {
    private:
        typedef std::vector<Piece> Pieces;
        Pieces pieces;

    public:
        int order_;

        Trajectory() = default;

        Trajectory(const std::vector<double> &durs,
                   const std::vector<Piece::CoefficientMat> &cMats);

        typedef std::shared_ptr<Trajectory> Ptr;

        inline void init(const int order)
        {
            order_ = order;
        }

        inline int getPieceNum() const
        {
            return pieces.size();
        }

        inline Piece getPiece(const int i) const
        {
            return pieces[i];
        }

        inline Eigen::VectorXd getDurations() const
        {
            int N = getPieceNum();
            Eigen::VectorXd durations(N);
            for (int i = 0; i < N; i++)
            {
                durations(i) = pieces[i].getDuration();
            }
            return durations;
        }

        inline double getTotalDuration() const
        {
            int N = getPieceNum();
            double totalDuration = 0.0;
            for (int i = 0; i < N; i++)
            {
                totalDuration += pieces[i].getDuration();
            }
            return totalDuration;
        }

        inline Eigen::Matrix3Xd getPositions() const
        {
            int N = getPieceNum();
            Eigen::Matrix3Xd positions(3, N + 1);
            for (int i = 0; i < N; i++)
            {
                positions.col(i) = pieces[i].getCoeffMat().col(order_);
            }
            positions.col(N) = pieces[N - 1].getPos(pieces[N - 1].getDuration());
            return positions;
        }

        inline const Piece &operator[](int i) const
        {
            return pieces[i];
        }

        inline Piece &operator[](int i)
        {
            return pieces[i];
        }

        inline void clear(void)
        {
            pieces.clear();
            return;
        }

        inline typename Pieces::const_iterator begin() const
        {
            return pieces.begin();
        }

        inline typename Pieces::const_iterator end() const
        {
            return pieces.end();
        }

        inline typename Pieces::iterator begin()
        {
            return pieces.begin();
        }

        inline typename Pieces::iterator end()
        {
            return pieces.end();
        }

        inline void reserve(const int &n)
        {
            pieces.reserve(n);
            return;
        }

        inline void emplace_back(const Piece &piece)
        {
            pieces.emplace_back(piece);
            return;
        }

        inline void emplace_back(const double &dur,
                                 const Piece::CoefficientMat &cMat)
        {
            pieces.emplace_back(dur, cMat);
            return;
        }

        inline void append(const Trajectory &traj)
        {
            pieces.insert(pieces.end(), traj.begin(), traj.end());
            return;
        }

        inline int locatePieceIdx(double &t) const
        {
            int N = getPieceNum();
            int idx;
            double dur;
            for (idx = 0;
                 idx < N &&
                 t > (dur = pieces[idx].getDuration());
                 idx++)
            {
                t -= dur;
            }
            if (idx == N)
            {
                idx--;
                t += pieces[idx].getDuration();
            }
            return idx;
        }

        inline Eigen::Vector3d getPos(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getPos(t);
        }

        inline Eigen::Vector3d getVel(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getVel(t);
        }

        inline Eigen::Vector3d getAcc(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getAcc(t);
        }

        inline Eigen::Vector3d getJer(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getJer(t);
        }

        inline Eigen::Vector3d getSna(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getSna(t);
        }

        inline Eigen::Vector3d getCra(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getCra(t);
        }

        inline Eigen::Vector3d getJuncPos(int juncIdx) const
        {
            if (juncIdx != getPieceNum())
            {
                return pieces[juncIdx].getCoeffMat().col(order_);
            }
            else
            {
                return pieces[juncIdx - 1].getPos(pieces[juncIdx - 1].getDuration());
            }
        }

        inline Eigen::Vector3d getJuncVel(int juncIdx) const
        {
            if (juncIdx != getPieceNum())
            {
                return pieces[juncIdx].getCoeffMat().col(order_ - 1);
            }
            else
            {
                return pieces[juncIdx - 1].getVel(pieces[juncIdx - 1].getDuration());
            }
        }

        inline Eigen::Vector3d getJuncAcc(int juncIdx) const
        {
            if (juncIdx != getPieceNum())
            {
                return pieces[juncIdx].getCoeffMat().col(order_ - 2) * 2.0;
            }
            else
            {
                return pieces[juncIdx - 1].getAcc(pieces[juncIdx - 1].getDuration());
            }
        }

        inline double getMaxVelRate() const
        {
            int N = getPieceNum();
            double maxVelRate = -INFINITY;
            double tempNorm;
            for (int i = 0; i < N; i++)
            {
                tempNorm = pieces[i].getMaxVelRate();
                maxVelRate = maxVelRate < tempNorm ? tempNorm : maxVelRate;
            }
            return maxVelRate;
        }

        inline double getMaxAccRate() const
        {
            int N = getPieceNum();
            double maxAccRate = -INFINITY;
            double tempNorm;
            for (int i = 0; i < N; i++)
            {
                tempNorm = pieces[i].getMaxAccRate();
                maxAccRate = maxAccRate < tempNorm ? tempNorm : maxAccRate;
            }
            return maxAccRate;
        }

        inline bool checkMaxVelRate(const double &maxVelRate) const
        {
            int N = getPieceNum();
            bool feasible = true;
            for (int i = 0; i < N && feasible; i++)
            {
                feasible = feasible && pieces[i].checkMaxVelRate(maxVelRate);
            }
            return feasible;
        }

        inline bool checkMaxAccRate(const double &maxAccRate) const
        {
            int N = getPieceNum();
            bool feasible = true;
            for (int i = 0; i < N && feasible; i++)
            {
                feasible = feasible && pieces[i].checkMaxAccRate(maxAccRate);
            }
            return feasible;
        }
    };
}

#endif