/************************************************************************
 * Date:        2023.02
 * Author:      Haokun Wang <hwangeh at connect dot ust dot hk>, 
                Aerial Robotics Group <https://uav.ust.hk>, HKUST.
 * E-mail:      hwangeh_at_connect_dot_ust_dot_hk
 * Description: This is the source file for the MINCO class, which is used
 *              to generate the minimum energy trajectory for the UAV with
 *              suspended payloads.
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

#include "alm/minco.hpp"

namespace minco {
MINCO::MINCO(const int& order, const int& dim) {
	traj_order_ = order;
	state_dim_ = dim;
	coefficient_num_ = order + 1;
}

double MINCO::factorial(double n) {
	if (n == 1 || n == 0)
		return 1;
	else
		return n * factorial(n - 1);
}

void MINCO::get_q(const int i, const int j, const int s, int& order,
                  double& result) {
	if (i > s and j > s) {
		order = i + j - 2 * s - 1;
		result =
		    (factorial(i - 1) * factorial(j - 1)) /
		    (factorial(i - s - 1) * factorial(j - s - 1) * (i + j - 2 * s - 1));
	} else {
		order = -1;
		result = 0;
	}
	return;
}

void MINCO::get_r(const int i, const int j, const int s, int& order,
                  double& result) {
	if (i > s and j > s) {
		order = i + j - 2 * s - 2;
		result = (factorial(i - 1) * factorial(j - 1)) /
		         (factorial(i - s - 1) * factorial(j - s - 1));
	} else {
		order = -1;
		result = 0;
	}
	return;
}

void MINCO::get_order(const int& order, const int& total_order,
                      Eigen::VectorXd& coeff_list) {
	coeff_list.resize(total_order + 1);
	for (int i = 0; i < total_order + 1; i++) {
		if (i < order) {
			coeff_list(i) = 0.0;
		} else {
			coeff_list(i) = factorial(i) / factorial(i - order);
		}
	}
	return;
}

void MINCO::setConditions(const Eigen::Matrix<double, 6, 4>& kHeadState,
                          const Eigen::Matrix<double, 6, 4>& kTailState,
                          const int& kPieceNum) {
	N = kPieceNum;
	// bound_conditions_num_ = kHeadState.cols();
	bound_conditions_num_ = (traj_order_ + 1) / 2;

	// headPVAJ.resize(kHeadState.rows(), kHeadState.cols());
	// tailPVAJ.resize(kTailState.rows(), kTailState.cols());
	headPVAJ.resize(state_dim_, bound_conditions_num_);
	tailPVAJ.resize(state_dim_, bound_conditions_num_);
	headPVAJ.setZero();
	tailPVAJ.setZero();
	headPVAJ.leftCols(bound_conditions_num_) =
	    kHeadState.leftCols(bound_conditions_num_);
	tailPVAJ.leftCols(bound_conditions_num_) =
	    kTailState.leftCols(bound_conditions_num_);

	A.create(coefficient_num_ * N, coefficient_num_, coefficient_num_);
	b.resize(coefficient_num_ * N, state_dim_);
	t_.resize(traj_order_, N);
	return;
}

void MINCO::setParameters(const Eigen::MatrixXd& kInnerPoints,
                          const Eigen::VectorXd& kTimeAllocation) {
	t_.row(0) = kTimeAllocation;
	for (int i = 1; i < traj_order_; i++) {
		t_.row(i) = t_.row(i - 1).cwiseProduct(t_.row(0));
	}

	A.reset();
	b.setZero();

	for (int i = 0; i < bound_conditions_num_; i++) {
		A(i, i) = factorial(i);
		b.row(i) = headPVAJ.col(i).transpose();
	}

	for (int i = 0; i < N - 1; i++) {
		// high order continuous
		for (int j = 0; j < (coefficient_num_ / 2) - 1; j++) {
			for (int k = 0; k < (coefficient_num_ / 2); k++) {
				if (j == k) {
					A(i * coefficient_num_ + bound_conditions_num_ + j,
					  i * coefficient_num_ + bound_conditions_num_ + k) =
					    factorial(bound_conditions_num_ + k);
				}
				if (j < k) {
					A(i * coefficient_num_ + bound_conditions_num_ + j,
					  i * coefficient_num_ + bound_conditions_num_ + k) =
					    factorial(bound_conditions_num_ + k) /
					    factorial(k - j) * t_(k - j - 1, i);
				}
			}
			A(i * coefficient_num_ + bound_conditions_num_ + j,
			  i * coefficient_num_ + bound_conditions_num_ + j +
			      coefficient_num_) = -factorial(bound_conditions_num_ + j);
		}

		// inner position constraint
		for (int j = 0; j < coefficient_num_; j++) {
			if (j == 0) {
				A(i * coefficient_num_ + bound_conditions_num_ +
				      (coefficient_num_ / 2) - 1,
				  i * coefficient_num_) = 1.0;
			} else {
				A(i * coefficient_num_ + bound_conditions_num_ +
				      (coefficient_num_ / 2) - 1,
				  i * coefficient_num_ + j) = t_(j - 1, i);
			}
		}

		// low order continous
		for (int j = 0; j < (coefficient_num_ / 2); j++) {

			for (int k = 0; k < coefficient_num_; k++) {
				if (j == k) {
					A(i * coefficient_num_ + bound_conditions_num_ +
					      (coefficient_num_ / 2) + j,
					  i * coefficient_num_ + k) = factorial(k);
				}
				if (j < k) {
					A(i * coefficient_num_ + bound_conditions_num_ +
					      (coefficient_num_ / 2) + j,
					  i * coefficient_num_ + k) =
					    factorial(k) / factorial(k - j) * t_(k - j - 1, i);
				}
			}
			A(i * coefficient_num_ + bound_conditions_num_ +
			      (coefficient_num_ / 2) + j,
			  i * coefficient_num_ + bound_conditions_num_ +
			      (coefficient_num_ / 2) + j) = -factorial(j);
		}
		// b parameters
		b.row(i * coefficient_num_ + coefficient_num_ - 1) =
		    kInnerPoints.col(i).transpose();
	}

	for (int i = (coefficient_num_ / 2); i > 0; i--) {
		for (int j = coefficient_num_; j > 0; j--) {
			if ((j - i) == (coefficient_num_ / 2)) {
				A(coefficient_num_ * N - i, coefficient_num_ * N - j) =
				    factorial(coefficient_num_ - j);
			}
			if ((j - i) < (coefficient_num_ / 2)) {
				A(coefficient_num_ * N - i, coefficient_num_ * N - j) =
				    factorial(coefficient_num_ - j) /
				    factorial((coefficient_num_ / 2) - j + i) *
				    t_((coefficient_num_ / 2) - j + i - 1, N - 1);
			}
		}
		b.row(coefficient_num_ * N - i) =
		    tailPVAJ.col((coefficient_num_ / 2) - i).transpose();
	}

	A.factorizeLU();
	A.solve(b);

	return;
}

void MINCO::getTrajectory(poly_traj::Trajectory::Ptr& load_traj,
                          poly_traj::Trajectory::Ptr& quad_traj) {
	load_traj->clear();
	load_traj->reserve(N);
	quad_traj->clear();
	quad_traj->reserve(N);
	Eigen::VectorXd T1 = t_.row(0);
	for (int i = 0; i < N; i++) {
		load_traj->emplace_back(
		    T1(i), b.block(coefficient_num_ * i, 0, coefficient_num_, 3)
		               .transpose()
		               .rowwise()
		               .reverse());
		quad_traj->emplace_back(
		    T1(i), b.block(coefficient_num_ * i, 3, coefficient_num_, 3)
		               .transpose()
		               .rowwise()
		               .reverse());
	}
	return;
}

void MINCO::getEnergy(double& energy) {
	int order = -1;
	double coeff = 0;
	energy = 0;
	Eigen::Vector3d temp1_(0, 0, 0);
	Eigen::Vector3d temp2_(0, 0, 0);
	// Eigen::VectorXd temp1_;
	// Eigen::VectorXd temp2_;

	for (int i = 0; i < N; i++) {
		for (int j = 0; j < coefficient_num_; j++) {
			for (int k = 0; k < coefficient_num_; k++) {
				get_q(j + 1, k + 1, coefficient_num_ / 2, order, coeff);
				if (order != -1) {
					temp1_ = b.row(i * coefficient_num_ + j).tail(3);
					temp2_ = b.row(i * coefficient_num_ + k).tail(3);
					if (j == k) {

						energy +=
						    coeff * temp1_.squaredNorm() * t_(order - 1, i);
					} else {
						energy += coeff * temp1_.dot(temp2_) * t_(order - 1, i);
					}
				}
			}
		}
	}
	return;
}

void MINCO::getEnergyPartialGradByCoeffs(Eigen::MatrixXd& gdC) {
	gdC.resize(coefficient_num_ * N, state_dim_);
	gdC.setZero();
	int order = -1;
	double coeff = 0;
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < coefficient_num_; j++) {
			for (int k = 0; k < coefficient_num_; k++) {
				get_q(j + 1, k + 1, coefficient_num_ / 2, order, coeff);
				if (j >= coefficient_num_ / 2 and k >= coefficient_num_ / 2) {
					gdC.row(i * coefficient_num_ + j) +=
					    2 * coeff * b.row(i * coefficient_num_ + k) *
					    t_(j + k - coefficient_num_, i);
					gdC.row(i * coefficient_num_ + j).head(3).setZero();
					// ROS_WARN("gdC.row(8*i + %d) += coeff:%f, b.row(8*i +
					// %d)*T%d(i)");
				}
			}
		}
	}
	return;
}

void MINCO::getEnergyPartialGradByTimes(Eigen::VectorXd& gdT) {
	gdT.resize(N);
	int order = -1;
	double coeff = 0;
	Eigen::Vector3d temp1_(0, 0, 0);
	Eigen::Vector3d temp2_(0, 0, 0);
	// Eigen::VectorXd temp1_;
	// Eigen::VectorXd temp2_;
	for (int i = 0; i < N; i++) {
		gdT(i) = 0;
		for (int j = 0; j < coefficient_num_; j++) {
			for (int k = 0; k < coefficient_num_; k++) {
				get_r(j + 1, k + 1, coefficient_num_ / 2, order, coeff);
				temp1_ = b.row(i * coefficient_num_ + j).tail(3);
				temp2_ = b.row(i * coefficient_num_ + k).tail(3);
				if (order == 0) {
					gdT(i) += coeff * temp1_.squaredNorm();
				}
				if (order > 0) {
					if (j == k) {
						gdT(i) +=
						    coeff * temp1_.squaredNorm() * t_(order - 1, i);
					} else {
						gdT(i) += coeff * temp1_.dot(temp2_) * t_(order - 1, i);
					}
				}
			}
		}
	}
	return;
}

void MINCO::propogateGrad(const Eigen::MatrixXd& partialGradByCoeffs,
                          const Eigen::VectorXd& partialGradByTimes,
                          Eigen::MatrixXd& gradByPoints,
                          Eigen::VectorXd& gradByTimes) {
	gradByPoints.resize(state_dim_, N - 1);
	gradByTimes.resize(N);
	Eigen::MatrixXd adjGrad = partialGradByCoeffs;
	A.solveAdj(adjGrad);

	for (int i = 0; i < N - 1; i++) {
		gradByPoints.col(i) =
		    adjGrad.row(coefficient_num_ * i + coefficient_num_ - 1)
		        .transpose();
	}

	Eigen::MatrixXd B1;
	Eigen::MatrixXd B2;
	B1.resize(coefficient_num_, state_dim_);
	B2.resize(coefficient_num_ / 2, state_dim_);

	Eigen::VectorXd coeff_list;
	for (int i = 0; i < N - 1; i++) {
		B1.setZero();
		for (int j = 0; j < coefficient_num_; j++) {
			coeff_list.setZero();
			if (j == 0) {
				get_order(j + 1, traj_order_, coeff_list);
				for (int k = 0; k < coeff_list.size(); k++) {
					if (k <= j + 1) {
						B1.row((j + coefficient_num_ / 2 - 1) %
						       coefficient_num_) +=
						    -coeff_list(k) * b.row(i * coefficient_num_ + k);
					} else {
						B1.row((j + coefficient_num_ / 2 - 1) %
						       coefficient_num_) +=
						    -coeff_list(k) * t_(k - 2, i) *
						    b.row(i * coefficient_num_ + k);
					}
					// ROS_WARN("B1.row(%d), coeff: %f, T%d, b.row(%d)",
					// (j+coefficient_num_/2-1)%coefficient_num_, coeff_list(k),
					// k-1, k);
				}
			} else if (j == 1) {
				B1.row((j + coefficient_num_ / 2 - 1) % coefficient_num_) =
				    B1.row((j - 1 + coefficient_num_ / 2 - 1) %
				           coefficient_num_);
				// ROS_WARN("B1.row(%d) =  B1.row(%d)",
				// (j+coefficient_num_/2-1)%coefficient_num_,
				// (j-1+coefficient_num_/2-1)%coefficient_num_);
			} else {
				get_order(j, traj_order_, coeff_list);
				for (int k = 0; k < coeff_list.size(); k++) {
					if (k <= j) {
						B1.row((j + coefficient_num_ / 2 - 1) %
						       coefficient_num_) +=
						    -coeff_list(k) * b.row(i * coefficient_num_ + k);
					} else {
						B1.row((j + coefficient_num_ / 2 - 1) %
						       coefficient_num_) +=
						    -coeff_list(k) * t_(k - j - 1, i) *
						    b.row(i * coefficient_num_ + k);
					}
					// ROS_WARN("B1.row(%d), coeff: %f, T%d, b.row(%d)",
					// (j+coefficient_num_/2-1)%coefficient_num_, coeff_list(k),
					// k-j, k);
				}
			}
		}
		gradByTimes(i) =
		    B1.cwiseProduct(
		          adjGrad.block(coefficient_num_ * i + coefficient_num_ / 2, 0,
		                        coefficient_num_, state_dim_))
		        .sum();
	}
	B2.setZero();
	for (int i = 0; i < coefficient_num_ / 2; i++) {
		coeff_list.setZero();
		get_order(i + 1, traj_order_, coeff_list);
		for (int j = coefficient_num_; j > 0; j--) {
			if (j >= coefficient_num_ - i - 1) {
				B2.row(i) += -coeff_list(coefficient_num_ - j) *
				             b.row(coefficient_num_ * N - j);
			} else {
				B2.row(i) += -coeff_list(coefficient_num_ - j) *
				             t_(coefficient_num_ - i - j - 2, N - 1) *
				             b.row(coefficient_num_ * N - j);
			}
			// ROS_WARN("B2.row(%d), coeff: %f, T%d, b.row(%d)", i,
			// coeff_list(coefficient_num_ - j), coefficient_num_-i-j-1, j);
		}
	}
	gradByTimes(N - 1) =
	    B2.cwiseProduct(
	          adjGrad.block(coefficient_num_ * N - coefficient_num_ / 2, 0,
	                        coefficient_num_ / 2, state_dim_))
	        .sum();
	gradByTimes += partialGradByTimes;
}

void BandedSystem::create(const int& n, const int& p, const int& q) {
	// In case of re-creating before destroying
	destroy();
	N = n;
	lowerBw = p;
	upperBw = q;
	int actualSize = N * (lowerBw + upperBw + 1);
	ptrData = new double[actualSize];
	std::fill_n(ptrData, actualSize, 0.0);
	return;
}

void BandedSystem::destroy(void) {
	if (ptrData != nullptr) {
		delete[] ptrData;
		ptrData = nullptr;
	}
	return;
}

void BandedSystem::reset(void) {
	std::fill_n(ptrData, N * (lowerBw + upperBw + 1), 0.0);
	return;
}

void BandedSystem::factorizeLU(void) {
	int iM, jM;
	double cVl;
	for (int k = 0; k <= N - 2; k++) {
		iM = std::min(k + lowerBw, N - 1);
		cVl = operator()(k, k);
		for (int i = k + 1; i <= iM; i++) {
			if (operator()(i, k) != 0.0) {
				operator()(i, k) /= cVl;
			}
		}
		jM = std::min(k + upperBw, N - 1);
		for (int j = k + 1; j <= jM; j++) {
			cVl = operator()(k, j);
			if (cVl != 0.0) {
				for (int i = k + 1; i <= iM; i++) {
					if (operator()(i, k) != 0.0) {
						operator()(i, j) -= operator()(i, k) * cVl;
					}
				}
			}
		}
	}
	return;
}

template <typename EIGENMAT> void BandedSystem::solve(EIGENMAT& b) {
	int iM;
	for (int j = 0; j <= N - 1; j++) {
		iM = std::min(j + lowerBw, N - 1);
		for (int i = j + 1; i <= iM; i++) {
			if (operator()(i, j) != 0.0) {
				b.row(i) -= operator()(i, j) * b.row(j);
			}
		}
	}
	for (int j = N - 1; j >= 0; j--) {
		b.row(j) /= operator()(j, j);
		iM = std::max(0, j - upperBw);
		for (int i = iM; i <= j - 1; i++) {
			if (operator()(i, j) != 0.0) {
				b.row(i) -= operator()(i, j) * b.row(j);
			}
		}
	}
	return;
}

template <typename EIGENMAT> void BandedSystem::solveAdj(EIGENMAT& b) {
	int iM;
	for (int j = 0; j <= N - 1; j++) {
		b.row(j) /= operator()(j, j);
		iM = std::min(j + upperBw, N - 1);
		for (int i = j + 1; i <= iM; i++) {
			if (operator()(j, i) != 0.0) {
				b.row(i) -= operator()(j, i) * b.row(j);
			}
		}
	}
	for (int j = N - 1; j >= 0; j--) {
		iM = std::max(0, j - lowerBw);
		for (int i = iM; i <= j - 1; i++) {
			if (operator()(j, i) != 0.0) {
				b.row(i) -= operator()(j, i) * b.row(j);
			}
		}
	}
	return;
}
} // namespace minco