/**
 * @file mpc_wrapper.h
 * @author Haojia Li (hlied@connect.ust.hk)
 * @brief MPC interface. Wrapper for ACADO.
 * Thanks for rpg_mpc
 * @version 1.0
 * @date 2022-07-09
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef MPC_WRAPPER_H
#define MPC_WRAPPER_H

#include <Eigen/Eigen>
#include <ros/ros.h>

namespace PayloadMPC
{

#include "acado_auxiliary_functions.h"
#include "acado_common.h"

    static constexpr int kSamples = ACADO_N;              // number of samples
    static constexpr int kStateSize = ACADO_NX;           // number of states
    static constexpr int kRefSize = ACADO_NY;             // number of reference states
    static constexpr int kEndRefSize = ACADO_NYN;         // number of end reference states
    static constexpr int kInputSize = ACADO_NU;           // number of inputs
    static constexpr int kCostSize = ACADO_NY - ACADO_NU; // number of state costs
    static constexpr int kOdSize = ACADO_NOD;             // number of online data
    // static constexpr real_t dt_{0.05};                    // time step

    // extern ACADOvariables acadoVariables;
    // extern ACADOworkspace acadoWorkspace;

    class MpcWrapper
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        MpcWrapper();
        // MpcWrapper(
        //     const Eigen::Ref<const Eigen::Matrix<real_t, kCostSize, kCostSize>> &Q,
        //     const Eigen::Ref<const Eigen::Matrix<real_t, kInputSize, kInputSize>> &R,
        //     const Eigen::Ref<const Eigen::Matrix<real_t, kStateSize, 1>> &initial_state,
        //     const Eigen::Ref<const Eigen::Matrix<real_t, kInputSize, 1>> &initial_input,
        //     real_t state_cost_scaling = 0.0, real_t input_cost_scaling = 0.0);
        void initialize(
            const Eigen::Ref<const Eigen::Matrix<real_t, kCostSize, kCostSize>> &Q,
            const Eigen::Ref<const Eigen::Matrix<real_t, kInputSize, kInputSize>> &R,
            const Eigen::Ref<const Eigen::Matrix<real_t, kStateSize, 1>> &initial_state,
            const Eigen::Ref<const Eigen::Matrix<real_t, kInputSize, 1>> &initial_input,
            real_t state_cost_scaling,
            real_t input_cost_scaling);
        bool setCosts(
            const Eigen::Ref<const Eigen::Matrix<real_t, kCostSize, kCostSize>> &Q,
            const Eigen::Ref<const Eigen::Matrix<real_t, kInputSize, kInputSize>> &R,
            real_t state_cost_scaling = 0.0, real_t input_cost_scaling = 0.0);

        bool setLimits(real_t min_thrust, real_t max_thrust,
                       real_t max_rollpitchrate, real_t max_yawrate);

        bool setReferencePose(
            const Eigen::Ref<const Eigen::Matrix<real_t, kStateSize, 1>> reference_state);
        bool setTrajectory(
            const Eigen::Ref<const Eigen::Matrix<real_t, kStateSize, kSamples + 1>> reference_states,
            const Eigen::Ref<const Eigen::Matrix<real_t, kInputSize, kSamples + 1>> inputs);

        bool solve(const Eigen::Ref<const Eigen::Matrix<real_t, kStateSize, 1>> state);
        bool update(const Eigen::Ref<const Eigen::Matrix<real_t, kStateSize, 1>> state,
                    bool do_preparation = true);
        bool prepare();
        void setDynamicParams(const real_t mass_q, const real_t mass_l, const real_t l_length); // Must use once before initialize
        void setExternalForce(const Eigen::Ref<const Eigen::Vector3d>& fl, const Eigen::Ref<const Eigen::Vector3d>& fq);
        void getState(const int node_index,
                      Eigen::Ref<Eigen::Matrix<real_t, kStateSize, 1>> return_state);
        void getStates(
            Eigen::Ref<Eigen::Matrix<real_t, kStateSize, kSamples + 1>> return_states);
        void getInput(const int node_index,
                      Eigen::Ref<Eigen::Matrix<real_t, kInputSize, 1>> return_input);
        void getInputs(
            Eigen::Ref<Eigen::Matrix<real_t, kInputSize, kSamples>> return_input);
        // real_t getTimestep() { return dt_; }

    private:
        Eigen::Map<Eigen::Matrix<real_t, kRefSize, kSamples, Eigen::ColMajor>>
            acado_reference_states_{acadoVariables.y};

        Eigen::Map<Eigen::Matrix<real_t, kEndRefSize, 1, Eigen::ColMajor>>
            acado_reference_end_state_{acadoVariables.yN};

        Eigen::Map<Eigen::Matrix<real_t, kStateSize, 1, Eigen::ColMajor>>
            acado_initial_state_{acadoVariables.x0};

        Eigen::Map<Eigen::Matrix<real_t, kStateSize, kSamples + 1, Eigen::ColMajor>>
            acado_states_{acadoVariables.x};

        Eigen::Map<Eigen::Matrix<real_t, kInputSize, kSamples, Eigen::ColMajor>>
            acado_inputs_{acadoVariables.u};

        Eigen::Map<Eigen::Matrix<real_t, kOdSize, kSamples+1, Eigen::ColMajor>>
          acado_online_data_{acadoVariables.od};

        Eigen::Map<Eigen::Matrix<real_t, kRefSize, kRefSize * kSamples>>
            acado_W_{acadoVariables.W};

        Eigen::Map<Eigen::Matrix<real_t, kEndRefSize, kEndRefSize>>
            acado_W_end_{acadoVariables.WN};

        Eigen::Map<Eigen::Matrix<real_t, 4, kSamples, Eigen::ColMajor>>
            acado_lower_bounds_{acadoVariables.lbValues};

        Eigen::Map<Eigen::Matrix<real_t, 4, kSamples, Eigen::ColMajor>>
            acado_upper_bounds_{acadoVariables.ubValues};

        // Eigen::Map<Eigen::Matrix<real_t, 1, kSamples>>
        //   acado_lower_affine_bounds_{acadoVariables.lbAValues};

        // Eigen::Map<Eigen::Matrix<real_t, 1, kSamples>>
        //   acado_upper_affine_bounds_{acadoVariables.ubAValues};

        Eigen::Matrix<real_t, kRefSize, kRefSize> W_ = (Eigen::Matrix<real_t, kRefSize, kRefSize>::Identity());

        Eigen::Matrix<real_t, kEndRefSize, kEndRefSize> WN_ =
            W_.block(0, 0, kEndRefSize, kEndRefSize);

        bool acado_is_prepared_{false};
        Eigen::Matrix<real_t, kInputSize, 1> kHoverInput_;

        real_t mass_q_{1.0}, mass_l_{0.1}, l_length_{0.5};
    };

} // namespace MPC
#endif