#include <memory>
#include <math.h>
#include <acado_optimal_control.hpp>
#include <acado_code_generation.hpp>
#include <acado_gnuplot.hpp>
#include <Yaml.hpp>

// Standalone code generation for a parameter-free quadrotor model
// with thrust and rates input.
#define RED "\033[31m"    /* Red */
#define YELLOW "\033[33m" /* Yellow */
#define RESET "\033[0m"

// if you want use your own parameters, run the program with your own parameter's file path. Such as,
//  ./quadrotor_payload_mpc_codegen ../config/mpc.yaml
int main(int argc, char **argv)
{
  // Use Acado
  USING_NAMESPACE_ACADO

  /*
  Switch between code generation and analysis.

  If CODE_GEN is true the system is compiled into an optimizaiton problem
  for real-time iteration and all code to run it online is generated.
  Constraints and reference structure is used but the values will be set on
  runtinme.

  If CODE_GEN is false, the system is compiled into a standalone optimization
  and solved on execution. The reference and constraints must be set in here.
  */
  const bool CODE_GEN = true;

  std::string cfg_file_path = "../config/model.yaml";
  if (argc > 1)
    cfg_file_path = argv[1];
  Yaml::Node cfg_root;

  try
  {
    Yaml::Parse(cfg_root, cfg_file_path.data());
  }
  catch (const std::exception &e)
  {
    std::cerr << RED << e.what() << std::endl;
  }
  std::cout << "Parameters file path: " << cfg_file_path << RESET << std::endl
            << std::endl;

  // System variables
  DifferentialState p_x, p_y, p_z;
  DifferentialState q_w, q_x, q_y, q_z;
  DifferentialState v_x, v_y, v_z;
  // DifferentialState payload_x, payload_y, payload_z;
  // DifferentialState payload_vx, payload_vy, payload_vz;
  DifferentialState cable_x, cable_y, cable_z;
  DifferentialState dcable_x, dcable_y, dcable_z;

  // IntermediateState dcable_sqr, q_sqr;
  // IntermediateState qwqy_plus_qxqz, qwqx_minus_qyqz;
  // IntermediateState     cos_cable_angle;

  Control T, w_x, w_y, w_z;
  DifferentialEquation f;
  Function h, hN;
  // OnlineData            p_F_x, p_F_y, p_F_z;
  // OnlineData            t_B_C_x, t_B_C_y, t_B_C_z;
  // OnlineData            q_B_C_w, q_B_C_x, q_B_C_y, q_B_C_z;

  // Parameters with exemplary values. These are set/overwritten at runtime.
  const double w_max_yaw = 1;                       // Maximal yaw rate [rad/s]
  const double w_max_xy = 5;                        // Maximal pitch and roll rate [rad/s]
  const double T_min = 4.0;                         // Minimal total thrust [N]
  const double T_max = 40.0;                        // Maximal total thrust [N]
  const double K_min_cos_cable_angle = cos(M_PI_4); // Minimal cos cable angle (cable.dot(gravity) => K_max_cable_angle)
  const double K_max_cos_cable_angle = 1.0;

  // dynamics
  double Mq = cfg_root["mass_q"].As<double>(0.98);        // Quadrotor mass[kg] default 0.98
  double Ml = cfg_root["mass_l"].As<double>(0.2);         // payload mass [kg] default 0.2
  double l_length = cfg_root["l_length"].As<double>(0.6); // cable length [m]  default 0.6
  double g_z = cfg_root["gravity"].As<double>(9.81);                          // Gravity is everywhere [m/s^2]
  double dt = cfg_root["step_T"].As<double>(0.05);                           // Discretization time [s]
  int N = cfg_root["step_N"].As<int>(20);                  // Number of nodes
  std::cout << YELLOW;
  std::cout << "Mq:       " << Mq << std::endl;
  std::cout << "Ml:       " << Ml << std::endl;
  std::cout << "l_length: " << l_length << std::endl;
  std::cout << "g_z:      " << g_z << std::endl;
  std::cout << "N:        " << N << std::endl;
  std::cout << "dt:       " << dt << std::endl;
  std::cout << RESET << std::endl;

  // dcable_sqr = dcable_x*dcable_x+ dcable_y*dcable_y+ dcable_z*dcable_z;
  // q_sqr = q_w*q_w - q_x*q_x- q_y*q_y + q_z*q_z;
  // qwqy_plus_qxqz = 2*(q_w*q_y + q_x*q_z);
  // qwqx_minus_qyqz = 2*(q_w*q_x - q_y*q_z);

  f << dot(p_x) == v_x;
  f << dot(p_y) == v_y;
  f << dot(p_z) == v_z;

  f << dot(q_w) == 0.5 * (-w_x * q_x - w_y * q_y - w_z * q_z);
  f << dot(q_x) == 0.5 * (w_x * q_w + w_z * q_y - w_y * q_z);
  f << dot(q_y) == 0.5 * (w_y * q_w - w_z * q_x + w_x * q_z);
  f << dot(q_z) == 0.5 * (w_z * q_w + w_y * q_x - w_x * q_y);

  f << dot(v_x) == (2 * (q_w * q_y + q_x * q_z) * T - (Ml * cable_x * (T * cable_z * (q_w * q_w - q_x * q_x - q_y * q_y + q_z * q_z) - Mq * l_length * (dcable_x * dcable_x + dcable_y * dcable_y + dcable_z * dcable_z) - T * cable_y * (2 * (q_w * q_x - q_y * q_z)) + T * cable_x * (2 * (q_w * q_y + q_x * q_z)))) / (Ml + Mq)) / Mq;
  f << dot(v_y) == -(T * (2 * (q_w * q_x - q_y * q_z)) + (Ml * cable_y * (T * cable_z * (q_w * q_w - q_x * q_x - q_y * q_y + q_z * q_z) - Mq * l_length * (dcable_x * dcable_x + dcable_y * dcable_y + dcable_z * dcable_z) - T * cable_y * (2 * (q_w * q_x - q_y * q_z)) + T * cable_x * (2 * (q_w * q_y + q_x * q_z)))) / (Ml + Mq)) / Mq;
  f << dot(v_z) == (T * (q_w * q_w - q_x * q_x - q_y * q_y + q_z * q_z) + Ml * (g_z - (cable_z * (T * cable_z * (q_w * q_w - q_x * q_x - q_y * q_y + q_z * q_z) - Mq * l_length * (dcable_x * dcable_x + dcable_y * dcable_y + dcable_z * dcable_z) - T * cable_y * (2 * (q_w * q_x - q_y * q_z)) + T * cable_x * (2 * (q_w * q_y + q_x * q_z)))) / (Ml + Mq)) - g_z * (Ml + Mq)) / Mq;

  // f << dot(payload_x) == payload_vx;
  // f << dot(payload_y) == payload_vy;
  // f << dot(payload_z) == payload_vz;

  // f << dot(payload_vx) == (cable_x * (T * cable_z * (q_w * q_w - q_x * q_x - q_y * q_y + q_z * q_z) - Mq * l_length * (dcable_x * dcable_x + dcable_y * dcable_y + dcable_z * dcable_z) - T * cable_y * (2 * (q_w * q_x - q_y * q_z)) + T * cable_x * (2 * (q_w * q_y + q_x * q_z)))) / (Ml + Mq);
  // f << dot(payload_vy) == (cable_y * (T * cable_z * (q_w * q_w - q_x * q_x - q_y * q_y + q_z * q_z) - Mq * l_length * (dcable_x * dcable_x + dcable_y * dcable_y + dcable_z * dcable_z) - T * cable_y * (2 * (q_w * q_x - q_y * q_z)) + T * cable_x * (2 * (q_w * q_y + q_x * q_z)))) / (Ml + Mq);
  // f << dot(payload_vz) == (cable_z * (T * cable_z * (q_w * q_w - q_x * q_x - q_y * q_y + q_z * q_z) - Mq * l_length * (dcable_x * dcable_x + dcable_y * dcable_y + dcable_z * dcable_z) - T * cable_y * (2 * (q_w * q_x - q_y * q_z)) + T * cable_x * (2 * (q_w * q_y + q_x * q_z)))) / (Ml + Mq) - g_z;

  f << dot(cable_x) == dcable_x;
  f << dot(cable_y) == dcable_y;
  f << dot(cable_z) == dcable_z;

  f << dot(dcable_x) == -(cable_y * T * (cable_x * (2 * (q_w * q_x - q_y * q_z)) + cable_y * (2 * (q_w * q_y + q_x * q_z))) - cable_z * (T * cable_x * (q_w * q_w - q_x * q_x - q_y * q_y + q_z * q_z) - T * cable_z * (2 * (q_w * q_y + q_x * q_z))) + Mq * cable_x * l_length * (dcable_x * dcable_x + dcable_y * dcable_y + dcable_z * dcable_z)) / (Mq * l_length);
  f << dot(dcable_y) == (cable_x * T * (cable_x * (2 * (q_w * q_x - q_y * q_z)) + cable_y * (2 * (q_w * q_y + q_x * q_z))) + cable_z * (T * cable_y * (q_w * q_w - q_x * q_x - q_y * q_y + q_z * q_z) + T * cable_z * (2 * (q_w * q_x - q_y * q_z))) - Mq * cable_y * l_length * (dcable_x * dcable_x + dcable_y * dcable_y + dcable_z * dcable_z)) / (Mq * l_length);
  f << dot(dcable_z) == -(cable_x * (T * cable_x * (q_w * q_w - q_x * q_x - q_y * q_y + q_z * q_z) - T * cable_z * (2 * (q_w * q_y + q_x * q_z))) + cable_y * (T * cable_y * (q_w * q_w - q_x * q_x - q_y * q_y + q_z * q_z) + T * cable_z * (2 * (q_w * q_x - q_y * q_z))) + Mq * cable_z * l_length * (dcable_x * dcable_x + dcable_y * dcable_y + dcable_z * dcable_z)) / (Mq * l_length);

  // cos_cable_angle = -cable_z*g_z/(sqrt(cable_x*cable_x+ cable_y*cable_y+ cable_z*cable_z)*sqrt(g_z*g_z)); // cable.norm() = 1

  // Cost: Sum(i=0, ..., N-1){h_i' * Q * h_i} + h_N' * Q_N * h_N
  // Running cost vector consists of all states and inputs.
  h << p_x << p_y << p_z
    << q_w << q_x << q_y << q_z
    << v_x << v_y << v_z
    // << payload_x << payload_y << payload_z
    // << payload_vx << payload_vy << payload_vz
    << cable_x << cable_y << cable_z
    << dcable_x << dcable_y << dcable_z
    << T << w_x << w_y << w_z;

  // End cost vector consists of all states (no inputs at last state).
  hN << p_x << p_y << p_z
     << q_w << q_x << q_y << q_z
     << v_x << v_y << v_z
    //  << payload_x << payload_y << payload_z
    //  << payload_vx << payload_vy << payload_vz
     << cable_x << cable_y << cable_z
     << dcable_x << dcable_y << dcable_z;

  // for non codegen cases
  //  Running cost weight matrix
  DMatrix Q(h.getDim(), h.getDim());
  Q.setIdentity();
  Q(0, 0) = 100;   // x
  Q(1, 1) = 100;   // y
  Q(2, 2) = 100;   // z
  Q(3, 3) = 100;   // qw
  Q(4, 4) = 100;   // qx
  Q(5, 5) = 100;   // qy
  Q(6, 6) = 100;   // qz
  Q(7, 7) = 1;     // vx
  Q(8, 8) = 1;     // vy
  Q(9, 9) = 1;     // vz
  // Q(10, 10) = 100; // payload_x
  // Q(11, 11) = 100; // payload_y
  // Q(12, 12) = 100; // payload_z
  // Q(13, 13) = 1;   // payload_vx
  // Q(14, 14) = 1;   // payload_vy
  // Q(15, 15) = 1;   // payload_vz
  Q(10, 10) = 10;  // cable_x
  Q(11, 11) = 10;  // cable_y
  Q(12, 12) = 10;  // cable_z
  Q(13, 13) = 10;  // dcable_x
  Q(14, 14) = 10;  // dcable_y
  Q(15, 15) = 10;  // dcable_z

  // End cost weight matrix
  DMatrix QN(hN.getDim(), hN.getDim());
  QN.setIdentity();
  QN(0, 0) = Q(0, 0); // x
  QN(1, 1) = Q(1, 1); // y
  QN(2, 2) = Q(2, 2); // z
  QN(3, 3) = Q(3, 3); // qw
  QN(4, 4) = Q(4, 4); // qx
  QN(5, 5) = Q(5, 5); // qy
  QN(6, 6) = Q(6, 6); // qz
  // QN(7,7) = Q(7,7);   // vx
  // QN(8,8) = Q(8,8);   // vy
  // QN(9,9) = Q(9,9);   // vz
  QN(7, 7) = 100;         // vx
  QN(8, 8) = 100;         // vy
  QN(9, 9) = 100;         // vz
  // QN(10, 10) = Q(10, 10); // payload_x
  // QN(11, 11) = Q(11, 11); // payload_y
  // QN(12, 12) = Q(12, 12); // payload_z
  // QN(13,13) = Q(13,13);   // payload_vx
  // QN(14,14) = Q(14,14);   // payload_vy
  // QN(15,15) = Q(15,15);   // payload_vz
  // QN(16,16) = Q(16,16);   // cable_x
  // QN(17,17) = Q(17,17);   // cable_y
  // QN(18,18) = Q(18,18);   // cable_z
  // QN(13, 13) = 100;       // payload_vx
  // QN(14, 14) = 100;       // payload_vy
  // QN(15, 15) = 100;       // payload_vz
  QN(10, 10) = 10;  // cable_x
  QN(11, 11) = 10;  // cable_y
  QN(12, 12) = 10;  // cable_z
  QN(13, 13) = 10;  // dcable_x
  QN(14, 14) = 10;  // dcable_y
  QN(15, 15) = 10;  // dcable_z

  // Set a reference for the analysis (if CODE_GEN is false).
  // Reference is at x = 2.0m in hover (qw = 1).
  DVector r(h.getDim());    // Running cost reference
  const double x_ref = 1.0; // x position of reference
  const double y_ref = 0.0; // y position of reference
  const double z_ref = 0.6; // z position of reference
  r.setZero();
  r(0) = x_ref;
  r(1) = y_ref;
  r(2) = z_ref;
  // r(10) = x_ref;
  // r(11) = y_ref;
  // r(12) = z_ref - l_length;
  r(3) = 1.0;
  r(12) = -1.0;
  r(16) = g_z * (Mq + Ml);

  DVector rN(hN.getDim()); // End cost reference
  rN.setZero();
  rN(0) = r(0);
  rN(1) = r(1);
  rN(2) = r(2);
  // rN(10) = r(10);
  // rN(11) = r(11);
  // rN(12) = r(12);
  rN(3) = r(3);
  rN(12) = r(12);

  // DEFINE AN OPTIMAL CONTROL PROBLEM:
  // ----------------------------------
  OCP ocp(0.0, dt*N, N);
  if (!CODE_GEN)
  {
    // For analysis, set references.
    ocp.minimizeLSQ(Q, h, r);
    ocp.minimizeLSQEndTerm(QN, hN, rN);
  }
  else
  {
    // For code generation, references are set during run time.
    BMatrix Q_sparse(h.getDim(), h.getDim());
    Q_sparse.setIdentity();
    BMatrix QN_sparse(hN.getDim(), hN.getDim());
    QN_sparse.setIdentity();
    ocp.minimizeLSQ(Q_sparse, h);
    ocp.minimizeLSQEndTerm(QN_sparse, hN); // Least Square term
  }

  // Add system dynamics
  ocp.subjectTo(f);
  // Add constraints
  ocp.subjectTo(-w_max_xy <= w_x <= w_max_xy);
  ocp.subjectTo(-w_max_xy <= w_y <= w_max_xy);
  ocp.subjectTo(-w_max_yaw <= w_z <= w_max_yaw);
  ocp.subjectTo(T_min <= T <= T_max);
  // ocp.subjectTo( -1.01 <= cable_z <= -K_min_cos_cable_angle);

  ocp.setNOD(0); // num of OnlineData   //10?13?

  if (!CODE_GEN)
  {
    // Set initial state
    ocp.subjectTo(AT_START, p_x == 0.0);
    ocp.subjectTo(AT_START, p_y == 0.0);
    ocp.subjectTo(AT_START, p_z == l_length);
    ocp.subjectTo(AT_START, q_w == 1.0);
    ocp.subjectTo(AT_START, q_x == 0.0);
    ocp.subjectTo(AT_START, q_y == 0.0);
    ocp.subjectTo(AT_START, q_z == 0.0);
    ocp.subjectTo(AT_START, v_x == 0.0);
    ocp.subjectTo(AT_START, v_y == 0.0);
    ocp.subjectTo(AT_START, v_z == 0.0);
    ocp.subjectTo(AT_START, w_x == 0.0);
    ocp.subjectTo(AT_START, w_y == 0.0);
    ocp.subjectTo(AT_START, w_z == 0.0);

    // ocp.subjectTo(AT_START, payload_x == 0.0);
    // ocp.subjectTo(AT_START, payload_y == 0.0);
    // ocp.subjectTo(AT_START, payload_z == 0.0);
    // ocp.subjectTo(AT_START, payload_vx == 0);
    // ocp.subjectTo(AT_START, payload_vy == 0);
    // ocp.subjectTo(AT_START, payload_vz == 0);
    ocp.subjectTo(AT_START, cable_x == 0.0);
    ocp.subjectTo(AT_START, cable_y == 0.0);
    ocp.subjectTo(AT_START, cable_z == -1.0);
    ocp.subjectTo(AT_START, dcable_x == 0);
    ocp.subjectTo(AT_START, dcable_y == 0);
    ocp.subjectTo(AT_START, dcable_z == 0);

    // Setup some visualization
    GnuplotWindow window1(PLOT_AT_EACH_ITERATION);
    window1.addSubplot(p_x, "position x");
    window1.addSubplot(p_y, "position y");
    window1.addSubplot(p_z, "position z");
    // window1.addSubplot( v_x,"velocity x" );
    // window1.addSubplot( v_y,"velocity y" );
    // window1.addSubplot( v_z,"velocity z" );
    // window1.addSubplot(payload_x, "payload position x");
    // window1.addSubplot(payload_y, "payload position y");
    // window1.addSubplot(payload_z, "payload position z");
    // window1.addSubplot(cable_x*cable_x+cable_y*cable_y+cable_z*cable_z - 1.0,"cable length");
    // window1.addSubplot( ((payload_x-p_x)*(payload_x-p_x)+(payload_y-p_y)*(payload_y-p_y)+(payload_z-p_z)*(payload_z-p_z)-l_length*l_length) ,"distance" );

    GnuplotWindow window3(PLOT_AT_EACH_ITERATION);
    window3.addSubplot(T, "Thrust");
    window3.addSubplot(w_x, "rotation-acc x");
    window3.addSubplot(w_y, "rotation-acc y");
    window3.addSubplot(w_z, "rotation-acc z");

    // Define an algorithm to solve it.
    OptimizationAlgorithm algorithm(ocp);
    algorithm.set(INTEGRATOR_TOLERANCE, 1e-6);
    algorithm.set(KKT_TOLERANCE, 1e-3);
    algorithm << window1;
    // algorithm << window2;
    algorithm << window3;
    algorithm.solve();
  }
  else
  {
    // For code generation, we can set some properties.
    // The main reason for a setting is given as comment.
    OCPexport mpc(ocp);

    mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);    // is robust, stable
    mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING); // good convergence
    mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING_N2); // due to qpOASES
    // mpc.set(INTEGRATOR_TYPE,        INT_RK4);
    mpc.set(INTEGRATOR_TYPE, INT_IRK_GL4); // accurate  INT_IRK_GL4
    mpc.set(NUM_INTEGRATOR_STEPS, N);
    mpc.set(QP_SOLVER, QP_QPOASES); // free, source code
    mpc.set(HOTSTART_QP, YES);
    mpc.set( LEVENBERG_MARQUARDT, 1e-10);
    // mpc.set( LINEAR_ALGEBRA_SOLVER, GAUSS_LU);
    // mpc.set( IMPLICIT_INTEGRATOR_NUM_ITS, 2);
    mpc.set(CG_USE_OPENMP, YES);                    // paralellization
    mpc.set(CG_HARDCODE_CONSTRAINT_VALUES, NO);     // set on runtime
    mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES); // time-varying costs
    mpc.set(USE_SINGLE_PRECISION, YES);             // Single precision

    // Do not generate tests, makes or matlab-related interfaces.
    mpc.set(GENERATE_TEST_FILE, NO);
    mpc.set(GENERATE_MAKE_FILE, NO);
    mpc.set(GENERATE_MATLAB_INTERFACE, NO);
    mpc.set(GENERATE_SIMULINK_INTERFACE, NO);

    // Finally, export everything.
    if (mpc.exportCode("quadrotor_payload_mpc") != SUCCESSFUL_RETURN)
      exit(EXIT_FAILURE);
    mpc.printDimensionsQP();
  }

  return EXIT_SUCCESS;
}