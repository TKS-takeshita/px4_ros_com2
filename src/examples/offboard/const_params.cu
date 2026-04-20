#include "px4_ros_com/const_params.hpp"

// double for host programs

const double CONST_PARAM::U_G        = 113.0;//rps for hover
const double CONST_PARAM::U_DIFF_LIM = 60.0;
const double CONST_PARAM::MAX_RPS    = CONST_PARAM::U_G + CONST_PARAM::U_DIFF_LIM; // max rps of the propeller

const double CONST_PARAM::A_OF_GRAVITY    = 9.80665;    // m/s^2
const double CONST_PARAM::I_XX            = 0.04418; //kg・m^2　from CAD
const double CONST_PARAM::I_YY            = 0.04226;
const double CONST_PARAM::I_ZZ            = 0.05619;
const double CONST_PARAM::ROTOR_DISTANCE  = 0.50; //m 向かい合うロータ間の距離
const double CONST_PARAM::MASS_OF_MACHINE = 2.35; // kg
const double CONST_PARAM::MAX_RPS_POW     = CONST_PARAM::MAX_RPS * CONST_PARAM::MAX_RPS;

const double CONST_PARAM::MAX_THRUST      = 53.6;      // N
const double CONST_PARAM::TORQUE_RATE     = 0.19 / CONST_PARAM::MAX_RPS / CONST_PARAM::MAX_RPS;      // N・m

const double CONST_PARAM::INIT_U_THRUST    = 0.0; // N
const double CONST_PARAM::INIT_E0         = 1.0;    // no units
const double CONST_PARAM::INIT_E1         = 0.0;    // no units
const double CONST_PARAM::INIT_E2         = 0.0;    // no units
const double CONST_PARAM::INIT_E3         = 0.0;    // no units
const double CONST_PARAM::INIT_WX         = 0.0;    // rad/sec
const double CONST_PARAM::INIT_WY         = 0.0;    // rad/sec
const double CONST_PARAM::INIT_WZ         = 0.0;    // rad/sec
const double CONST_PARAM::INIT_X          = 0.0;    // m
const double CONST_PARAM::INIT_Y          = 0.0;    // m
const double CONST_PARAM::INIT_Z          = 0.0;    // m
const double CONST_PARAM::INIT_XP         = 0.0;    // m/s
const double CONST_PARAM::INIT_YP         = 0.0;    // m/s
const double CONST_PARAM::INIT_ZP         = 0.0;    // m/s

const double CONST_PARAM::INIT_TARGET_E0  = 1.0;
const double CONST_PARAM::INIT_TARGET_E1  = 0.0;
const double CONST_PARAM::INIT_TARGET_E2  = 0.0;
const double CONST_PARAM::INIT_TARGET_E3  = 0.0;
const double CONST_PARAM::INIT_TARGET_WX  = 0.0;
const double CONST_PARAM::INIT_TARGET_WY  = 0.0;
const double CONST_PARAM::INIT_TARGET_WZ  = 0.0;
const double CONST_PARAM::INIT_TARGET_X   = 0.0;
const double CONST_PARAM::INIT_TARGET_Y   = 0.0;
const double CONST_PARAM::INIT_TARGET_Z   = -1.2;
const double CONST_PARAM::INIT_TARGET_XP  = 0.0;
const double CONST_PARAM::INIT_TARGET_YP  = 0.0;
const double CONST_PARAM::INIT_TARGET_ZP  = 0.0;

const double CONST_PARAM::FINAL_TIME       = 15.0;   // s
const double CONST_PARAM::TIME_STEP        = 0.001; // s
const int    CONST_PARAM::INT_PRINT_SCREEN = 1;
const int    CONST_PARAM::INT_PRINT_FILE   = 1;
const double CONST_PARAM::ABS_ERROR        = 1.0E-7;
const double CONST_PARAM::REL_ERROR        = 1.0E-8;


const double CONST_PARAM::CONTROL_PERIOD      = 0.02;   // s
const int    CONST_PARAM::N_OF_SAMPLES        = _DEVICE_CONST_THREAD_PER_BLOCK * _DEVICE_CONST_N_OF_BLOCK;
const int    CONST_PARAM::N_OF_THE_USING_BEST = 100;
const int    CONST_PARAM::ITERATION_TIMES     = 2;
const double CONST_PARAM::U_UPPER_LIM         = CONST_PARAM::U_G + CONST_PARAM::U_DIFF_LIM; // no units
const double CONST_PARAM::U_LOWER_LIM         = CONST_PARAM::U_G - CONST_PARAM::U_DIFF_LIM; // no units

#ifdef UNPREDICTABLE_IMPULSE
    const double CONST_PARAM::IMPULSE_TIME = 3.0;               // s
    const double CONST_PARAM::IMPULSE_XP   = 1.0;               // m/s
    const double CONST_PARAM::IMPLUSE_WY   = 2 * 3.14159265359; // ras/s
#endif

#if defined(UNPREDICTABLE_COLLISION_WITH_WALL) || defined(PREDICTABLE_COLLISION_WITH_WALL)
    const double CONST_PARAM::X_WALL               = -1.0;  // m
    const double CONST_PARAM::WALL_NORMAL_VECTOR_X = 1.0;   // no units
    const double CONST_PARAM::WALL_NORMAL_VECTOR_Y = 0.0;   // no units
    const double CONST_PARAM::WALL_NORMAL_VECTOR_Z = 0.0;   // no units
    const double CONST_PARAM::R_OF_RING            = 0.135; // m
    const double CONST_PARAM::COEFF_OF_REST        = 0.5;   // coefficient of restitution
#endif

const std::string CONST_PARAM::FILE_HEADER[11] = {"% FILE: quadcopter_mcmpc.1\n%\n"
                                                  "%       t           RpsCW1         RpsCW2         RpsCCW1        RpsCCW2\n"
                                                  "%     (sec)        (NoUnits)      (NoUnits)      (NoUnits)      (NoUnits)\n\n",
                                                  "% FILE: quadcopter_mcmpc.2\n%\n"
                                                  "%       t           theta_x        theta_y        theta_z\n"
                                                  "%     (sec)        (degrees)      (degrees)      (degrees)\n\n",
                                                  "% FILE: quadcopter_mcmpc.3\n%\n"
                                                  "%       t              x              y              z\n"
                                                  "%     (sec)           (m)            (m)            (m)\n\n",
                                                  "% FILE: quadcopter_mcmpc.4\n%\n"
                                                  "%       t        P_EARTHo_DRONEcm[1] P_EARTHo_DRONEcm[2] P_EARTHo_DRONEcm[3] EARTH_DRONE[1,1] EARTH_DRONE[1,2] EARTH_DRONE[1,3] EARTH_DRONE[2,1] EARTH_DRONE[2,2] EARTH_DRONE[2,3] EARTH_DRONE[3,1] EARTH_DRONE[3,2] EARTH_DRONE[3,3]\n"
                                                  "%   (second)           (meter)             (meter)             (meter)          (NoUnits)        (NoUnits)        (NoUnits)        (NoUnits)        (NoUnits)        (NoUnits)        (NoUnits)        (NoUnits)        (NoUnits)\n\n",
                                                  "% FILE: quadcopter_mcmpc.5\n%\n"
                                                  "%       t        P_EARTHo_RoterCCW1[1] P_EARTHo_RoterCCW1[2] P_EARTHo_RoterCCW1[3]\n"
                                                  "%   (second)            (meter)               (meter)               (meter)\n\n",
                                                  "% FILE: quadcopter_mcmpc.6\n%\n"
                                                  "%       t        P_EARTHo_RoterCCW2[1] P_EARTHo_RoterCCW2[2] P_EARTHo_RoterCCW2[3]\n"
                                                  "%   (second)            (meter)               (meter)               (meter)\n\n",
                                                  "% FILE: quadcopter_mcmpc.7\n%\n"
                                                  "%       t       P_EARTHo_RoterCW1[1] P_EARTHo_RoterCW1[2] P_EARTHo_RoterCW1[3]\n"
                                                  "%   (second)           (meter)              (meter)              (meter)\n\n",
                                                  "% FILE: quadcopter_mcmpc.8\n%\n"
                                                  "%       t       P_EARTHo_RoterCW2[1] P_EARTHo_RoterCW2[2] P_EARTHo_RoterCW2[3]\n"
                                                  "%   (second)           (meter)              (meter)              (meter)\n\n",
                                                  "% FILE: quadcopter_mcmpc.9\n%\n"
                                                  "%       t               Cost\n"
                                                  "%   (second)            ( - )\n\n",
                                                  "% FILE: quadcopter_mcmpc.10\n%\n"
                                                  "%       t             x_p            y_p            z_p\n"
                                                  "%     (sec)          (m/s)          (m/s)          (m/s)\n\n",
                                                  "% FILE: quadcopter_mcmpc.11\n%\n"
                                                  "%       t             w_x            w_y            w_z\n"
                                                  "%     (sec)         (rad/s)        (rad/s)        (rad/s)\n\n"
                                                };

//float for host programs
const float CONST_PARAM_FLOAT::U_G             = (float)CONST_PARAM::U_G;
const float CONST_PARAM_FLOAT::U_UPPER_LIM     = (float)CONST_PARAM::U_UPPER_LIM;
const float CONST_PARAM_FLOAT::U_LOWER_LIM     = (float)CONST_PARAM::U_LOWER_LIM;

const float CONST_PARAM_FLOAT::I_XX            = (float)CONST_PARAM::I_XX;
const float CONST_PARAM_FLOAT::I_YY            = (float)CONST_PARAM::I_YY;
const float CONST_PARAM_FLOAT::I_ZZ            = (float)CONST_PARAM::I_ZZ;
const float CONST_PARAM_FLOAT::ROTOR_DISTANCE  = (float)CONST_PARAM::ROTOR_DISTANCE;
const float CONST_PARAM_FLOAT::MAX_RPS_POW     = (float)CONST_PARAM::MAX_RPS_POW;
const float CONST_PARAM_FLOAT::MAX_THRUST      = (float)CONST_PARAM::MAX_THRUST;
const float CONST_PARAM_FLOAT::MASS_OF_MACHINE = (float)CONST_PARAM::MASS_OF_MACHINE;
const float CONST_PARAM_FLOAT::TORQUE_RATE     = (float)CONST_PARAM::TORQUE_RATE;
const float CONST_PARAM_FLOAT::A_OF_GRAVITY    = (float)CONST_PARAM::A_OF_GRAVITY;

const float CONST_PARAM_FLOAT::INIT_TARGET_E0  = (float)CONST_PARAM::INIT_TARGET_E0;
const float CONST_PARAM_FLOAT::INIT_TARGET_E1  = (float)CONST_PARAM::INIT_TARGET_E1;
const float CONST_PARAM_FLOAT::INIT_TARGET_E2  = (float)CONST_PARAM::INIT_TARGET_E2;
const float CONST_PARAM_FLOAT::INIT_TARGET_E3  = (float)CONST_PARAM::INIT_TARGET_E3;
const float CONST_PARAM_FLOAT::INIT_TARGET_WX  = (float)CONST_PARAM::INIT_TARGET_WX;
const float CONST_PARAM_FLOAT::INIT_TARGET_WY  = (float)CONST_PARAM::INIT_TARGET_WY;
const float CONST_PARAM_FLOAT::INIT_TARGET_WZ  = (float)CONST_PARAM::INIT_TARGET_WZ;
const float CONST_PARAM_FLOAT::INIT_TARGET_X   = (float)CONST_PARAM::INIT_TARGET_X;
const float CONST_PARAM_FLOAT::INIT_TARGET_Y   = (float)CONST_PARAM::INIT_TARGET_Y;
const float CONST_PARAM_FLOAT::INIT_TARGET_Z   = (float)CONST_PARAM::INIT_TARGET_Z;
const float CONST_PARAM_FLOAT::INIT_TARGET_XP  = (float)CONST_PARAM::INIT_TARGET_XP;
const float CONST_PARAM_FLOAT::INIT_TARGET_YP  = (float)CONST_PARAM::INIT_TARGET_YP;
const float CONST_PARAM_FLOAT::INIT_TARGET_ZP  = (float)CONST_PARAM::INIT_TARGET_ZP;

#ifdef PREDICTABLE_COLLISION_WITH_WALL
const float CONST_PARAM_FLOAT::X_WALL               = (float)CONST_PARAM::X_WALL;
const float CONST_PARAM_FLOAT::WALL_NORMAL_VECTOR_X = (float)CONST_PARAM::WALL_NORMAL_VECTOR_X;
const float CONST_PARAM_FLOAT::WALL_NORMAL_VECTOR_Y = (float)CONST_PARAM::WALL_NORMAL_VECTOR_Y;
const float CONST_PARAM_FLOAT::WALL_NORMAL_VECTOR_Z = (float)CONST_PARAM::WALL_NORMAL_VECTOR_Z;
const float CONST_PARAM_FLOAT::R_OF_RING            = (float)CONST_PARAM::R_OF_RING;
const float CONST_PARAM_FLOAT::COEFF_OF_REST        = (float)CONST_PARAM::COEFF_OF_REST;
#endif

const float CONST_PARAM_FLOAT::CONTROL_PERIOD        = (float)CONST_PARAM::CONTROL_PERIOD;
const float CONST_PARAM_FLOAT::INTEGRATION_STEP_SIZE = (float)CONST_PARAM::CONTROL_PERIOD / 2.0f;   // Set it to the 1/N value of CONTROL_PERIOD 

const float CONST_PARAM_FLOAT::SIGMA_CONST[4] = {2.0f, 0.1f, 0.1f, 0.1f};
