#pragma once

#include <string>

//#define UNPREDICTABLE_IMPULSE
//#define UNPREDICTABLE_COLLISION_WITH_WALL
//#define PREDICTABLE_COLLISION_WITH_WALL

#define _N_OF_ODES 		13 // number of states [quaternion(4), angular velocity(3),position(3), velocity(3)]
#define _PI_FROAT		3.141592653589793f
#define _RAD_TO_DEG		180.0f / _PI_FROAT
#define _INV_SQRT_2		0.7071067811865475f // 1/sqrt(2)

// const for device (GPU)
#define _DEVICE_CONST_HORIZON 			50 // horizon
#define _DEVICE_CONST_THREAD_PER_BLOCK 	128 //_DEVICE_CONST_THREAD_PER_BLOCK * N_OF_BLOCK = N_OF_SAMPLES
#define _DEVICE_CONST_N_OF_BLOCK 		64

// cost for MPC
#define _COST_Q_X 		1.0f
#define _COST_Q_Y 		1.0f
#define _COST_Q_Z 		2.0f
#define _COST_Q_XP 		3.0f
#define _COST_Q_YP 		3.0f
#define _COST_Q_ZP 		5.0f
#define _COST_Q_E1 		0.001f
#define _COST_Q_E2 		0.001f
#define _COST_Q_E3 		0.001f
#define _COST_Q_WX 		36.0f
#define _COST_Q_WY 		36.0f
#define _COST_Q_WZ 		9.0f
#define _COST_Q_ZI 		0.0f
#define _COST_R_VZ 		0.0001f
#define _COST_R_VWX 	0.001f
#define _COST_R_VWY 	0.001f
#define _COST_R_VWZ 	0.001f

// const for host(CPU)

struct CONST_PARAM
{
	static const double U_G; 		//rps for hover
	static const double U_DIFF_LIM;
	static const double MAX_RPS;	// max rps of the propeller

	static const double A_OF_GRAVITY; // gravity acceleration
	static const double I_XX;
	static const double I_YY;
	static const double I_ZZ;
	static const double ROTOR_DISTANCE;
	static const double MAX_RPS_POW; // max rps^2 of the propeller
	static const double MAX_THRUST;
	static const double MASS_OF_MACHINE;
	static const double TORQUE_RATE;

	static const double INIT_U_THRUST;

	static const double INIT_E0;
	static const double INIT_E1;
	static const double INIT_E2;
	static const double INIT_E3;
	static const double INIT_WX;
	static const double INIT_WY;
	static const double INIT_WZ;
	static const double INIT_X;
	static const double INIT_Y;
	static const double INIT_Z;
	static const double INIT_XP;
	static const double INIT_YP;
	static const double INIT_ZP;

	static const double INIT_TARGET_E0;
	static const double INIT_TARGET_E1;
	static const double INIT_TARGET_E2;
	static const double INIT_TARGET_E3;
	static const double INIT_TARGET_WX;
	static const double INIT_TARGET_WY;
	static const double INIT_TARGET_WZ;
	static const double INIT_TARGET_X;
	static const double INIT_TARGET_Y;
	static const double INIT_TARGET_Z;
	static const double INIT_TARGET_XP;
	static const double INIT_TARGET_YP;
	static const double INIT_TARGET_ZP;

	static const double FINAL_TIME;
	static const double TIME_STEP;
	static const int 	INT_PRINT_SCREEN;
	static const int 	INT_PRINT_FILE;
	static const double ABS_ERROR; // absolute error
	static const double REL_ERROR; // relative error

	static const double CONTROL_PERIOD;
	static const int 	N_OF_SAMPLES;
	static const int 	N_OF_THE_USING_BEST;
	static const int 	ITERATION_TIMES; // iteration times for MPC optimization

	static const double U_UPPER_LIM; // upper limit of input
	static const double U_LOWER_LIM; // lower limit of input

	static const std::string FILE_HEADER[11];

#ifdef UNPREDICTABLE_IMPULSE
	static const double IMPULSE_TIME;
	static const double IMPULSE_XP;
	static const double IMPLUSE_WY;
#endif

#if defined(UNPREDICTABLE_COLLISION_WITH_WALL) || defined(PREDICTABLE_COLLISION_WITH_WALL)
	static const double X_WALL;
	static const double WALL_NORMAL_VECTOR_X;
	static const double WALL_NORMAL_VECTOR_Y;
	static const double WALL_NORMAL_VECTOR_Z;
	static const double R_OF_RING;
	static const double COEFF_OF_REST;
#endif
};

struct CONST_PARAM_FLOAT
{
	static const float U_G;
    static const float U_UPPER_LIM;
    static const float U_LOWER_LIM;

    static const float A_OF_GRAVITY;
	static const float I_XX;
    static const float I_YY;
    static const float I_ZZ;
    static const float ROTOR_DISTANCE;
    static const float MAX_RPS_POW;
    static const float MAX_THRUST;
    static const float MASS_OF_MACHINE;
    static const float TORQUE_RATE;

	static const float INIT_TARGET_E0;
	static const float INIT_TARGET_E1;
	static const float INIT_TARGET_E2;
	static const float INIT_TARGET_E3;
	static const float INIT_TARGET_WX;
	static const float INIT_TARGET_WY;
	static const float INIT_TARGET_WZ;
	static const float INIT_TARGET_X;
	static const float INIT_TARGET_Y;
	static const float INIT_TARGET_Z;
	static const float INIT_TARGET_XP;
	static const float INIT_TARGET_YP;
	static const float INIT_TARGET_ZP;

	static const float CONTROL_PERIOD;
    static const float INTEGRATION_STEP_SIZE;

    static const float SIGMA_CONST[4];
    
#ifdef PREDICTABLE_COLLISION_WITH_WALL
    static const float X_WALL;
    static const float WALL_NORMAL_VECTOR_X;
    static const float WALL_NORMAL_VECTOR_Y;
    static const float WALL_NORMAL_VECTOR_Z;
    static const float R_OF_RING;
    static const float COEFF_OF_REST;
#endif

};
