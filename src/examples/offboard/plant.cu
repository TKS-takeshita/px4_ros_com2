#include "px4_ros_com/plant.cuh"
#include <stdio.h>


// 初期化
__host__ __device__ void Plant::initialize(float state[], float param[], float dt){
    memcpy(this->state, state, DIM_STT*sizeof(float));
    memcpy(this->param, param, DIM_PARAM*sizeof(float));
    this->DT = dt;
}

__host__ __device__ void Plant::initialize(float dt){
    this->DT = dt;
}

// 状態量の表示
__host__ __device__ void Plant::show_stt(){
    printf("\n");
    for(int i=0; i<DIM_PARAM; i++){
        printf("%f", param[i]);
    }
    printf("\n");
}

//　パラメータの変更
__host__ __device__ void Plant::change_param(float param[]){
    memcpy(this->param, param, DIM_PARAM*sizeof(float));
}

// 1stepシミュレーション
__host__ __device__ void Plant::simulation(float obs[], float input[], enum solver slv){
    float next[DIMENTION_OF_STATE];

    system_eq(next, input, slv);
    observation_eq(obs, next);
    memcpy(this->state, next, DIM_STT*sizeof(float));
}

__host__ __device__ void Plant::simulation(float input[], enum solver slv){
    float next[DIMENTION_OF_STATE];
    system_eq(next, input, slv);
    memcpy(this->state, next, DIM_STT*sizeof(float));
}

//システム方程式
__host__ __device__ void Plant::system_eq(float next[], float in[], enum solver slv){
    switch(slv){
        case eul:
            {
                float diff[DIMENTION_OF_STATE];
                calc_diff_stt(diff, state, in);
                euler_method(next, state, diff, DT);
            }
            break;
        case rk4:
            runge_kutta_4_method(next, state, in, DT);
            break;
        default:
            printf("err Plant::system_eq\n");
    }
}

// 観測方程式
__host__ __device__ void Plant::observation_eq(float obs[], float stt[]){
    memcpy(obs, stt, DIM_STT*sizeof(float));
}

// 4次ルンゲクッタ法
__host__ __device__ void Plant::runge_kutta_4_method(float next[], float stt[], float in[], float DT){
    float diff[DIMENTION_OF_STATE];

    float kk1[DIMENTION_OF_STATE];
    float kk2[DIMENTION_OF_STATE];
    float kk3[DIMENTION_OF_STATE];
    float kk4[DIMENTION_OF_STATE];
    float tmp[DIMENTION_OF_STATE];
    
    // calc kk1
    calc_diff_stt(kk1, stt, in);
    // calc kk2
    euler_method(tmp, stt, kk1, DT*0.5f);
    calc_diff_stt(kk2, tmp, in);
    // calc kk3
    euler_method(tmp, stt, kk2, DT*0.5f);
    calc_diff_stt(kk3, tmp, in);
    // calc kk4
    euler_method(tmp, stt, kk3, DT);
    calc_diff_stt(kk4, tmp, in);
    // calc diff
    for(int j=0; j<DIMENTION_OF_STATE; j++){
        diff[j] = (kk1[j] + 2.0f*kk2[j] + 2.0f*kk3[j] + kk4[j]) / 6.0f;
    }
    euler_method(next, stt, diff, DT);
}

// オイラー法数値積分
__host__ __device__ void Plant::euler_method(float next[], float stt[], float diff[], float dt){
    for(int j=0; j<this>DIM_STT; ++j){
        next[j] = stt[j] + diff[j]*dt;
    }
    // クォータニオン正規化
    const float q_norm = sqrtf(next[0]*next[0] + next[1]*next[1] + next[2]*next[2] + next[3]*next[3]);
    next[0] /= q_norm;//q0
    next[1] /= q_norm;//q1
    next[2] /= q_norm;//q2
    next[3] /= q_norm;//q3
}

// 状態量の微分値
__host__ __device__ void Plant::calc_diff_stt(float stt_diff[]){
    // 状態変数
    const float q0 = stt[0];//クォータニオン 慣性座標系
    const float q1 = stt[1];
    const float q2 = stt[2];
    const float q3 = stt[3];
    const float wx = stt[4];//角速度　機体座標系
    const float wy = stt[5];
    const float wz = stt[6];
    const float px = stt[7];//位置　慣性座標系
    const float py = stt[8];
    const float pz = stt[9];
    const float vx = stt[10];//速度　慣性座標系
    const float vy = stt[11];
    const float vz = stt[12];

    // パラメータ
    const float MM = param[5];
    const float LL = 0.5f;

    const float Ixx = param[1];
    const float Iyy = param[2];
    const float Izz = param[3];

    const float ug = param[0];//デカップリング入力バイアス
    const float GG = 9.80665f;//重力加速度[m/s2]
    const float u_diff_lim = 55.0f;
    const float umax = 230.0f;
    const float Tmax = param[4];
    const float taumax = param[6];

    //デカップリング処理
    const float u_raw_cw1 = ug + in[0] - in[2] + in[3];
    const float u_raw_cw2 = ug + in[0] + in[2] + in[3];
    const float u_raw_ccw1 = ug + in[0] + in[1] - in[3];
    const float u_raw_ccw2 = ug + in[0] - in[1] - in[3];

    // 入力飽和
    const float u_cw1 = (u_raw_cw1 >umax)*umax + (u_raw_cw1 <(ug-u_diff_lim))*(ug-u_diff_lim) + u_raw_cw1;
    const float u_cw2 = (u_raw_cw2 >umax)*umax + (u_raw_cw2 <(ug-u_diff_lim))*(ug-u_diff_lim) + u_raw_cw2;
    const float u_ccw1 = (u_raw_ccw1 >umax)*umax + (u_raw_ccw1 <(ug-u_diff_lim))*(ug-u_diff_lim) + u_raw_ccw1;
    const float u_ccw2 = (u_raw_ccw2 >umax)*umax + (u_raw_ccw2 <(ug-u_diff_lim))*(ug-u_diff_lim) + u_raw_ccw2;

    //計算用中間変数
    const float inv_umax_2 = 1.0 / (umax*umax);
}

// オイラー法数値積分
__host__ __device__ void Plant::euler_method(float next[], float stt[], float diff[], float dt){
	for(int j=0; j<this->DIM_STT; j++)
		next[j] = stt[j] + diff[j] * dt;

	// クォータニオン正規化
	const float q_norm = sqrtf(next[0]*next[0] + next[1]*next[1] + next[2]*next[2] + next[3]*next[3]);
	/* q0 */ next[0] = next[0] / q_norm;
	/* q1 */ next[1] = next[1] / q_norm;
	/* q2 */ next[2] = next[2] / q_norm;
	/* q3 */ next[3] = next[3] / q_norm;
}

// 状態量の微分値
__host__ __device__ void Plant::calc_diff_stt(float stt_diff[], float stt[], float in[]){
    // 状態変数
	const float q0 = stt[0];  // クォータニオン 世界座標系
	const float q1 = stt[1];  // クォータニオン 世界座標系
	const float q2 = stt[2];  // クォータニオン 世界座標系
	const float q3 = stt[3];  // クォータニオン 世界座標系 
	const float wx = stt[4];  // 角速度         機体座標系 FLU
	const float wy = stt[5];  // 角速度         機体座標系 FLU
	const float wz = stt[6];  // 角速度         機体座標系 FLU
	const float px = stt[7];  // 位置           世界座標系
	const float py = stt[8];  // 位置           世界座標系
    const float pz = stt[9];  // 位置           世界座標系
    const float vx = stt[10]; // 速度           世界座標系
	const float vy = stt[11]; // 速度           世界座標系
	const float vz = stt[12]; // 速度           世界座標系
    // パラメータ
    const float MM = param[6];// 機体質量[kg]
	const float LL = param[7];// アーム長[m]0.25548

    const float Ix = param[1];
	const float Iy = param[2];
	const float Iz = param[3];
    const float ug = param[0];   // デカップリング入力バイアス [rps]
	const float GG = 9.80665f; // 重力加速度[m/s^2]
	const float umin = 439.82293f;//最小回転速度[rad/s]
    const float umax = 1120.81554f;//最大回転速度[rad/s]
    const float Tmax = param[4];// 最大推力[N]
    const float taumax = param[5]; // 最大トルク[Nm]
    const float inv_sqrt2 = 0.70710678f;
    const float thrust_coef = 1.09e-5f; // 推力係数[N・s^2]
    const float torque_coef = 1.52e-7f; // トルク係数[Nm・s^2]

    //in[0] : vz, in[1] : vwx, in[2] : vwy, in[3] : vwz (機体座標系)

    const float u_raw_cw1  = ug + in[0] + (in[1] - in[2])*inv_sqrt2 + in[3];
	const float u_raw_cw2  = ug + in[0] - (in[1] - in[2])*inv_sqrt2 + in[3];
	const float u_raw_ccw1 = ug + in[0] - (in[1] + in[2])*inv_sqrt2 - in[3];
	const float u_raw_ccw2 = ug + in[0] + (in[1] + in[2])*inv_sqrt2 - in[3];

    const float u_cw1  = fminf(umax, fmaxf(umin, u_raw_cw1));
    const float u_cw2  = fminf(umax, fmaxf(umin, u_raw_cw2));
    const float u_ccw1 = fminf(umax, fmaxf(umin, u_raw_ccw1));
    const float u_ccw2 = fminf(umax, fmaxf(umin, u_raw_ccw2));

    const float inv_umax_2 = 1.0 / (umax*umax);
    const float FF = thrust_coef * ((u_cw1*u_cw1) + (u_cw2*u_cw2) + (u_ccw1*u_ccw1) + (u_ccw2*u_ccw2));
    // 状態微分計算
	/* q0 */ stt_diff[0]  =  0.5f*(-q1*wx - q2*wy - q3*wz);
	/* q1 */ stt_diff[1]  =  0.5f*( q0*wx + q2*wz - q3*wy);
	/* q2 */ stt_diff[2]  =  0.5f*( q0*wy + q3*wx - q1*wz);
	/* q3 */ stt_diff[3]  =  0.5f*( q0*wz + q1*wy - q2*wx);

    /* wx */ stt_diff[4]  = ((Iy-Iz)*wy*wz + inv_sqrt2*LL*thrust_coef*( -(u_ccw1*u_ccw1) + (u_cw1*u_cw1) + (u_ccw2*u_ccw2) - (u_cw1*u_cw1))) / Ix; // zhu
	/* wy */ stt_diff[5]  = ((Iz-Ix)*wx*wz + inv_sqrt2*LL*thrust_coef*((u_ccw1*u_ccw1) + (u_cw2*u_cw2) - (u_ccw2*u_ccw2) - (u_cw1*u_cw1))) / Iy; // zhu
	/* wz */ stt_diff[6]  = ((Ix-Iy)*wx*wy + torque_coef * (-(u_ccw1*u_ccw1) + (u_cw1*u_cw1) - (u_ccw2*u_ccw2) + (u_cw2*u_cw2))) / Iz;

    /* px */ stt_diff[7]  = vx;
	/* py */ stt_diff[8]  = vy;
	/* pz */ stt_diff[9]  = vz;
	/* vx */ stt_diff[10] = 2.0f*(q0*q2+q1*q3) * FF/MM;
    /* vy */ stt_diff[11] = 2.0f*(q2*q3-q0*q1) * FF/MM;
    /* vz */ stt_diff[12] = FF*(-1.0f+2.0f*q0*q0+2.0f*q3*q3)/MM - GG;
}