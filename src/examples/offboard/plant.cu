#include "plant.cuh"
#include <stdio.h>


// 初期化
__host__ __device__ void Plant::initialize(float state[], float param[], float dt){
    memcpy(this->state, state, DIM_STT*sizeof(float));
    memcpy(this->param, param, DIM_PARAM*sizeof(float));
    this->DT = dt;
}

__host__device__ void Plant::initialize(float dt){
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
__host__ __device__ void Plant::system_eq(flaot next[], float in[], enum solver slv){
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
        case col_eul:
            {
                static float state_pre[DIMENTION_OF_STATE] = {};
                // パラメータ
                const float r_of_ring = 0.375;
            }
        case col_rk4:
            {

            }
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
        
        diff[j] = (kk[j] + 2.0f*kk2[j] + 2.0f*kk3[j] + kk4[j]) 
    }

}
