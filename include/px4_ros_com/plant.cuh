#ifndef PLANT_CUH
#define PLANT_CUH

#include "plant.cuh"
#include "px4_ros_com/common.cuh"

class Plant{
    public:
        float state[DIMENTION_OF_STATE];//状態量
        float param[DIMENTION_OF_PARAM];//パラメータ

        __host__ __device__ void initialize(float state[], float param[], float dt);
        __host__ __device__ void initialize(float dt);
        __host__ __device__ void show_stt();
        __host__ __device__ void change_param(float param[]);
        __host__ __device__ void simulation(float obs[], float input[], enum solver slv);
        __host__ __device__ void simulation(float input[], enum solver slv);
        __host__ __device__ void euler_method(float next[], float stt[], float diff[], float dt);
        __host__ __device__ void calc_diff_stt(float stt_diff[], float stt[], float in[]);

    private:
        const int DIM_STT = DIMENTION_OF_STATE;
        const int DIM_PARAM = DIMENTION_OF_PARAM;
        float DT;
        int col_flag = 0;

        __host__ __device__ void system_eq(float stt_diff[], float input[], enum solver slv);
        __host__ __device__ void observation_eq(float obs[], float stt[]);
        __host__ __device__ void runge_kutta_4_method(float next[], float stt[], float in[], float dt);
        __host__ __device__ void runge_kutta_4_method(float next[], float diff[], float stt[], float in[], float dt);
        __host__ __device__ void calc_disc_dyn();
};

#endif