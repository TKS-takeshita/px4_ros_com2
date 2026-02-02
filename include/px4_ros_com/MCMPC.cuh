#ifndef MCMPC_H
#define MCMPC_H

#include <thrust/device_vector.h>
#include "cuda_check_error.cuh"
#include "common.cuh"
#include "plant.cuh"

class MCMPC{
    public:
        MCMPC(float dt, float in_sdev[], float in_min[], float in_max[], float param[],
              float stt_min[], flaot stt_max[], float cstR[], float cstQ[], float cstQf[],
              float CST_BARRIER, float LAMBDA, int ITERATION, enum solver SOLVER, float stt_ref[]);
        ~MCMPC();

        void get_opt_in(float return_in[], float stt_ini[], float stt_ref[]=NULL);
        void update_param(float param[]);
    private:
        // 定数
        // CUDA処理変数
        unsigned int THREAD_P_BLOCK = PF_SI_THREAD_PER_BLOCK; // CUDAのスレッド数
        unsigned int BLOCK_NUM; //予測シミュレーションのブロック数
        unsigned int BLOCK_NUM_RND; //乱数生成用のブロック数

        const int SAMP_NUM = SAMPLE_NUM_OF_MCMPC;//サンプル数
        const int HRIZON_NUM = HORIZON_NUM_OF_MCMPC;//予測ホライズン
        const int SAMP_NUM_ELITE = SAMPLE_NUM_OF_MCMPC_ELITE;//エリートサンプル数

        // 各種ベクトルの次元数
        const int DIM_IN = DIMENTION_OF_INPUT;//入力の次元数
        const int DIM_STT = DIMENTION_OF_STATE;//状態の次元数
        const int DIM_PRM = DIMENTION_OF_PARAM;//パラメータの次元数
        float dt;
        enum solver SOLVER;

        // コスト・制約パラメータ
        float* in_min;//入力最小値
        float* in_max;//入力最大値
        float* stt_min;//状態量最小値
        float* stt_max;//状態量最大値
        float* cstR;//入力コスト
        float* cstQ;//状態量コスト
        float* cstQf;//状態量終端コスト
        float CST_BARRIER;//制約バリアコスト
        //最適化パラメータ
        float* in_sdev;//入力列生成標準偏差
        float LAMBDA;//MC積分定数
        int ITERATION;//最適化反復回数

        //変数
        //乱数
        curandState* d_rnd;//デバイス乱数
        //計算用変数
        float* in_seq;//入力列
        float* in_seq_opt;//最適入力列
        float* stt_ini;//予測シミュレーション初期状態
        float* stt_ref;//目標状態
        float* param;//予測シミュレーションパラメータ
        float* costs;//コスト
        float* costs_exp;//指数コスト
        thrust::device_vector<int> idx_seq;//連番配列
       
};


// CUDA関数
__global__ void MCMPC_predictive_simulate(curandState* d_rnd,
            float in_seq[], float in_seq_opt[], float sdev[], float in_min[], float in_max[], 
            float stt_ini[], float dt, float param[],
            float stt_ref[], float stt_min[], float stt_max[],
            float cstR[], float cstQ[], float cstQf[], float CST_BARRIER,
            float costs[], float costs_exp[], float cstQf[], enum solver SOLVER);

__device__ float MCMPC_cost_func(float in[], float stt[], float stt_ref[],
            float stt_min[], float stt_max[], float cstR[], float cstQ[], float CST_BARRIER);

__device__ void MCMPC_gen_input_seq(float in_seq[],
            curandState* d_rnd, int idx, float in_seq_opt[], float sdev[], float in_min[], float in_max[]);

#endif // MCMPC_H