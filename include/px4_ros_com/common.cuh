#ifndef COMMON_CUH
#define COMMON_CUH
#include "common.cuh"

#include <random>
#include <math.h>
#include <stdlib.h>

#include <cuda_runtime.h>
#include <curand.h>
#include <curand_kernel.h>

//mode
// #define OUTPUT_PARTICLE_DATA_FILE //各粒子データファイルの出力

//plant
#define DIMENTION_OF_STATE  13 //状態ベクトルの次元
#define DIMENTION_OF_INPUT  4 //入力ベクトルの次元
#define DIMENTION_OF_OUTPUT 13 //出力ベクトルの次元
#define DIMENTION_OF_PARAM  7 //パラメータベクトルの次元

//PF同定
#define SAMPLE_NUM_OF_PF_SI 4098 //PF同定のサンプル数
#define HORIZON_NUM_OF_PF_SI 75 //PF同定のホライズン
const int PF_SI_THREAD_PER_BLOCK = 256;

//MCMPC
#define SAMPLE_NUM_OF_MCMPC 8192 //MCMPCのサンプル数
#define HORIZON_NUM_OF_MCMPC 75 //MCMPCのホライズン
#define SAMPLE_NUM_OF_MCMPC_ELITE 100 //MCMPCのエリートサンプル数
const int MCMPC_THREAD_PER_BLOCK = 64;

enum solver{
	eul,
	rk4,
	col_eul,
	col_rk4
};

namespace COM {
	unsigned int count_blocks(unsigned int thread_num, unsigned int thread_per_block);
	__global__ void make_rand_arr(curandState *d_rnd, long long int seed);
} // namespace COM

#endif
