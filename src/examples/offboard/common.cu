#include "px4_ros_com/common.cuh"

// ブロック数決定
unsigned int COM::count_blocks(unsigned int thread_num, unsigned int thread_per_block){
    unsigned int num;
    num = thread_num / thread_per_block;
    if (thread_num < thread_per_block || thread_num % thread_per_block > 0){
        num++;
    }
    return num;
}


/* 乱数配列生成 */
__global__ void COM::make_rand_arr(curandState *d_rnd, long long int seed){
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	curand_init(seed, idx, 0, &d_rnd[idx]);
}