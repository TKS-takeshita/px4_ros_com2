#include "MCMPC.cuh"
#include <iostream>
#include <thrust/fill.h>
#include <thrust/host_vector.h>

// Public

MCMPC::MCMPC(float DT, float in_sdev[], float in_min[], float in_max[], float param[],
             float stt_min[], float stt_max[], float cstR[], float cstQ[], float cstQf[],
             float CST_BARRIER, float LAMBDA, int ITERATION, enum solver SOLVER, float stt_ref[])
    : dt(DT), SOLVER(SOLVER), CST_BARRIER(CST_BARRIER), LAMBDA(LAMBDA), ITERATION(ITERATION)
{
    cudaMalloc((void**)&d_rnd, DIM_IN*HRIZON_NUM*SAMP_NUM*sizeof(curandState)); //デバイス乱数メモリ確保

    // デバイスメモリ確保
    //デバイスメモリ確保
	cudaMallocManaged((void**)&(this->in_seq),     DIM_IN*HRIZON_NUM*SAMP_NUM*sizeof(float));//全サンプル*全ホライズン*入力次元分の制御入力列
	cudaMallocManaged((void**)&(this->in_seq_opt), DIM_IN*HRIZON_NUM*sizeof(float));//最適制御入力列
	cudaMalloc((void**)&(this->in_sdev), DIM_IN*sizeof(float));
	cudaMalloc((void**)&(this->in_min),  DIM_IN*sizeof(float));
	cudaMalloc((void**)&(this->in_max),  DIM_IN*sizeof(float));
	cudaMalloc((void**)&(this->stt_ini), DIM_STT*sizeof(float));
	cudaMalloc((void**)&(this->param),   DIM_PRM*sizeof(float));
	cudaMalloc((void**)&(this->stt_ref), DIM_STT*sizeof(float));
	cudaMalloc((void**)&(this->stt_min), DIM_STT*sizeof(float));
	cudaMalloc((void**)&(this->stt_max), DIM_STT*sizeof(float));
	cudaMalloc((void**)&(this->cstR),    DIM_IN*sizeof(float));
	cudaMalloc((void**)&(this->cstQ),    DIM_STT*sizeof(float));
	cudaMalloc((void**)&(this->cstQf),   DIM_STT*sizeof(float));
	cudaMallocManaged((void**)&(this->costs),     SAMP_NUM*sizeof(float));
	cudaMallocManaged((void**)&(this->costs_exp), SAMP_NUM*sizeof(float));

    //ブロック数・スレッド数計算
	BLOCK_NUM       = COM::count_blocks(SAMP_NUM, THREAD_P_BLOCK);
	BLOCK_NUM_RND   = COM::count_blocks(DIM_IN*HRIZON_NUM*SAMP_NUM, THREAD_P_BLOCK);

	//入力列用乱数生成
	srand((unsigned int)time(NULL));
	COM::make_rand_arr<<<BLOCK_NUM_RND, THREAD_P_BLOCK>>>(d_rnd, rand());

	//変数代入
	thrust::fill(this->in_seq,     (this->in_seq)+DIM_IN*HRIZON_NUM*SAMP_NUM, 0.0f);
	thrust::fill(this->in_seq_opt, (this->in_seq_opt)+DIM_IN*HRIZON_NUM, 0.0f);
	this->DT = DT;
	cudaMemcpy(this->in_sdev, in_sdev, DIM_IN*sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(this->in_min,  in_min,  DIM_IN*sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(this->in_max,  in_max,  DIM_IN*sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(this->stt_ini, stt_ini, DIM_STT*sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(this->param,   param,   DIM_PRM*sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(this->stt_ref, stt_ref, DIM_STT*sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(this->stt_min, stt_min, DIM_STT*sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(this->stt_max, stt_max, DIM_STT*sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(this->cstR,    cstR,    DIM_IN*sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(this->cstQ,    cstQ,    DIM_STT*sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(this->cstQf,   cstQf,   DIM_STT*sizeof(float), cudaMemcpyHostToDevice);
	this->CST_BARRIER = CST_BARRIER;
	this->LAMBDA = LAMBDA;
	this->ITERATION = ITERATION;
	this->SOLVER = SOLVER;
	thrust::fill(this->costs,   (this->costs)+SAMP_NUM, 0.0f);
	thrust::host_vector<int> idx_seq_host(SAMP_NUM);
	//連番配列
	this->idx_seq = idx_seq_host;
	thrust::sequence(idx_seq.begin(), idx_seq.end());

	CHECK(cudaDeviceSynchronize());
}

// 最適入力列計算
void MCMPC::get_opt_in(float return_in[], float stt_ini[], float stt_ref[]){
	// 現時刻状態量取得
	cudaMemcpy(this->stt_ini, stt_ini, DIM_STT*sizeof(float), cudaMemcpyHostToDevice);
	// 目標状態量取得
	if(stt_ref!=NULL)
		cudaMemcpy(this->stt_ref, stt_ref, DIM_STT*sizeof(float), cudaMemcpyHostToDevice);

	// 前時刻最適入力列を1stepシフト
	memmove(in_seq_opt, in_seq_opt + DIM_IN,
        DIM_IN*(HRIZON_NUM-1)*sizeof(float));

	CHECK(cudaDeviceSynchronize());

	// 反復計算
	for(int itr_num=0; itr_num<ITERATION; itr_num++){
		// 乱数入力列生成&予測シミュ&コスト計算
		MCMPC_predictive_simulate<<<BLOCK_NUM, THREAD_P_BLOCK>>>(
			d_rnd, in_seq, in_seq_opt, in_sdev, in_min, in_max, this->stt_ini, DT, param, 
			this->stt_ref, stt_min, stt_max, cstR, cstQ, cstQf, CST_BARRIER,
			costs, costs_exp, cstQf, LAMBDA, SOLVER);

		// CHECK(cudaDeviceSynchronize());

		thrust::sequence(idx_seq.begin(), idx_seq.end());
		// CHECK(cudaDeviceSynchronize());

		thrust::sort_by_key(costs, costs+SAMP_NUM, idx_seq.begin());
		// CHECK(cudaDeviceSynchronize());

		// エリートサンプルの加重平均計算
		int idx_elite;
		float in_seq_opt_num[HRIZON_NUM*DIM_IN] = {};
		float in_seq_opt_den[HRIZON_NUM*DIM_IN] = {};

		// float cost_elite_sum = 0.0f;
		float cost_elite_sum = thrust::reduce(costs, costs+SAMP_NUM_ELITE, 0.0f, thrust::plus<float>());
		float inv_cost_elite_sum = 1.0f / cost_elite_sum;
		// CHECK(cudaDeviceSynchronize());
		for(int k=0; k<SAMP_NUM_ELITE; k++){
			idx_elite = idx_seq[k];
			float cost_exp = expf(-costs[k] * inv_cost_elite_sum);
			for(int i=0; i<HRIZON_NUM; i++){
				for(int j=0; j<DIM_IN; j++){
					int num_opt = i*DIM_IN + j;
					in_seq_opt_num[num_opt] += in_seq[idx_elite*HRIZON_NUM*DIM_IN + (num_opt)]* cost_exp;
					in_seq_opt_den[num_opt] += cost_exp;
				}
			}
		}
		thrust::transform(in_seq_opt_num, in_seq_opt_num+HRIZON_NUM*DIM_IN,
						in_seq_opt_den,
						in_seq_opt,
						thrust::divides<float>());
		CHECK(cudaDeviceSynchronize());
	}
	// 最適入力return
	memcpy(return_in, in_seq_opt, DIM_IN*sizeof(float));
}

void MCMPC::update_param(float new_param[]){
	cudaMemcpy(this->param, new_param, DIM_PRM*sizeof(float), cudaMemcpyHostToDevice);
}

MCMPC::~MCMPC(){
	cudaFree(d_rnd);

	cudaFree(in_seq);
	cudaFree(in_seq_opt);
	cudaFree(in_sdev);
	cudaFree(in_min);
	cudaFree(in_max);
	cudaFree(stt_ini);
	cudaFree(param);
	cudaFree(stt_ref);
	cudaFree(stt_min);
	cudaFree(stt_max);
	cudaFree(cstR);
	cudaFree(cstQ);
	cudaFree(cstQf);
	cudaFree(costs);
	cudaFree(costs_exp);
}

// CUDA
// 乱数入力列生成&予測シミュ＆コスト計算
__global__ void MCMPC_predictive_simulate(curandState* d_rnd, 
	float in_seq[], float in_seq_opt[], float in_sdev[], float in_min[], float in_max[],
	float stt_init[], float DT, float param[],
	float cstR[], float stt_min[], float stt_max[],
	float costs[], float costs_exp[], float LAMBDA, enum solver SOLVER){
	
	const int SAMP_NUM = SAMPLE_NUM_OF_MCMPC;
	const int HRIZON_NUM = HORIZON_NUM_OF_MCMPC;
	const int DIM_IN = DIMENTION_OF_INPUT;
	int idx = blockIdx.x * blockDim.x + threadIdx.x;

	if(idx<SAMP_NUM){
		// 前時刻の最適入力列から新入力列生成
		MCMPC_gen_input_seq(in_seq, d_rnd, idx, in_seq_opt, in_sdev, in_min, in_max);
		// 予測シミュ&コスト計算
		Plant plant;
		plant.initialize(stt_init, param, DT);

		float cost_tmp = 0.0f;
		int num;

		// 予測シミュレーション
		for(int tm=0; tm < HRIZON_NUM; tm++){
			num = idx * HORIZON_NUM * DIM_IN + tm * DIM_IN;
			plant.simulation(in_seq+num, SOLVER);
			// コスト計算
			cost_tmp += MCMPC_cost_func(in_seq+num, plant.state, stt_ref, stt_min, stt_max, cstR, cstQ, CST_BARRIER);
		}
		float cstRzero[DIM_IN] = {};
		// 終端コスト計算
		cost_tmp += MCMPC_cost_func(in_seq+num, plant.state, stt_ref, stt_min, stt_max, cstRzero, cstQf, CST_BARRIER);
		bool is_nan = __isnanf(cost_tmp);
		costs[idx] = is_nan ? INFINITY : cost_tmp;
		costs_exp[idx] = is_nan ? 0.0f : expf(-cost_tmp/LAMBDA);
	}
}

__device__ float MCMPC_cost_func(float in[], float stt[], float stt_ref[], float stt_min[], float stt_max[], 
								 float cstR[], float cstQ[], float CST_BARRIER){
	const int DIM_IN = DIMENTION_OF_INPUT;
	const int DIM_STT = DIMENTION_OF_STATE;
	float cost = 0.0f;

	// 入力コスト
	for(int i=0; i<DIM_IN; i++) cost += cstR[i]*in[i]*in[i];
	// 状態量コスト
	for(int j=0; j<DIM_STT; j++) {
		cost += (stt[j] < stt_min[j] || stt[j] > stt_max[j]) ? CST_BARRIER : 0.0f;
		cost += cstQ[j] * (stt[j] - stt_ref[j]) * (stt[j] - stt_ref[j]);
	}

	// 位置制約
	// cost += (stt[7] < -10.0f || stt[8] > 10.0f) * CST_BARRIER;
	return cost;
}


// 入力列乱数生成　サンプルidx番目
__device__ void MCMPC_gen_input_seq(float in_seq[],
                                     curandState* d_rnd, int idx,
                                     float in_seq_opt[], float in_sdev[],
                                     float in_min[], float in_max[]) {
	const int DIM_IN = DIMENTION_OF_INPUT;
	const int HRIZON_NUM = HORIZON_NUM_OF_MCMPC;
	int num;
	int base = idx*HRIZON_NUM*DIM_IN;

	// 前時刻の最適入力列取得
	memcpy(in_tmp_pre, in_seq_opt, DIM_IN*HRIZON_NUM*sizeof(float));

	// 前時刻の最適入力を平均とした正規分布乱数列生成
	for (int i = 0; i < HRIZON_NUM; i++) {
		for(int j=0; j<DIM_IN; j++){
			num_opt = i*DIM_IN + j;
			num = base + num_opt;
			//入力乱数生成
			float u = in_seq_opt[num_opt] + in_sdev[j] * COM::curand_normal(&d_rnd[num]);
			// 入力制約
			u = fminf(fmaxf(u, in_min[j]), in_max[j]);
			in_seq[num] = u;
		}
	}
}
