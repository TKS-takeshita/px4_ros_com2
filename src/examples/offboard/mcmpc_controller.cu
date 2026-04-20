#include <iostream>

#include <cuda.h>
#include <curand.h>
#include <curand_kernel.h>
#include <stdlib.h>

#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/sequence.h>
#include <thrust/copy.h>
#include <thrust/sort.h>

#include "px4_ros_com/const_params.hpp"
#include "px4_ros_com/mcmpc_controller.cuh"
#include "mpc_simulator.cu" //__device__関数をインライン展開のため, 分割コンパイルせず直接include

namespace qc_mcmpc
{
	// 実体を定義(externを外す)
	__constant__ target_state_t target_state_device;

	__constant__ float u_g_device;
	__constant__ float u_upper_lim_device;
	__constant__ float u_lower_lim_device;

	__constant__ float i_xx_device;
	__constant__ float i_yy_device;
	__constant__ float i_zz_device;
	__constant__ float rotor_distance_device;
	__constant__ float max_rps_pow_device;
	__constant__ float max_thrust_device;
	__constant__ float mass_of_machine_device;
	__constant__ float torque_rate_device;
	__constant__ float a_of_gravity_device;

#ifdef PREDICTABLE_COLLISION_WITH_WALL
	__constant__ float x_wall_device;
	__constant__ float wall_nv_x_device;
	__constant__ float wall_nv_y_device;
	__constant__ float wall_nv_z_device;
	__constant__ float r_of_ring_device;
	__constant__ float coeff_of_rest_device;
#endif

	__constant__ float control_period_device;
	__constant__ float integration_step_size_device;

	__constant__ float var_and_z_i_device[_N_OF_ODES + 1];
	__constant__ input_array average_input_device;
	__constant__ float sigma_k_device[4];

	__global__ static void init_curand_seed(curandState *state_array, int seed);
	__global__ static void generate_input_samples_and_calc_costs(curandState *state, input_array* input_array_sample_device, float* cost_vec);
	__global__ static void extract_elite_sample(input_array* src, input_array* dst, int* elite_indices);

	// コスト再計算用関数
	static void input_constraint_cpu(float& rps_z, float& rps_wx, float& rps_wy, float& rps_ws);

#ifdef PREDICTABLE_COLLISION_WITH_WALL
	static float dot_vec_cpu(float v1_x, float v1_y, float v1_z, float v2_x, float v2_y, float v2_z);
	static void cross_vec_cpu(float v1_x, float v1_y, float v1_z, float v2_x, float v2_y, float v2_z, float& v_ans_x, float& v_ans_y, float& v_ans_z);
	static void inverse_3x3__cpu(float matrix_src[3][3], float ans[3][3]);
	static void calculate_deviations_cpu(float var_and_z_i[], float var_p[], float rps_cw1, float rps_cw2, float rps_ccw1, float rps_ccw2, bool& col_flag, float v_plus[], float w_plus[]);
#else
	static void calculate_deviations_cpu(float var_and_z_i[], float var_p[], float rps_cw1, float rps_cw2, float rps_ccw1, float rps_ccw2);
#endif

	// コンストラクタ
	mcmpc_controller::mcmpc_controller()
	{
		// 目標状態の設定
		target_state_t init_target{};
		init_target.e0 = CONST_PARAM_FLOAT::INIT_TARGET_E0;
		init_target.e1 = CONST_PARAM_FLOAT::INIT_TARGET_E1;
		init_target.e2 = CONST_PARAM_FLOAT::INIT_TARGET_E2;
		init_target.e3 = CONST_PARAM_FLOAT::INIT_TARGET_E3;
		init_target.wx = CONST_PARAM_FLOAT::INIT_TARGET_WX;
		init_target.wy = CONST_PARAM_FLOAT::INIT_TARGET_WY;
		init_target.wz = CONST_PARAM_FLOAT::INIT_TARGET_WZ;
		init_target.x = CONST_PARAM_FLOAT::INIT_TARGET_X;
		init_target.y = CONST_PARAM_FLOAT::INIT_TARGET_Y;
		init_target.z = CONST_PARAM_FLOAT::INIT_TARGET_Z;
		init_target.xp = CONST_PARAM_FLOAT::INIT_TARGET_ZP;
		init_target.yp = CONST_PARAM_FLOAT::INIT_TARGET_YP;
		init_target.zp = CONST_PARAM_FLOAT::INIT_TARGET_ZP;

		// 分散の設定　（変数なのは分散固定化が暫定的措置であるため）
		for(int i=0; i<4; i++)
			sigma_k[i] = CONST_PARAM_FLOAT::SIGMA_CONST[i];

		// PIDカスケード用修正
		for(int i = 0; i < _DEVICE_CONST_HORIZON; i++){
			best_input_array.decoupled_rps[i][0] = CONST_PARAM::INIT_U_THRUST;
			for(int j= 1; j<4; j++)
				best_input_array.decoupled_rps[i][j] = 0.0f;
		}
		// curandの乱数シード設定
		cudaMalloc(&curand_state_array, CONST_PARAM::N_OF_SAMPLES * sizeof(curandState));
		init_curand_seed<<< _DEVICE_CONST_N_OF_BLOCK, _DEVICE_CONST_THREAD_PER_BLOCK >>>(curand_state_array, (unsigned)time(NULL));

		// device_vector を生成
		thrust::device_vector<input_array> input_vec_dev_temp(CONST_PARAM::N_OF_SAMPLES);
		input_array_device_vec = input_vec_dev_temp;

		thrust::device_vector<input_array> input_vec_dev_elite_temp(CONST_PARAM::N_OF_THE_USING_BEST);
		input_array_device_vec_elite = input_vec_dev_elite_temp;

		thrust::device_vector<int> indices_vec_dev_temp(CONST_PARAM::N_OF_SAMPLES);
		indices_device_vec = indices_vec_dev_temp;

		thrust::device_vector<float> cost_vec_dev_temp( CONST_PARAM::N_OF_SAMPLES );
		cost_device_vec_for_sorting = cost_vec_dev_temp;

		// host_vectorを生成
		thrust::host_vector<input_array> input_vec_host_elite_temp( CONST_PARAM::N_OF_THE_USING_BEST );
		input_array_host_vec_elite = input_vec_host_elite_temp;

		//  __constant__ メモリに定数をコピー
		cudaMemcpyToSymbol(target_state_device, &init_target, sizeof(target_state_t));

		cudaMemcpyToSymbol( sigma_k_device, sigma_k, 4 * sizeof( float ) );

        cudaMemcpyToSymbol( u_g_device, &CONST_PARAM_FLOAT::U_G, sizeof( float ) );
        cudaMemcpyToSymbol( u_upper_lim_device, &CONST_PARAM_FLOAT::U_UPPER_LIM, sizeof( float ) );
        cudaMemcpyToSymbol( u_lower_lim_device, &CONST_PARAM_FLOAT::U_LOWER_LIM, sizeof( float ) );

        cudaMemcpyToSymbol( i_xx_device, &CONST_PARAM_FLOAT::I_XX, sizeof( float ) );
        cudaMemcpyToSymbol( i_yy_device, &CONST_PARAM_FLOAT::I_YY, sizeof( float ) );
        cudaMemcpyToSymbol( i_zz_device, &CONST_PARAM_FLOAT::I_ZZ, sizeof( float ) );

        cudaMemcpyToSymbol( rotor_distance_device,  &CONST_PARAM_FLOAT::ROTOR_DISTANCE,  sizeof( float ) );
        cudaMemcpyToSymbol( max_rps_pow_device,     &CONST_PARAM_FLOAT::MAX_RPS_POW,     sizeof( float ) );
        cudaMemcpyToSymbol( max_thrust_device,      &CONST_PARAM_FLOAT::MAX_THRUST,      sizeof( float ) );
        cudaMemcpyToSymbol( mass_of_machine_device, &CONST_PARAM_FLOAT::MASS_OF_MACHINE, sizeof( float ) );
        cudaMemcpyToSymbol( torque_rate_device,     &CONST_PARAM_FLOAT::TORQUE_RATE,     sizeof( float ) );
        cudaMemcpyToSymbol( a_of_gravity_device,    &CONST_PARAM_FLOAT::A_OF_GRAVITY,    sizeof( float ) );

        cudaMemcpyToSymbol( control_period_device,        &CONST_PARAM_FLOAT::CONTROL_PERIOD,        sizeof( float ) );
        cudaMemcpyToSymbol( integration_step_size_device, &CONST_PARAM_FLOAT::INTEGRATION_STEP_SIZE, sizeof( float ) );

#ifdef PREDICTABLE_COLLISION_WITH_WALL
        cudaMemcpyToSymbol( x_wall_device,        &CONST_PARAM_FLOAT::X_WALL,               sizeof( float ) );
        cudaMemcpyToSymbol( wall_nv_x_device,     &CONST_PARAM_FLOAT::WALL_NORMAL_VECTOR_X, sizeof( float ) );
        cudaMemcpyToSymbol( wall_nv_y_device,     &CONST_PARAM_FLOAT::WALL_NORMAL_VECTOR_Y, sizeof( float ) );
        cudaMemcpyToSymbol( wall_nv_z_device,     &CONST_PARAM_FLOAT::WALL_NORMAL_VECTOR_Z, sizeof( float ) );
        cudaMemcpyToSymbol( r_of_ring_device,     &CONST_PARAM_FLOAT::R_OF_RING,            sizeof( float ) );
        cudaMemcpyToSymbol( coeff_of_rest_device, &CONST_PARAM_FLOAT::COEFF_OF_REST,        sizeof( float ) );
#endif
	}

	// デストラクタ
	mcmpc_controller::~mcmpc_controller()
	{
		cudaFree(curand_state_array);
	}

	// 乱数シードの初期化
	__global__ static void init_curand_seed( curandState* state_array, int seed )
    {
        int id = blockDim.x * blockIdx.x + threadIdx.x;
        curand_init( seed, id, 0, &state_array[id] );
    }

	// GPU上で入力列を生成, シミュレーションしコストを求める
	__global__ static void generate_input_samples_and_calc_costs(curandState* state, input_array* input_array_sample_device, float* cost_vec){
		int id = blockDim.x * blockIdx.x + threadIdx.x;

		input_array_sample_device[id].generate_input_array(state[id]);
		input_array_sample_device[id].do_simulation();

		// sort用に, float配列にコストを同順でコピー
		cost_vec[id] = input_array_sample_device[id].cost;
	}

	// エリートサンプルだけの入力列 devivce_vectorをGPUで生成 (ホストへの大量転送, ホストからのランダムアクセスを防ぐ)
	__global__ static void extract_elite_sample(input_array* src, input_array* dst, int* elite_indices){
		int id = blockDim.x * blockIdx.x + threadIdx.x;//blockDim.x = 1, threadIdx.x = 0

		dst[id].cost = src[elite_indices[id]].cost;
		for(int i=0; i< _DEVICE_CONST_HORIZON; i++){
			for(int j=0; j<4; j++){
				dst[id].decoupled_rps[i][j] = src[elite_indices[id]].decoupled_rps[i][j];
			}
		}
	}

	static void input_constraint_cpu(float& rps_cw1, float& rps_cw2, float& rps_ccw1, float& rps_ccw2){
		// rps_cw1
		if(rps_cw1 > CONST_PARAM_FLOAT::U_UPPER_LIM) rps_cw1 = CONST_PARAM_FLOAT::U_UPPER_LIM;
		if(rps_cw1 < CONST_PARAM_FLOAT::U_LOWER_LIM) rps_cw1 = CONST_PARAM_FLOAT::U_LOWER_LIM;
		// rps_cw2
		if(rps_cw2 > CONST_PARAM_FLOAT::U_UPPER_LIM) rps_cw2 = CONST_PARAM_FLOAT::U_UPPER_LIM;
		if(rps_cw2 < CONST_PARAM_FLOAT::U_LOWER_LIM) rps_cw2 = CONST_PARAM_FLOAT::U_LOWER_LIM;
		// rps_ccw1
		if(rps_ccw1 > CONST_PARAM_FLOAT::U_UPPER_LIM) rps_ccw1 = CONST_PARAM_FLOAT::U_UPPER_LIM;
		if(rps_ccw1 < CONST_PARAM_FLOAT::U_LOWER_LIM) rps_ccw1 = CONST_PARAM_FLOAT::U_LOWER_LIM;
		// rps_ccw2
		if(rps_ccw2 > CONST_PARAM_FLOAT::U_UPPER_LIM) rps_ccw2 = CONST_PARAM_FLOAT::U_UPPER_LIM;
		if(rps_ccw2 < CONST_PARAM_FLOAT::U_LOWER_LIM) rps_ccw2 = CONST_PARAM_FLOAT::U_LOWER_LIM;
	}

	void update_target_state_device(const target_state_t& target)
	{
        cudaMemcpyToSymbol(target_state_device, &target, sizeof(target_state_t));
    }

	// 運動方程式
#ifdef PREDICTABLE_COLLISION_WITH_WALL
	// 衝突予測あり
	static float dot_vec_cpu(float v1_x, float v1_y, float v1_z, float v2_x, float v2_y, float v2_z){
		return v1_x * v2_x + v1_y * v2_y + v1_z * v2_z;
	}

	static void cross_vec_cpu(float v1_x, float v1_y, float v1_z, float v2_x, float v2_y, float v2_z, float& v_ans_x, float& v_ans_y, float& v_ans_z){
		v_ans_x = v1_y * v2_z - v1_z * v2_y;
		v_ans_y = v1_z * v2_x - v1_x * v2_z;
		v_ans_z = v1_x * v2_y - v1_y * v2_x;
	}

	static void inverse_3x3__cpu(float matrix_src[3][3], float ans[3][3]){
		float det  = matrix_src[0][0]*matrix_src[1][1]*matrix_src[2][2];
              det += matrix_src[1][0]*matrix_src[2][1]*matrix_src[0][2];
              det += matrix_src[2][0]*matrix_src[0][1]*matrix_src[1][2];
              det -= matrix_src[2][0]*matrix_src[1][1]*matrix_src[0][2];
              det -= matrix_src[1][0]*matrix_src[0][1]*matrix_src[2][2];
              det -= matrix_src[0][0]*matrix_src[2][1]*matrix_src[1][2];
        
        ans[0][0] =  ( matrix_src[1][1]*matrix_src[2][2] - matrix_src[1][2]*matrix_src[2][1] ) / det;
        ans[0][1] = -( matrix_src[1][0]*matrix_src[2][2] - matrix_src[1][2]*matrix_src[2][0] ) / det;
        ans[0][2] =  ( matrix_src[1][0]*matrix_src[2][1] - matrix_src[1][1]*matrix_src[2][0] ) / det;
        ans[1][0] = -( matrix_src[0][1]*matrix_src[2][2] - matrix_src[0][2]*matrix_src[2][1] ) / det;
        ans[1][1] =  ( matrix_src[0][0]*matrix_src[2][2] - matrix_src[0][2]*matrix_src[2][0] ) / det;
        ans[1][2] = -( matrix_src[0][0]*matrix_src[2][1] - matrix_src[0][1]*matrix_src[2][0] ) / det;
        ans[2][0] =  ( matrix_src[0][1]*matrix_src[1][2] - matrix_src[0][2]*matrix_src[1][1] ) / det;
        ans[2][1] = -( matrix_src[0][0]*matrix_src[1][2] - matrix_src[0][2]*matrix_src[1][0] ) / det;
        ans[2][2] =  ( matrix_src[0][0]*matrix_src[1][1] - matrix_src[0][1]*matrix_src[1][0] ) / det;
	}

	static void calculate_deviations_cpu(float var_and_z_i[], float var_p[], float rps_cw1, float rps_cw2, float rps_ccw1, float rps_ccw2, bool& col_flag, float v_plus[], float w_plus[] ){
		// === 衝突判定 & 衝突後速度/角速度計算 ===

		// ガードリングの法線ベクトル
		float ring_nv_x = 2.0f*var_and_z_i[0]*var_and_z_i[2] + 2.0f*var_and_z_i[1]*var_and_z_i[3];
        float ring_nv_y = 2.0f*var_and_z_i[2]*var_and_z_i[3] - 2.0f*var_and_z_i[0]*var_and_z_i[1];
        float ring_nv_z = -1.0f + 2.0f*var_and_z_i[0]*var_and_z_i[0] + 2.0f*var_and_z_i[3]*var_and_z_i[3];

		// 衝突の判別式, リングと壁の共有点が存在するか
		float discriminant = ( 2.0f*ring_nv_x*ring_nv_y*(CONST_PARAM_FLOAT::X_WALL-var_and_z_i[7]) - 2.0f*(ring_nv_y*ring_nv_y+ring_nv_z*ring_nv_z)*var_and_z_i[8] )*( 2.0f*ring_nv_x*ring_nv_y*(CONST_PARAM_FLOAT::X_WALL-var_and_z_i[7]) - 2.0f*(ring_nv_y*ring_nv_y+ring_nv_z*ring_nv_z)*var_and_z_i[8] )
                            - 4*(ring_nv_y*ring_nv_y+ring_nv_z*ring_nv_z)*( (ring_nv_x*ring_nv_x+ring_nv_z*ring_nv_z)*(CONST_PARAM_FLOAT::X_WALL-var_and_z_i[7])*(CONST_PARAM_FLOAT::X_WALL-var_and_z_i[7]) - 2.0f*ring_nv_x*ring_nv_y*(CONST_PARAM_FLOAT::X_WALL-var_and_z_i[7])*var_and_z_i[8] + (ring_nv_y*ring_nv_y+ring_nv_z*ring_nv_z)*var_and_z_i[8]*var_and_z_i[8] - (ring_nv_z*CONST_PARAM_FLOAT::R_OF_RING)*(ring_nv_z*CONST_PARAM_FLOAT::R_OF_RING) );


		// col_flag は撃力の2倍印加防止と, コントローラへ衝突情報を伝える
		if(discriminant >= 0.0f && !col_flag){
			// 衝突前の速度
			float v_minus[3];
			v_minus[0] = var_and_z_i[10];
            v_minus[1] = var_and_z_i[11];
            v_minus[2] = var_and_z_i[12];

			// 衝突前の角速度(世界座標系に変換)
			float w_minus[3];
			w_minus[0] = 2.0f*var_and_z_i[6]*(var_and_z_i[0]*var_and_z_i[2]+var_and_z_i[1]*var_and_z_i[3]) + var_and_z_i[4]*(-1.0f+2.0f*var_and_z_i[0]*var_and_z_i[0]+2.0f*var_and_z_i[1]*var_and_z_i[1]) - 2.0f*var_and_z_i[5]*(var_and_z_i[0]*var_and_z_i[3]-var_and_z_i[1]*var_and_z_i[2]);
            w_minus[1] = 2.0f*var_and_z_i[4]*(var_and_z_i[0]*var_and_z_i[3]+var_and_z_i[1]*var_and_z_i[2]) + var_and_z_i[5]*(-1.0f+2.0f*var_and_z_i[0]*var_and_z_i[0]+2.0f*var_and_z_i[2]*var_and_z_i[2]) - 2.0f*var_and_z_i[6]*(var_and_z_i[0]*var_and_z_i[1]-var_and_z_i[2]*var_and_z_i[3]);
            w_minus[2] = 2.0f*var_and_z_i[5]*(var_and_z_i[0]*var_and_z_i[1]+var_and_z_i[2]*var_and_z_i[3]) + var_and_z_i[6]*(-1.0f+2.0f*var_and_z_i[0]*var_and_z_i[0]+2.0f*var_and_z_i[3]*var_and_z_i[3]) - 2.0f*var_and_z_i[4]*(var_and_z_i[0]*var_and_z_i[2]-var_and_z_i[1]*var_and_z_i[3]);

			// 機体の質量中心から衝突点までのベクトル
			float r_vec[3];
			r_vec[0] = CONST_PARAM_FLOAT::X_WALL - var_and_z_i[7];
			r_vec[1] = -(ring_nv_x*ring_nv_y*(CONST_PARAM_FLOAT::X_WALL-var_and_z_i[7]) - ring_nv_y*ring_nv_y+ring_nv_z*ring_nv_z)*var_and_z_i[8] ) / ( ring_nv_y*ring_nv_y+ring_nv_z*ring_nv_z ) - var_and_z_i[8];
			r_vec[2] = -(ring_nv_x*r_vec[0] + ring_nv_y*r_vec[1] ) / ring_nv_z;

			// 機体の慣性テンソル (世界座標系)
			float i_tensor[3][3];
			i_tensor[0][0] = 4.0f*CONST_PARAM_FLOAT::I_ZZ*(var_and_z_i[0]*var_and_z_i[2]+var_and_z_i[1]*var_and_z_i[3])*(var_and_z_i[0]*var_and_z_i[2]+var_and_z_i[1]*var_and_z_i[3])+4.0f*CONST_PARAM_FLOAT::I_YY*(var_and_z_i[0]*var_and_z_i[3]-var_and_z_i[1]*var_and_z_i[2])*(var_and_z_i[0]*var_and_z_i[3]-var_and_z_i[1]*var_and_z_i[2])+CONST_PARAM_FLOAT::I_XX*(-1.0f+2.0f*var_and_z_i[0]*var_and_z_i[0]+2.0f*var_and_z_i[1]*var_and_z_i[1])*(-1.0f+2.0f*var_and_z_i[0]*var_and_z_i[0]+2.0f*var_and_z_i[1]*var_and_z_i[1]);
            i_tensor[0][1] = 2.0f*CONST_PARAM_FLOAT::I_XX*(var_and_z_i[0]*var_and_z_i[3]+var_and_z_i[1]*var_and_z_i[2])*(-1.0f+2.0f*var_and_z_i[0]*var_and_z_i[0]+2.0f*var_and_z_i[1]*var_and_z_i[1])-4.0f*CONST_PARAM_FLOAT::I_ZZ*(var_and_z_i[0]*var_and_z_i[2]+var_and_z_i[1]*var_and_z_i[3])*(var_and_z_i[0]*var_and_z_i[1]-var_and_z_i[2]*var_and_z_i[3])-2.0f*CONST_PARAM_FLOAT::I_YY*(var_and_z_i[0]*var_and_z_i[3]-var_and_z_i[1]*var_and_z_i[2])*(-1.0f+2.0f*var_and_z_i[0]*var_and_z_i[0]+2.0f*var_and_z_i[2]*var_and_z_i[2]);
            i_tensor[0][2] = 2.0f*CONST_PARAM_FLOAT::I_ZZ*(var_and_z_i[0]*var_and_z_i[2]+var_and_z_i[1]*var_and_z_i[3])*(-1.0f+2.0f*var_and_z_i[0]*var_and_z_i[0]+2.0f*var_and_z_i[3]*var_and_z_i[3])-4.0f*CONST_PARAM_FLOAT::I_YY*(var_and_z_i[0]*var_and_z_i[1]+var_and_z_i[2]*var_and_z_i[3])*(var_and_z_i[0]*var_and_z_i[3]-var_and_z_i[1]*var_and_z_i[2])-2.0f*CONST_PARAM_FLOAT::I_XX*(var_and_z_i[0]*var_and_z_i[2]-var_and_z_i[1]*var_and_z_i[3])*(-1.0f+2.0f*var_and_z_i[0]*var_and_z_i[0]+2.0f*var_and_z_i[1]*var_and_z_i[1]);
            i_tensor[1][0] = i_tensor[0][1];
            i_tensor[1][1] = 4.0f*CONST_PARAM_FLOAT::I_XX*(var_and_z_i[0]*var_and_z_i[3]+var_and_z_i[1]*var_and_z_i[2])*(var_and_z_i[0]*var_and_z_i[3]+var_and_z_i[1]*var_and_z_i[2])+4.0f*CONST_PARAM_FLOAT::I_ZZ*(var_and_z_i[0]*var_and_z_i[1]-var_and_z_i[2]*var_and_z_i[3])*(var_and_z_i[0]*var_and_z_i[1]-var_and_z_i[2]*var_and_z_i[3])+CONST_PARAM_FLOAT::I_YY*(-1.0f+2.0f*var_and_z_i[0]*var_and_z_i[0]+2.0f*var_and_z_i[2]*var_and_z_i[2])*(-1.0f+2.0f*var_and_z_i[0]*var_and_z_i[0]+2.0f*var_and_z_i[2]*var_and_z_i[2]);
            i_tensor[1][2] = 2.0f*CONST_PARAM_FLOAT::I_YY*(var_and_z_i[0]*var_and_z_i[1]+var_and_z_i[2]*var_and_z_i[3])*(-1.0f+2.0f*var_and_z_i[0]*var_and_z_i[0]+2.0f*var_and_z_i[2]*var_and_z_i[2])-4.0f*CONST_PARAM_FLOAT::I_XX*(var_and_z_i[0]*var_and_z_i[3]+var_and_z_i[1]*var_and_z_i[2])*(var_and_z_i[0]*var_and_z_i[2]-var_and_z_i[1]*var_and_z_i[3])-2.0f*CONST_PARAM_FLOAT::I_ZZ*(var_and_z_i[0]*var_and_z_i[1]-var_and_z_i[2]*var_and_z_i[3])*(-1.0f+2.0f*var_and_z_i[0]*var_and_z_i[0]+2.0f*var_and_z_i[3]*var_and_z_i[3]);
            i_tensor[2][0] = i_tensor[0][2];
            i_tensor[2][1] = i_tensor[1][2];
            i_tensor[2][2] = 4.0f*CONST_PARAM_FLOAT::I_YY*(var_and_z_i[0]*var_and_z_i[1]+var_and_z_i[2]*var_and_z_i[3])*(var_and_z_i[0]*var_and_z_i[1]+var_and_z_i[2]*var_and_z_i[3])+4.0f*CONST_PARAM_FLOAT::I_XX*(var_and_z_i[0]*var_and_z_i[2]-var_and_z_i[1]*var_and_z_i[3])*(var_and_z_i[0]*var_and_z_i[2]-var_and_z_i[1]*var_and_z_i[3])+CONST_PARAM_FLOAT::I_ZZ*(-1.0f+2.0f*var_and_z_i[0]*var_and_z_i[0]+2.0f*var_and_z_i[3]*var_and_z_i[3])*(-1.0f+2.0f*var_and_z_i[0]*var_and_z_i[0]+2.0f*var_and_z_i[3]*var_and_z_i[3]);

			float i_inverse[3][3];
			inverse_3x3__cpu(i_tensor, i_inverse);

			float vec_temp1[3];
			cross_vec_cpu(w_minus[0], w_minus[1], w_minus[2], r_vec[0], r_vec[1], r_vec[2], vec_temp1[0], vec_temp1[1], vec_temp1[2]);
			for(int i=0; i<3; i++)
				vec_temp1[i] = v_minus[i] + vec_temp1[i];

			float vec_temp2[3];
			cross_vec_cpu(r_vec[0], r_vec[1], r_vec[2], CONST_PARAM_FLOAT::WALL_NORMAL_VECTOR_X, CONST_PARAM_FLOAT::WALL_NORMAL_VECTOR_Y, CONST_PARAM_FLOAT::WALL_NORMAL_VECTOR_Z, vec_temp2[0], vec_temp2[1], vec_temp2[2]);
			for(int i=0; i<3; i++){
				vec_temp2[i] = i_inverse[i][0]*vec_temp2[0] + i_inverse[i][1]*vec_temp2[1] + i_inverse[i][2]*vec_temp2[2];
			}

			float vec_temp3[3];
			cross_vec_cpu(vec_temp2[0], vec_temp2[1], vec_temp2[2], r_vec[0], r_vec[1], r_vec[2], vec_temp3[0], vec_temp3[1], vec_temp3[2]);

			// 機体が壁から受ける力積の大きさ (向きは wall_nv で指定　判別式の都合上, 実質y-z平面に平行な壁しか扱えない)
			float col_ft = -( 1 + CONST_PARAM_FLOAT::COEFF_OF_REST ) * dot_vec_cpu( vec_temp1[0], vec_temp1[1], vec_temp1[2], CONST_PARAM_FLOAT::WALL_NORMAL_VECTOR_X, CONST_PARAM_FLOAT::WALL_NORMAL_VECTOR_Y, CONST_PARAM_FLOAT::WALL_NORMAL_VECTOR_Z )
                        / ( 1/CONST_PARAM_FLOAT::MASS_OF_MACHINE + dot_vec_cpu( vec_temp3[0], vec_temp3[1], vec_temp3[2], CONST_PARAM_FLOAT::WALL_NORMAL_VECTOR_X, CONST_PARAM_FLOAT::WALL_NORMAL_VECTOR_Y, CONST_PARAM_FLOAT::WALL_NORMAL_VECTOR_Z ) );

			// 衝突後の速度
			v_plus[0] = (col_ft/CONST_PARAM_FLOAT::MASS_OF_MACHINE) * CONST_PARAM_FLOAT::WALL_NORMAL_VECTOR_X + v_minus[0];
            v_plus[1] = (col_ft/CONST_PARAM_FLOAT::MASS_OF_MACHINE) * CONST_PARAM_FLOAT::WALL_NORMAL_VECTOR_Y + v_minus[1];
            v_plus[2] = (col_ft/CONST_PARAM_FLOAT::MASS_OF_MACHINE) * CONST_PARAM_FLOAT::WALL_NORMAL_VECTOR_Z + v_minus[2];

			float vec_temp4[3];
            cross_vec_cpu( r_vec[0], r_vec[1], r_vec[2], col_ft*CONST_PARAM_FLOAT::WALL_NORMAL_VECTOR_X, col_ft*CONST_PARAM_FLOAT::WALL_NORMAL_VECTOR_Y, col_ft*CONST_PARAM_FLOAT::WALL_NORMAL_VECTOR_Z, vec_temp4[0], vec_temp4[1], vec_temp4[2] );

			// 衝突後の角速度
			w_plus[0] = i_inverse[0][0]*vec_temp4[0] + i_inverse[0][1]*vec_temp4[1] + i_inverse[0][2]*vec_temp4[2] + w_minus[0];
            w_plus[1] = i_inverse[1][0]*vec_temp4[0] + i_inverse[1][1]*vec_temp4[1] + i_inverse[1][2]*vec_temp4[2] + w_minus[1];
            w_plus[2] = i_inverse[2][0]*vec_temp4[0] + i_inverse[2][1]*vec_temp4[1] + i_inverse[2][2]*vec_temp4[2] + w_minus[2];

            col_flag = true;
		}
		else if(discriminant < 0.0f)
			col_flag = false;

		/* e0p */ var_p[0] = -0.5f*var_and_z_i[1]*var_and_z_i[4] - 0.5f*var_and_z_i[2]*var_and_z_i[5] - 0.5f*var_and_z_i[3]*var_and_z_i[6];
        /* e1p */ var_p[1] =  0.5f*var_and_z_i[0]*var_and_z_i[4] + 0.5f*var_and_z_i[2]*var_and_z_i[6] - 0.5f*var_and_z_i[3]*var_and_z_i[5];
        /* e2p */ var_p[2] =  0.5f*var_and_z_i[0]*var_and_z_i[5] + 0.5f*var_and_z_i[3]*var_and_z_i[4] - 0.5f*var_and_z_i[1]*var_and_z_i[6];
        /* e3p */ var_p[3] =  0.5f*var_and_z_i[0]*var_and_z_i[6] + 0.5f*var_and_z_i[1]*var_and_z_i[5] - 0.5f*var_and_z_i[2]*var_and_z_i[4];

        /* wxp */ var_p[4] = ((CONST_PARAM_FLOAT::I_YY-CONST_PARAM_FLOAT::I_ZZ)*var_and_z_i[5]*var_and_z_i[6]+0.5f*INV_SQRT_2*CONST_PARAM_FLOAT::ROTOR_DISTANCE*CONST_PARAM_FLOAT::MAX_THRUST*(-rps_ccw1*fabsf(rps_ccw1)+rps_ccw2*fabsf(rps_ccw2)+rps_cw1*fabsf(rps_cw1)-rps_cw2*fabsf(rps_cw2))/CONST_PARAM_FLOAT::MAX_RPS_POW)/CONST_PARAM_FLOAT::I_XX;
        /* wyp */ var_p[5] = ((CONST_PARAM_FLOAT::I_ZZ-CONST_PARAM_FLOAT::I_XX)*var_and_z_i[4]*var_and_z_i[6]+0.5f*INV_SQRT_2*CONST_PARAM_FLOAT::ROTOR_DISTANCE*CONST_PARAM_FLOAT::MAX_THRUST*(-rps_ccw1*fabsf(rps_ccw1)+rps_ccw2*fabsf(rps_ccw2)-rps_cw1*fabsf(rps_cw1)+rps_cw2*fabsf(rps_cw2))/CONST_PARAM_FLOAT::MAX_RPS_POW)/CONST_PARAM_FLOAT::I_YY;
        /* wzp */ var_p[6] = ((CONST_PARAM_FLOAT::I_XX-CONST_PARAM_FLOAT::I_YY)*var_and_z_i[4]*var_and_z_i[5]-CONST_PARAM_FLOAT::TORQUE_RATE*(rps_ccw1*fabsf(rps_ccw1)+rps_ccw2*fabsf(rps_ccw2)-rps_cw1*fabsf(rps_cw1)-rps_cw2*fabsf(rps_cw2)))/CONST_PARAM_FLOAT::I_ZZ;

        /* xp  */ var_p[7] = var_and_z_i[10];
        /* yp  */ var_p[8] = var_and_z_i[11];
        /* zp  */ var_p[9] = var_and_z_i[12];

        /* xpp */ var_p[10] =  2.0f*CONST_PARAM_FLOAT::MAX_THRUST*(var_and_z_i[0]*var_and_z_i[2]+var_and_z_i[1]*var_and_z_i[3])*(rps_ccw1*fabsf(rps_ccw1)+rps_ccw2*fabsf(rps_ccw2)+rps_cw1*fabsf(rps_cw1)+rps_cw2*fabsf(rps_cw2))/(CONST_PARAM_FLOAT::MASS_OF_MACHINE*CONST_PARAM_FLOAT::MAX_RPS_POW);
        /* ypp */ var_p[11] = -2.0f*CONST_PARAM_FLOAT::MAX_THRUST*(var_and_z_i[0]*var_and_z_i[1]-var_and_z_i[2]*var_and_z_i[3])*(rps_ccw1*fabsf(rps_ccw1)+rps_ccw2*fabsf(rps_ccw2)+rps_cw1*fabsf(rps_cw1)+rps_cw2*fabsf(rps_cw2))/(CONST_PARAM_FLOAT::MASS_OF_MACHINE*CONST_PARAM_FLOAT::MAX_RPS_POW);
        /* zpp */ var_p[12] = CONST_PARAM_FLOAT::MAX_THRUST*(-1.0f+2.0f*var_and_z_i[0]*var_and_z_i[0]+2.0f*var_and_z_i[3]*var_and_z_i[3])*(rps_ccw1*fabsf(rps_ccw1)+rps_ccw2*fabsf(rps_ccw2)+rps_cw1*fabsf(rps_cw1)+rps_cw2*fabsf(rps_cw2))/(CONST_PARAM_FLOAT::MASS_OF_MACHINE*CONST_PARAM_FLOAT::MAX_RPS_POW) - CONST_PARAM_FLOAT::A_OF_GRAVITY;

	}
#else
	// 衝突予測なし
	static void calculate_deviations_cpu(float var_and_z_i[], float var_p[], float rps_cw1, float rps_cw2, float rps_ccw1, float rps_ccw2){
		/* e0p */ var_p[0] = -0.5f*var_and_z_i[1]*var_and_z_i[4] - 0.5f*var_and_z_i[2]*var_and_z_i[5] - 0.5f*var_and_z_i[3]*var_and_z_i[6];
		/* e1p */ var_p[1] =  0.5f*var_and_z_i[0]*var_and_z_i[4] + 0.5f*var_and_z_i[2]*var_and_z_i[6] - 0.5f*var_and_z_i[3]*var_and_z_i[5];
		/* e2p */ var_p[2] =  0.5f*var_and_z_i[0]*var_and_z_i[5] + 0.5f*var_and_z_i[3]*var_and_z_i[4] - 0.5f*var_and_z_i[1]*var_and_z_i[6];
		/* e3p */ var_p[3] =  0.5f*var_and_z_i[0]*var_and_z_i[6] + 0.5f*var_and_z_i[1]*var_and_z_i[5] - 0.5f*var_and_z_i[2]*var_and_z_i[4];

		/* wxp */ var_p[4] = ((CONST_PARAM_FLOAT::I_YY-CONST_PARAM_FLOAT::I_ZZ)*var_and_z_i[5]*var_and_z_i[6]+0.5f*INV_SQRT_2*CONST_PARAM_FLOAT::ROTOR_DISTANCE*CONST_PARAM_FLOAT::MAX_THRUST*(-rps_ccw1*fabsf(rps_ccw1)+rps_ccw2*fabsf(rps_ccw2)+rps_cw1*fabsf(rps_cw1)-rps_cw2*fabsf(rps_cw2))/CONST_PARAM_FLOAT::MAX_RPS_POW)/CONST_PARAM_FLOAT::I_XX;
		/* wyp */ var_p[5] = ((CONST_PARAM_FLOAT::I_ZZ-CONST_PARAM_FLOAT::I_XX)*var_and_z_i[4]*var_and_z_i[6]+0.5f*INV_SQRT_2*CONST_PARAM_FLOAT::ROTOR_DISTANCE*CONST_PARAM_FLOAT::MAX_THRUST*(-rps_ccw1*fabsf(rps_ccw1)+rps_ccw2*fabsf(rps_ccw2)-rps_cw1*fabsf(rps_cw1)+rps_cw2*fabsf(rps_cw2))/CONST_PARAM_FLOAT::MAX_RPS_POW)/CONST_PARAM_FLOAT::I_YY;
		/* wzp */ var_p[6] = ((CONST_PARAM_FLOAT::I_XX-CONST_PARAM_FLOAT::I_YY)*var_and_z_i[4]*var_and_z_i[5]-CONST_PARAM_FLOAT::TORQUE_RATE*(rps_ccw1*fabsf(rps_ccw1)+rps_ccw2*fabsf(rps_ccw2)-rps_cw1*fabsf(rps_cw1)-rps_cw2*fabsf(rps_cw2)))/CONST_PARAM_FLOAT::I_ZZ;

		/* xp  */ var_p[7] = var_and_z_i[10];
		/* yp  */ var_p[8] = var_and_z_i[11];
		/* zp  */ var_p[9] = var_and_z_i[12];
		
		/* xpp */ var_p[10] =  2.0f*CONST_PARAM_FLOAT::MAX_THRUST*(var_and_z_i[0]*var_and_z_i[2]+var_and_z_i[1]*var_and_z_i[3])*(rps_ccw1*fabsf(rps_ccw1)+rps_ccw2*fabsf(rps_ccw2)+rps_cw1*fabsf(rps_cw1)+rps_cw2*fabsf(rps_cw2))/(CONST_PARAM_FLOAT::MASS_OF_MACHINE*CONST_PARAM_FLOAT::MAX_RPS_POW);
		/* ypp */ var_p[11] = -2.0f*CONST_PARAM_FLOAT::MAX_THRUST*(var_and_z_i[0]*var_and_z_i[1]-var_and_z_i[2]*var_and_z_i[3])*(rps_ccw1*fabsf(rps_ccw1)+rps_ccw2*fabsf(rps_cw2)+rps_cw1*fabsf(rps_cw1)+rps_cw2*fabsf(rps_cw2))/(CONST_PARAM_FLOAT::MASS_OF_MACHINE*CONST_PARAM_FLOAT::MAX_RPS_POW);
		/* zpp */ var_p[12] = CONST_PARAM_FLOAT::MAX_THRUST*(-1.0f+2.0f*var_and_z_i[0]*var_and_z_i[0]+2.0f*var_and_z_i[3]*var_and_z_i[3])*(rps_ccw1*fabsf(rps_ccw1)+rps_ccw2*fabsf(rps_ccw2)+rps_cw1*fabsf(rps_cw1)+rps_cw2*fabsf(rps_cw2))/(CONST_PARAM_FLOAT::MASS_OF_MACHINE*CONST_PARAM_FLOAT::MAX_RPS_POW) - CONST_PARAM_FLOAT::A_OF_GRAVITY;
	}
#endif

	float mcmpc_controller::calc_weighted_average_and_min_cost(float var_and_z_i[])
	{
		// 0, 1, 2, ... となる昇順インデックスを生成
        thrust::sequence( indices_device_vec.begin(), indices_device_vec.end() );

		// cost_device_vec_for_sortingを昇順にインデックスをソート
        thrust::sort_by_key( cost_device_vec_for_sorting.begin(), cost_device_vec_for_sorting.end(), indices_device_vec.begin() );

		// input_array_device_vec 中からエリートサンプルのみをホストにコピー，input_array_host_vec_elite はソート済みの配列となる
        extract_elite_sample<<< CONST_PARAM::N_OF_THE_USING_BEST, 1 >>>( thrust::raw_pointer_cast( input_array_device_vec.data() ), thrust::raw_pointer_cast( input_array_device_vec_elite.data() ), thrust::raw_pointer_cast( indices_device_vec.data() ) );
        input_array_host_vec_elite = input_array_device_vec_elite;

		// コストを正規化するためのλを計算（expが0にならないための処理）
        float lambda = 0.0f;

        for ( int n = 0; n < CONST_PARAM::N_OF_THE_USING_BEST; n++ )
            lambda += input_array_host_vec_elite[n].cost;
        lambda /= (float)CONST_PARAM::N_OF_THE_USING_BEST;

		// 加重平均を計算
        float weight_temp;
        float sum_of_weight = 0.0f;

        for ( int i = 0; i < _DEVICE_CONST_HORIZON; i++ )
            for ( int j = 0; j < 4; j++ )
                best_input_array.decoupled_rps[i][j] = 0.0f;

        for ( int n = 0; n < CONST_PARAM::N_OF_THE_USING_BEST; n++ )
        {
            weight_temp = exp( -input_array_host_vec_elite[n].cost / lambda );
            sum_of_weight += weight_temp;

            for ( int i = 0; i < _DEVICE_CONST_HORIZON; i++ )
                for ( int j = 0; j < 4; j++ )
                    best_input_array.decoupled_rps[i][j] += input_array_host_vec_elite[n].decoupled_rps[i][j] * weight_temp;
        }

        for ( int i = 0; i < _DEVICE_CONST_HORIZON; i++ )
            for ( int j = 0; j < 4; j++ )
                best_input_array.decoupled_rps[i][j] /= sum_of_weight;

		//
        // 加重平均された入力列に対してコスト関数を再計算
        //
        best_input_array.cost = 0.0f;

        float var_and_z_i_temp[_N_OF_ODES + 1];
        for ( int i = 0; i < _N_OF_ODES + 1; i++ )
            var_and_z_i_temp[i] = var_and_z_i[i];

        float var_p_temp[_N_OF_ODES];

		// 衝突予測用変数
#ifdef PREDICTABLE_COLLISION_WITH_WALL
        bool  col_flag = false;
        float v_plus[3], w_plus[3];
        bool col_constraint_flag = false;
#endif
		for ( int i = 0; i < _DEVICE_CONST_HORIZON; i++ )
        {
			// PIDカスケード用修正
            float ref_th = fmaxf(0.0f, fminf(CONST_PARAM_FLOAT::MAX_THRUST, best_input_array.decoupled_rps[i][vz]));
            float ref_qx = best_input_array.decoupled_rps[i][vwx];
            float ref_qy = best_input_array.decoupled_rps[i][vwy];
            float ref_qz = best_input_array.decoupled_rps[i][vwz];

			for ( float t = 0.0f; t < CONST_PARAM_FLOAT::CONTROL_PERIOD - CONST_PARAM_FLOAT::INTEGRATION_STEP_SIZE / 2; t += CONST_PARAM_FLOAT::INTEGRATION_STEP_SIZE )
            {
#ifdef PREDICTABLE_COLLISION_WITH_WALL
                calculate_deviations_cpu( var_and_z_i_temp, var_p_temp, rps_cw1, rps_cw2, rps_ccw1, rps_ccw2, col_flag, v_plus, w_plus );
#else
                // calculate_deviations_cpu( var_and_z_i_temp, var_p_temp, rps_cw1, rps_cw2, rps_ccw1, rps_ccw2);
#endif
				float inv_mass = 1.0f / CONST_PARAM_FLOAT::MASS_OF_MACHINE;
                for ( int k = 0; k < _N_OF_ODES; k++ ) var_p_temp[k] = var_and_z_i_temp[k];
				/* e0p */ var_and_z_i_temp[0]   = sqrtf(fmaxf(0.0f, 1 - ref_qx*ref_qx - ref_qy*ref_qy - ref_qz*ref_qz));//TCB programから変更 fmaxf(1.0f, ...) からfmaxf(0.0f, ...) に変更
                /* e1p */ var_and_z_i_temp[1]   = ref_qx;
                /* e2p */ var_and_z_i_temp[2]   = ref_qy;
                /* e3p */ var_and_z_i_temp[3]   = ref_qz;

				float sign_w = copysignf(2.0f / CONST_PARAM_FLOAT::INTEGRATION_STEP_SIZE, var_p_temp[0]*var_and_z_i_temp[0] + var_p_temp[1]*ref_qx + var_p_temp[2]*ref_qy + var_p_temp[3]*ref_qz);
                /* wxp */ var_and_z_i_temp[4]   = sign_w *                  (-var_p_temp[1]*var_and_z_i_temp[0] + var_p_temp[0]*ref_qx + var_p_temp[3]*ref_qy - var_p_temp[2]*ref_qz);
                /* wyp */ var_and_z_i_temp[5]   = sign_w *                  (-var_p_temp[2]*var_and_z_i_temp[0] - var_p_temp[3]*ref_qx + var_p_temp[0]*ref_qy + var_p_temp[1]*ref_qz); 
                /* wzp */ var_and_z_i_temp[6]   = sign_w *                  (-var_p_temp[3]*var_and_z_i_temp[0] + var_p_temp[2]*ref_qx - var_p_temp[1]*ref_qy + var_p_temp[0]*ref_qz); 


				/* xp  */ var_and_z_i_temp[7]  +=  var_p_temp[10] * CONST_PARAM_FLOAT::INTEGRATION_STEP_SIZE;
                /* yp  */ var_and_z_i_temp[8]  +=  var_p_temp[11] * CONST_PARAM_FLOAT::INTEGRATION_STEP_SIZE;
                /* zp  */ var_and_z_i_temp[9]  +=  var_p_temp[12] * CONST_PARAM_FLOAT::INTEGRATION_STEP_SIZE;

				/* xpp */ var_and_z_i_temp[10] += -CONST_PARAM_FLOAT::INTEGRATION_STEP_SIZE * ref_th * inv_mass * (2.0f*var_p_temp[0]*var_p_temp[2] + 2.0f*var_p_temp[1]*var_p_temp[3]); 
                /* ypp */ var_and_z_i_temp[11] += -CONST_PARAM_FLOAT::INTEGRATION_STEP_SIZE * ref_th * inv_mass * (2.0f*var_p_temp[2]*var_p_temp[3] - 2.0f*var_p_temp[0]*var_p_temp[1]);
                /* zpp */ var_and_z_i_temp[12] += -CONST_PARAM_FLOAT::INTEGRATION_STEP_SIZE * ref_th * inv_mass * (2.0f*var_p_temp[0]*var_p_temp[0] + 2.0f*var_p_temp[3]*var_p_temp[3] - 1.0f) + CONST_PARAM_FLOAT::A_OF_GRAVITY;

                /* z_i */ var_and_z_i_temp[_N_OF_ODES] += var_and_z_i_temp[9] * CONST_PARAM_FLOAT::INTEGRATION_STEP_SIZE;
                
				// 衝突予測がONのとき，速度と角速度を上書きする
#ifdef PREDICTABLE_COLLISION_WITH_WALL
                if ( col_flag )
                {
                    col_constraint_flag = true;

                    var_and_z_i_temp[10] = v_plus[0];
                    var_and_z_i_temp[11] = v_plus[1];
                    var_and_z_i_temp[12] = v_plus[2];

                    // 角速度は機体座標系に変換してから代入
                    var_and_z_i_temp[4] = 2.0f*w_plus[1]*(var_and_z_i_temp[0]*var_and_z_i_temp[3]+var_and_z_i_temp[1]*var_and_z_i_temp[2]) + w_plus[0]*(-1.0f+2.0f*var_and_z_i_temp[0]*var_and_z_i_temp[0]+2.0f*var_and_z_i_temp[1]*var_and_z_i_temp[1]) - 2.0f*w_plus[2]*(var_and_z_i_temp[0]*var_and_z_i_temp[2]-var_and_z_i_temp[1]*var_and_z_i_temp[3]);
                    var_and_z_i_temp[5] = 2.0f*w_plus[2]*(var_and_z_i_temp[0]*var_and_z_i_temp[1]+var_and_z_i_temp[2]*var_and_z_i_temp[3]) + w_plus[1]*(-1.0f+2.0f*var_and_z_i_temp[0]*var_and_z_i_temp[0]+2.0f*var_and_z_i_temp[2]*var_and_z_i_temp[2]) - 2.0f*w_plus[0]*(var_and_z_i_temp[0]*var_and_z_i_temp[3]-var_and_z_i_temp[1]*var_and_z_i_temp[2]);
                    var_and_z_i_temp[6] = 2.0f*w_plus[0]*(var_and_z_i_temp[0]*var_and_z_i_temp[2]+var_and_z_i_temp[1]*var_and_z_i_temp[3]) + w_plus[2]*(-1.0f+2.0f*var_and_z_i_temp[0]*var_and_z_i_temp[0]+2.0f*var_and_z_i_temp[3]*var_and_z_i_temp[3]) - 2.0f*w_plus[1]*(var_and_z_i_temp[0]*var_and_z_i_temp[1]-var_and_z_i_temp[2]*var_and_z_i_temp[3]);
                }
#endif
			}
			// コストの計算（0.5かけるのはACADOに合わせるため）
            best_input_array.cost += 0.5f * (_COST_Q_X*(var_and_z_i_temp[7] -target_state_device.x )*(var_and_z_i_temp[7] -target_state_device.x) + _COST_Q_Y *(var_and_z_i_temp[8] -target_state_device.y) *(var_and_z_i_temp[8] -target_state_device.y) + _COST_Q_Z *(var_and_z_i_temp[9] -target_state_device.z) *(var_and_z_i_temp[9] -target_state_device.z)          // x, y, z
                						  + _COST_Q_XP*(var_and_z_i_temp[10]-target_state_device.xp)*(var_and_z_i_temp[10]-target_state_device.xp)+ _COST_Q_YP*(var_and_z_i_temp[11]-target_state_device.yp)*(var_and_z_i_temp[11]-target_state_device.yp)+ _COST_Q_ZP*(var_and_z_i_temp[12]-target_state_device.zp)*(var_and_z_i_temp[12]-target_state_device.zp)     // xp, yp, zp
										  + _COST_Q_E1*(var_and_z_i_temp[1] -target_state_device.e1)*(var_and_z_i_temp[1] -target_state_device.e1)+ _COST_Q_E2*(var_and_z_i_temp[2] -target_state_device.e2)*(var_and_z_i_temp[2] -target_state_device.e2)+ _COST_Q_E3*(var_and_z_i_temp[3] -target_state_device.e3)*(var_and_z_i_temp[3] -target_state_device.e3)         // e1, e2, e3
										  + _COST_Q_WX*(var_and_z_i_temp[4] -target_state_device.wx)*(var_and_z_i_temp[4] -target_state_device.wx)+ _COST_Q_WY*(var_and_z_i_temp[5] -target_state_device.wy)*(var_and_z_i_temp[5] -target_state_device.wy)+ _COST_Q_WZ*(var_and_z_i_temp[6] -target_state_device.wz)*(var_and_z_i_temp[6] -target_state_device.wz)          // wx, wy, wz
										  + _COST_Q_ZI*var_and_z_i_temp[_N_OF_ODES]*var_and_z_i_temp[_N_OF_ODES]                                                                                                  // z_i
										  + _COST_R_VZ*best_input_array.decoupled_rps[i][vz]*best_input_array.decoupled_rps[i][vz]
										  + _COST_R_VWX*best_input_array.decoupled_rps[i][vwx]*best_input_array.decoupled_rps[i][vwx]
										  + _COST_R_VWY*best_input_array.decoupled_rps[i][vwy]*best_input_array.decoupled_rps[i][vwy]
										  + _COST_R_VWZ*best_input_array.decoupled_rps[i][vwz]*best_input_array.decoupled_rps[i][vwz]
            );
		}
		return best_input_array.cost;
	}

	// 最初に get_instance() が呼ばれたときに唯一のインスタンスを生成し，以降はそれを保持（シングルトン）
    mcmpc_controller& mcmpc_controller::get_instance()
    {
        static mcmpc_controller instance;   // インスタンスの生成
        return instance;
    }

	float mcmpc_controller::calc_optimal_input(float var_and_z_i[], double &optimal_input1, double &optimal_input2, double &optimal_input3, double &optimal_input4 )
    {
		float min_cost = std::numeric_limits<float>::infinity();

		// 変数のコピー
        cudaMemcpyToSymbol( var_and_z_i_device, var_and_z_i, (_N_OF_ODES +1) * sizeof( float ) );

		// インプットシフト
        for ( int i = 0; i < _DEVICE_CONST_HORIZON - 1; i++ )
            for ( int j = 0; j < 4; j++ )
                best_input_array.decoupled_rps[i][j] = best_input_array.decoupled_rps[i + 1][j];

		// 準最適制御入力の計算
        for ( int k = 0; k < CONST_PARAM::ITERATION_TIMES; k++ )
        {
            cudaMemcpyToSymbol( average_input_device, &best_input_array, sizeof( input_array ) );
            
            generate_input_samples_and_calc_costs<<< _DEVICE_CONST_N_OF_BLOCK, _DEVICE_CONST_THREAD_PER_BLOCK >>>( curand_state_array, thrust::raw_pointer_cast( input_array_device_vec.data() ), thrust::raw_pointer_cast( cost_device_vec_for_sorting.data() ) );

            cudaDeviceSynchronize();

            min_cost = calc_weighted_average_and_min_cost( var_and_z_i );
        }

		// PIDカスケード用修正　入力：スラスト＋姿勢
        optimal_input1  = (double) (max(0.0f, min(CONST_PARAM::MAX_THRUST, best_input_array.decoupled_rps[0][vz])));
        optimal_input2  = (double)(best_input_array.decoupled_rps[0][vwx]);
        optimal_input3 = (double)(best_input_array.decoupled_rps[0][vwy]);
        optimal_input4 = (double)(best_input_array.decoupled_rps[0][vwz]);

		return min_cost;
	}

	// グラフ出力用に計算済みの準最適入力をコピー
	void mcmpc_controller::copy_best_input_array(input_array &dst)
	{
		for ( int i = 0; i < _DEVICE_CONST_HORIZON; i++ )
            for ( int j = 0; j < 4; j++ )
                dst.decoupled_rps[i][j] = best_input_array.decoupled_rps[i][j];
	}
}