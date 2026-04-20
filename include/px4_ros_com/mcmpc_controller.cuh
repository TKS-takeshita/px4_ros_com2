#pragma once 
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include "px4_ros_com/mcmpc_constants.cuh"

enum decoupled_rotor
{
    vz,
    vwx,
    vwy,
    vwz
};

namespace qc_mcmpc
{
    // 目標状態の更新用関数
	    void update_target_state_device(const target_state_t& target);
        
    // シングルトンとして実装
    class mcmpc_controller
    {
    private:
        mcmpc_controller();
        ~mcmpc_controller();

        float calc_weighted_average_and_min_cost(float var_and_z_i[]);

        curandState *curand_state_array;

        input_array best_input_array;
        thrust::device_vector<input_array> input_array_device_vec;

        // sort
        thrust::device_vector<int> indices_device_vec;
        thrust::device_vector<float> cost_device_vec_for_sorting;

        thrust::device_vector<input_array> input_array_device_vec_elite;
        thrust::host_vector<input_array> input_array_host_vec_elite;

        float sigma_k[4];

    public:
        // シングルトン実装のため、コピーコンストラクタ, コピー代入演算子, ムーブコンストラクタ, ムーブ代入演算子の使用を禁止
        mcmpc_controller(const mcmpc_controller &) = delete;
        mcmpc_controller & operator=(const mcmpc_controller &) = delete;
        mcmpc_controller(mcmpc_controller &&) = delete;
        mcmpc_controller & operator=(mcmpc_controller &&) = delete;

        // インスタンス生成
        static mcmpc_controller & get_instance();
        
        // 準最適入力を計算し, 最小コストを返す
        float calc_optimal_input(float var_and_z_i[], double &optimal_rps_cw1, double &optimal_rps_cw2, double &optimal_rps_ccw1, double &optimal_rps_ccw2);
        
        // グラフ出力ように, 計算済み順最適入力をコピー
        void copy_best_input_array(input_array &dst);
    };
}