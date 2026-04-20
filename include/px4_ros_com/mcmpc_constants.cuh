#pragma once
#include "const_params.hpp"
#include <cuda.h>
#include <curand.h>
#include <curand_kernel.h>

namespace qc_mcmpc {

    struct target_state_t
    {
        float e0;
        float e1;
        float e2;
        float e3;
        float wx;
        float wy;
        float wz;
        float x;
        float y;
        float z;
        float xp;
        float yp;
        float zp;
    };
    
      // 入力列
    class input_array
    {
    public:
        float decoupled_rps[_DEVICE_CONST_HORIZON][4];
        float cost;

        __device__ void generate_input_array(curandState &state);
        __device__ void do_simulation();
    };

    //  __constant__ メモリ上に展開する定数 (初期化時のみ変更)
    extern __constant__ target_state_t target_state_device;

    extern __constant__ float u_g_device;
    extern __constant__ float u_upper_lim_device;
    extern __constant__ float u_lower_lim_device;

    extern __constant__ float i_xx_device;
    extern __constant__ float i_yy_device;
    extern __constant__ float i_zz_device;
    extern __constant__ float rotor_distance_device;
    extern __constant__ float max_rps_pow_device;
    extern __constant__ float max_thrust_device;
    extern __constant__ float mass_of_machine_device;
    extern __constant__ float torque_rate_device;
    extern __constant__ float a_of_gravity_device;

#ifdef PREDICTABLE_COLLISION_WITH_WALL
    extern __constant__ float x_wall_device;
    extern __constant__ float wall_nv_x_device;
    extern __constant__ float wall_nv_y_device;
    extern __constant__ float wall_nv_z_device;
    extern __constant__ float r_of_ring_device;
    extern __constant__ float coeff_of_rest_device;
#endif

    extern __constant__ float control_period_device;
    extern __constant__ float integration_step_size_device;

    // __constant__ メモリ上に展開する変数（毎時刻更新）
    extern __constant__ float var_and_z_i_device[_N_OF_ODES + 1];
    extern __constant__ input_array average_input_device;
    extern __constant__ float sigma_k_device[4];
}