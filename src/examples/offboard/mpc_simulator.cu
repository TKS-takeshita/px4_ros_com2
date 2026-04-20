#include "px4_ros_com/mcmpc_controller.cuh"
#include "px4_ros_com/const_params.hpp"

#include <cmath>

#include <cuda.h>
#include <curand.h>
#include <curand_kernel.h>

constexpr float INV_SQRT_2 = 0.70710678118f;

namespace qc_mcmpc
{
    __device__ static void input_constraint( float& rps_cw1, float& rps_cw2, float& rps_ccw1, float& rps_ccw2 );

#ifdef PREDICTABLE_COLLISION_WITH_WALL
    __device__ static float dot_vec( float v1_x, float v1_y, float v1_z, float v2_x, float v2_y, float v2_z );
    __device__ static void cross_vec( float v1_x, float v1_y, float v1_z, float v2_x, float v2_y, float v2_z, float& v_ans_x, float& v_ans_y, float& v_ans_z );
    __device__ static void inverse_3x3( float matrix_src[3][3], float ans[3][3] );

    __device__ static void calculate_deviations( float var_and_z_i[], float var_p[], float rps_cw1, float rps_cw2, float rps_ccw1, float rps_ccw2, bool& col_flag, float v_plus[], float w_plus[] );
#else
    __device__ static void calculate_deviations( float var_and_z_i[], float var_p[], float rps_cw1, float rps_cw2, float rps_ccw1, float rps_ccw2 );
#endif

    __device__ static void input_constraint( float& rps_cw1, float& rps_cw2, float& rps_ccw1, float& rps_ccw2 )
    {
        // rps_cw1
        if ( rps_cw1 > u_upper_lim_device )   rps_cw1 = u_upper_lim_device;
        if ( rps_cw1 < u_lower_lim_device )   rps_cw1 = u_lower_lim_device;

        // rps_cw2
        if ( rps_cw2 > u_upper_lim_device )   rps_cw2 = u_upper_lim_device;
        if ( rps_cw2 < u_lower_lim_device )   rps_cw2 = u_lower_lim_device;

        // rps_ccw1
        if ( rps_ccw1 > u_upper_lim_device )   rps_ccw1 = u_upper_lim_device;
        if ( rps_ccw1 < u_lower_lim_device )   rps_ccw1 = u_lower_lim_device;

        // rps_ccw2
        if ( rps_ccw2 > u_upper_lim_device )   rps_ccw2 = u_upper_lim_device;
        if ( rps_ccw2 < u_lower_lim_device )   rps_ccw2 = u_lower_lim_device;
    }

#ifdef PREDICTABLE_COLLISION_WITH_WALL
    // 衝突予測あり
    __device__ static float dot_vec( float v1_x, float v1_y, float v1_z, float v2_x, float v2_y, float v2_z )
    {
        return v1_x*v2_x + v1_y*v2_y + v1_z*v2_z;
    }

    __device__ static void cross_vec( float v1_x, float v1_y, float v1_z, float v2_x, float v2_y, float v2_z, float& v_ans_x, float& v_ans_y, float& v_ans_z )
    {
        v_ans_x = v1_y*v2_z - v1_z*v2_y;
        v_ans_y = v1_z*v2_x - v1_x*v2_z;
        v_ans_z = v1_x*v2_y - v1_y*v2_x;
    }

    __device__ static void inverse_3x3( float matrix_src[3][3], float ans[3][3] )
    {
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

    __device__ static void calculate_deviations( float var_and_z_i[], float var_p[], float rps_cw1, float rps_cw2, float rps_ccw1, float rps_ccw2, bool& col_flag, float v_plus[], float w_plus[] )
    {
        // === 衝突判定 & 衝突後速度/角速度計算 ===

        // ガードリングの法線ベクトル
        float ring_nv_x = 2.0f*var_and_z_i[0]*var_and_z_i[2] + 2.0f*var_and_z_i[1]*var_and_z_i[3];
        float ring_nv_y = 2.0f*var_and_z_i[2]*var_and_z_i[3] - 2.0f*var_and_z_i[0]*var_and_z_i[1];
        float ring_nv_z = -1.0f + 2.0f*var_and_z_i[0]*var_and_z_i[0] + 2.0f*var_and_z_i[3]*var_and_z_i[3];

        // 衝突の判別式，リングと壁の共有点が存在するかを判別している
        float discriminant = ( 2.0f*ring_nv_x*ring_nv_y*(x_wall_device-var_and_z_i[7]) - 2.0f*(ring_nv_y*ring_nv_y+ring_nv_z*ring_nv_z)*var_and_z_i[8] )*( 2.0f*ring_nv_x*ring_nv_y*(x_wall_device-var_and_z_i[7]) - 2.0f*(ring_nv_y*ring_nv_y+ring_nv_z*ring_nv_z)*var_and_z_i[8] )
                            - 4*(ring_nv_y*ring_nv_y+ring_nv_z*ring_nv_z)*( (ring_nv_x*ring_nv_x+ring_nv_z*ring_nv_z)*(x_wall_device-var_and_z_i[7])*(x_wall_device-var_and_z_i[7]) - 2.0f*ring_nv_x*ring_nv_y*(x_wall_device-var_and_z_i[7])*var_and_z_i[8] + (ring_nv_y*ring_nv_y+ring_nv_z*ring_nv_z)*var_and_z_i[8]*var_and_z_i[8] - (ring_nv_z*r_of_ring_device)*(ring_nv_z*r_of_ring_device) );

        // col_flag は撃力の2重印加防止と，コントローラへ衝突情報を伝えるためのもの
        if ( discriminant >= 0.0f && !col_flag )
        {
            // 衝突前の速度
            float v_minus[3];
            v_minus[0] = var_and_z_i[10];
            v_minus[1] = var_and_z_i[11];
            v_minus[2] = var_and_z_i[12];

            // 衝突前の角速度（世界座標系に変換）
            float w_minus[3];
            w_minus[0] = 2.0f*var_and_z_i[6]*(var_and_z_i[0]*var_and_z_i[2]+var_and_z_i[1]*var_and_z_i[3]) + var_and_z_i[4]*(-1.0f+2.0f*var_and_z_i[0]*var_and_z_i[0]+2.0f*var_and_z_i[1]*var_and_z_i[1]) - 2.0f*var_and_z_i[5]*(var_and_z_i[0]*var_and_z_i[3]-var_and_z_i[1]*var_and_z_i[2]);
            w_minus[1] = 2.0f*var_and_z_i[4]*(var_and_z_i[0]*var_and_z_i[3]+var_and_z_i[1]*var_and_z_i[2]) + var_and_z_i[5]*(-1.0f+2.0f*var_and_z_i[0]*var_and_z_i[0]+2.0f*var_and_z_i[2]*var_and_z_i[2]) - 2.0f*var_and_z_i[6]*(var_and_z_i[0]*var_and_z_i[1]-var_and_z_i[2]*var_and_z_i[3]);
            w_minus[2] = 2.0f*var_and_z_i[5]*(var_and_z_i[0]*var_and_z_i[1]+var_and_z_i[2]*var_and_z_i[3]) + var_and_z_i[6]*(-1.0f+2.0f*var_and_z_i[0]*var_and_z_i[0]+2.0f*var_and_z_i[3]*var_and_z_i[3]) - 2.0f*var_and_z_i[4]*(var_and_z_i[0]*var_and_z_i[2]-var_and_z_i[1]*var_and_z_i[3]);

            // 機体の質量中心から衝突点までのベクトル
            float r_vec[3];
            r_vec[0] = x_wall_device - var_and_z_i[7];
            r_vec[1] = -( ring_nv_x*ring_nv_y*(x_wall_device-var_and_z_i[7]) - (ring_nv_y*ring_nv_y+ring_nv_z*ring_nv_z)*var_and_z_i[8] ) / ( ring_nv_y*ring_nv_y+ring_nv_z*ring_nv_z ) - var_and_z_i[8];
            r_vec[2] = -( ring_nv_x*r_vec[0] + ring_nv_y*r_vec[1] )/ring_nv_z;

            // 機体の慣性テンソル（世界座標系に変化位）
            float i_tensor[3][3];
            i_tensor[0][0] = 4.0f*i_zz_device*(var_and_z_i[0]*var_and_z_i[2]+var_and_z_i[1]*var_and_z_i[3])*(var_and_z_i[0]*var_and_z_i[2]+var_and_z_i[1]*var_and_z_i[3])+4.0f*i_yy_device*(var_and_z_i[0]*var_and_z_i[3]-var_and_z_i[1]*var_and_z_i[2])*(var_and_z_i[0]*var_and_z_i[3]-var_and_z_i[1]*var_and_z_i[2])+i_xx_device*(-1.0f+2.0f*var_and_z_i[0]*var_and_z_i[0]+2.0f*var_and_z_i[1]*var_and_z_i[1])*(-1.0f+2.0f*var_and_z_i[0]*var_and_z_i[0]+2.0f*var_and_z_i[1]*var_and_z_i[1]);
            i_tensor[0][1] = 2.0f*i_xx_device*(var_and_z_i[0]*var_and_z_i[3]+var_and_z_i[1]*var_and_z_i[2])*(-1.0f+2.0f*var_and_z_i[0]*var_and_z_i[0]+2.0f*var_and_z_i[1]*var_and_z_i[1])-4.0f*i_zz_device*(var_and_z_i[0]*var_and_z_i[2]+var_and_z_i[1]*var_and_z_i[3])*(var_and_z_i[0]*var_and_z_i[1]-var_and_z_i[2]*var_and_z_i[3])-2.0f*i_yy_device*(var_and_z_i[0]*var_and_z_i[3]-var_and_z_i[1]*var_and_z_i[2])*(-1.0f+2.0f*var_and_z_i[0]*var_and_z_i[0]+2.0f*var_and_z_i[2]*var_and_z_i[2]);
            i_tensor[0][2] = 2.0f*i_zz_device*(var_and_z_i[0]*var_and_z_i[2]+var_and_z_i[1]*var_and_z_i[3])*(-1.0f+2.0f*var_and_z_i[0]*var_and_z_i[0]+2.0f*var_and_z_i[3]*var_and_z_i[3])-4.0f*i_yy_device*(var_and_z_i[0]*var_and_z_i[1]+var_and_z_i[2]*var_and_z_i[3])*(var_and_z_i[0]*var_and_z_i[3]-var_and_z_i[1]*var_and_z_i[2])-2.0f*i_xx_device*(var_and_z_i[0]*var_and_z_i[2]-var_and_z_i[1]*var_and_z_i[3])*(-1.0f+2.0f*var_and_z_i[0]*var_and_z_i[0]+2.0f*var_and_z_i[1]*var_and_z_i[1]);
            i_tensor[1][0] = i_tensor[0][1];
            i_tensor[1][1] = 4.0f*i_xx_device*(var_and_z_i[0]*var_and_z_i[3]+var_and_z_i[1]*var_and_z_i[2])*(var_and_z_i[0]*var_and_z_i[3]+var_and_z_i[1]*var_and_z_i[2])+4.0f*i_zz_device*(var_and_z_i[0]*var_and_z_i[1]-var_and_z_i[2]*var_and_z_i[3])*(var_and_z_i[0]*var_and_z_i[1]-var_and_z_i[2]*var_and_z_i[3])+i_yy_device*(-1.0f+2.0f*var_and_z_i[0]*var_and_z_i[0]+2.0f*var_and_z_i[2]*var_and_z_i[2])*(-1.0f+2.0f*var_and_z_i[0]*var_and_z_i[0]+2.0f*var_and_z_i[2]*var_and_z_i[2]);
            i_tensor[1][2] = 2.0f*i_yy_device*(var_and_z_i[0]*var_and_z_i[1]+var_and_z_i[2]*var_and_z_i[3])*(-1.0f+2.0f*var_and_z_i[0]*var_and_z_i[0]+2.0f*var_and_z_i[2]*var_and_z_i[2])-4.0f*i_xx_device*(var_and_z_i[0]*var_and_z_i[3]+var_and_z_i[1]*var_and_z_i[2])*(var_and_z_i[0]*var_and_z_i[2]-var_and_z_i[1]*var_and_z_i[3])-2.0f*i_zz_device*(var_and_z_i[0]*var_and_z_i[1]-var_and_z_i[2]*var_and_z_i[3])*(-1.0f+2.0f*var_and_z_i[0]*var_and_z_i[0]+2.0f*var_and_z_i[3]*var_and_z_i[3]);
            i_tensor[2][0] = i_tensor[0][2];
            i_tensor[2][1] = i_tensor[1][2];
            i_tensor[2][2] = 4.0f*i_yy_device*(var_and_z_i[0]*var_and_z_i[1]+var_and_z_i[2]*var_and_z_i[3])*(var_and_z_i[0]*var_and_z_i[1]+var_and_z_i[2]*var_and_z_i[3])+4.0f*i_xx_device*(var_and_z_i[0]*var_and_z_i[2]-var_and_z_i[1]*var_and_z_i[3])*(var_and_z_i[0]*var_and_z_i[2]-var_and_z_i[1]*var_and_z_i[3])+i_zz_device*(-1.0f+2.0f*var_and_z_i[0]*var_and_z_i[0]+2.0f*var_and_z_i[3]*var_and_z_i[3])*(-1.0f+2.0f*var_and_z_i[0]*var_and_z_i[0]+2.0f*var_and_z_i[3]*var_and_z_i[3]);

            float i_inverse[3][3];
            inverse_3x3( i_tensor, i_inverse );

            float vec_temp1[3];
            cross_vec( w_minus[0], w_minus[1], w_minus[2], r_vec[0], r_vec[1], r_vec[2], vec_temp1[0], vec_temp1[1], vec_temp1[2] );
            for ( int i = 0; i < 3; i++ )
                vec_temp1[i] = v_minus[i] + vec_temp1[i];

            float vec_temp2[3];
            cross_vec( r_vec[0], r_vec[1], r_vec[2], wall_nv_x_device, wall_nv_y_device, wall_nv_z_device, vec_temp2[0], vec_temp2[1], vec_temp2[2] );
            for ( int i = 0; i < 3; i++ )
                vec_temp2[i] = i_inverse[i][0]*vec_temp2[0] + i_inverse[i][1]*vec_temp2[1] + i_inverse[i][2]*vec_temp2[2];

            float vec_temp3[3];
            cross_vec( vec_temp2[0], vec_temp2[1], vec_temp2[2], r_vec[0], r_vec[1], r_vec[2], vec_temp3[0], vec_temp3[1], vec_temp3[2] );

            // 機体が壁から受ける力積の大きさ（向きは wall_nv で指定。判別式の都合上，実質 y-z 平面に平行な壁しか作れない）
            float col_ft = -( 1 + coeff_of_rest_device ) * dot_vec( vec_temp1[0], vec_temp1[1], vec_temp1[2], wall_nv_x_device, wall_nv_y_device, wall_nv_z_device )
                        / ( 1/mass_of_machine_device + dot_vec( vec_temp3[0], vec_temp3[1], vec_temp3[2], wall_nv_x_device, wall_nv_y_device, wall_nv_z_device ) );

            // 衝突後の速度
            v_plus[0] = (col_ft/mass_of_machine_device) * wall_nv_x_device + v_minus[0];
            v_plus[1] = (col_ft/mass_of_machine_device) * wall_nv_y_device + v_minus[1];
            v_plus[2] = (col_ft/mass_of_machine_device) * wall_nv_z_device + v_minus[2];

            float vec_temp4[3];
            cross_vec( r_vec[0], r_vec[1], r_vec[2], col_ft*wall_nv_x_device, col_ft*wall_nv_y_device, col_ft*wall_nv_z_device, vec_temp4[0], vec_temp4[1], vec_temp4[2] );

            // 衝突後の角速度（世界座標系）
            w_plus[0] = i_inverse[0][0]*vec_temp4[0] + i_inverse[0][1]*vec_temp4[1] + i_inverse[0][2]*vec_temp4[2] + w_minus[0];
            w_plus[1] = i_inverse[1][0]*vec_temp4[0] + i_inverse[1][1]*vec_temp4[1] + i_inverse[1][2]*vec_temp4[2] + w_minus[1];
            w_plus[2] = i_inverse[2][0]*vec_temp4[0] + i_inverse[2][1]*vec_temp4[1] + i_inverse[2][2]*vec_temp4[2] + w_minus[2];

            col_flag = true;
        }
        else if ( discriminant < 0.0f )
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
    __device__ static void calculate_deviations( float var_and_z_i[], float var_p[], float rps_cw1, float rps_cw2, float rps_ccw1, float rps_ccw2 )
    {
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
#endif
    __device__ void input_array::generate_input_array(curandState &state)
    {
#ifdef USE_INPUT_INTERPOLATION
        // 補間を利用した入力列の生成
        int t_randp[_DEVICE_CONST_N_OF_RANDP];
        for ( int i = 0; i < _DEVICE_CONST_N_OF_RANDP; i++ )
        {
            t_randp[i] = (_DEVICE_CONST_HORIZON / (_DEVICE_CONST_N_OF_RANDP - 1)) * i;    // n_of_randp = 6, horizon 150 -> t_randp[i] = 0, 30, 60, 90, 120, 149
            if ( t_randp[i] == _DEVICE_CONST_HORIZON )
                t_randp[i]--;

            // 代表点のみ，乱数で生成
            for ( int j = 0; j < 4; j++ )
                decoupled_rps[t_randp[i]][j] = average_input_device.decoupled_rps[t_randp[i]][j] + curand_normal( &state ) * sigma_k_device[j];
        }

        // 他の点をラグランジュ補間で生成
        int randp_ctr = 0;
        for ( int i = 1; i < _DEVICE_CONST_HORIZON; i++ )
        {
            // 乱数で生成済みの時刻はスキップ
            bool continue_flag = false;
            for ( int j = 0; j < _DEVICE_CONST_N_OF_RANDP; j++ )
                if ( i == t_randp[j] )
                    continue_flag = true;

            if ( continue_flag )
            {
                if ( randp_ctr + _DEVICE_CONST_ORDER_OF_INTERPOLATION + 1 < _DEVICE_CONST_N_OF_RANDP )
                    randp_ctr++;
                continue;
            }

            // ラグランジュ補間
            for ( int j = 0; j < 4; j++ )
            {
                float v_temp = 0.0f;

                for ( int i1 = 0; i1 < _DEVICE_CONST_ORDER_OF_INTERPOLATION + 1; i1++ )
                {
                    float ln = 1.0f, ld = 1.0f;
                    for ( int i2 = 0; i2 < _DEVICE_CONST_ORDER_OF_INTERPOLATION + 1; i2++ )
                    {
                        if ( i2 != i1 )
                        {
                            ln *= (float)(i - t_randp[randp_ctr + i2]);
                            ld *= (float)(t_randp[randp_ctr + i1] - t_randp[randp_ctr + i2]);
                        }
                    }
                    v_temp += decoupled_rps[t_randp[randp_ctr + i1]][j] * ln / ld;
                }
                decoupled_rps[i][j] = v_temp;
            }
        }
#else
        // 入力列の生成（補間による次元削減なし）
        for ( int i = 0; i < _DEVICE_CONST_HORIZON; i++ )
            for ( int j = 0; j < 4; j++ )
                decoupled_rps[i][j] = average_input_device.decoupled_rps[i][j] + curand_normal( &state ) * sigma_k_device[j];
#endif
    }

    __device__ void input_array::do_simulation()
    {
        cost = 0.0f;

        // __constant__ メモリから状態量をコピー
        float var_and_z_i_temp[_N_OF_ODES + 1];
        for ( int i = 0; i < _N_OF_ODES + 1; i++ )
            var_and_z_i_temp[i] = var_and_z_i_device[i];
        
        float var_p_temp[_N_OF_ODES];

        // 衝突予測用変数
#ifdef PREDICTABLE_COLLISION_WITH_WALL
        bool  col_flag = false;
        float v_plus[3], w_plus[3];
        bool col_constraint_flag = false;
#endif

        // オイラー積分とコスト評価
        for ( int i = 0; i < _DEVICE_CONST_HORIZON; i++ )
        {
            // PIDカスケード用修正
            float ref_th = fmaxf(0.0f, fminf(max_thrust_device, decoupled_rps[i][vz]));
            float ref_qx = fmaxf(-1.0f, fminf(1.0f, decoupled_rps[i][vwx]));
            float ref_qy = fmaxf(-1.0f, fminf(1.0f, decoupled_rps[i][vwy]));
            float ref_qz = fmaxf(-1.0f, fminf(1.0f, decoupled_rps[i][vwz]));
            decoupled_rps[i][vz]  = ref_th;
            decoupled_rps[i][vwx] = ref_qx;
            decoupled_rps[i][vwy] = ref_qy;
            decoupled_rps[i][vwz] = ref_qz;
            
            for ( float t = 0.0f; t < control_period_device - integration_step_size_device / 2; t += integration_step_size_device )
            {
#ifdef PREDICTABLE_COLLISION_WITH_WALL
    calculate_deviations(var_and_z_i_temp, var_p_temp,
                         rps_cw1, rps_cw2, rps_ccw1, rps_ccw2,
                         col_flag, v_plus, w_plus);
#else
    // calculate_deviations(var_and_z_i_temp, var_p_temp,rps_cw1, rps_cw2, rps_ccw1, rps_ccw2);
#endif
                // PIDカスケード用修正
                float inv_mass = 1.0f / mass_of_machine_device;
                for ( int k = 0; k < _N_OF_ODES; k++ ) var_p_temp[k] = var_and_z_i_temp[k];
                /* e0p */ var_and_z_i_temp[0]   = sqrtf(fmaxf(0.0f, 1 - ref_qx*ref_qx - ref_qy*ref_qy - ref_qz*ref_qz));
                /* e1p */ var_and_z_i_temp[1]   = ref_qx;
                /* e2p */ var_and_z_i_temp[2]   = ref_qy;
                /* e3p */ var_and_z_i_temp[3]   = ref_qz;

                float sign_w = copysignf(2.0f / integration_step_size_device, var_p_temp[0]*var_and_z_i_temp[0] + var_p_temp[1]*ref_qx + var_p_temp[2]*ref_qy + var_p_temp[3]*ref_qz);
                /* wxp */ var_and_z_i_temp[4]   = sign_w *                  (-var_p_temp[1]*var_and_z_i_temp[0] + var_p_temp[0]*ref_qx + var_p_temp[3]*ref_qy - var_p_temp[2]*ref_qz);
                /* wyp */ var_and_z_i_temp[5]   = sign_w *                  (-var_p_temp[2]*var_and_z_i_temp[0] - var_p_temp[3]*ref_qx + var_p_temp[0]*ref_qy + var_p_temp[1]*ref_qz); 
                /* wzp */ var_and_z_i_temp[6]   = sign_w *                  (-var_p_temp[3]*var_and_z_i_temp[0] + var_p_temp[2]*ref_qx - var_p_temp[1]*ref_qy + var_p_temp[0]*ref_qz); 

                /* xp  */ var_and_z_i_temp[7]  +=  var_p_temp[10] * integration_step_size_device;
                /* yp  */ var_and_z_i_temp[8]  +=  var_p_temp[11] * integration_step_size_device;
                /* zp  */ var_and_z_i_temp[9]  +=  var_p_temp[12] * integration_step_size_device;

                /* xpp */ var_and_z_i_temp[10] += integration_step_size_device * (-ref_th) * inv_mass * (2.0f*var_p_temp[0]*var_p_temp[2] + 2.0f*var_p_temp[1]*var_p_temp[3]); 
                /* ypp */ var_and_z_i_temp[11] += integration_step_size_device * (-ref_th) * inv_mass * (2.0f*var_p_temp[2]*var_p_temp[3] - 2.0f*var_p_temp[0]*var_p_temp[1]);
                /* zpp */ var_and_z_i_temp[12] += integration_step_size_device *((-ref_th) * inv_mass * (2.0f*var_p_temp[0]*var_p_temp[0] + 2.0f*var_p_temp[3]*var_p_temp[3] - 1.0f) + a_of_gravity_device);

                /* z_i */ var_and_z_i_temp[_N_OF_ODES] += var_and_z_i_temp[9] * integration_step_size_device;

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
            // コストの計算
            cost += (_COST_Q_X*(var_and_z_i_temp[7] -target_state_device.x )*(var_and_z_i_temp[7] -target_state_device.x) + _COST_Q_Y *(var_and_z_i_temp[8] -target_state_device.y) *(var_and_z_i_temp[8] -target_state_device.y) + _COST_Q_Z *(var_and_z_i_temp[9] -target_state_device.z) *(var_and_z_i_temp[9] -target_state_device.z)     // x, y, z
                 +  _COST_Q_XP*(var_and_z_i_temp[10]-target_state_device.xp)*(var_and_z_i_temp[10]-target_state_device.xp)+ _COST_Q_YP*(var_and_z_i_temp[11]-target_state_device.yp)*(var_and_z_i_temp[11]-target_state_device.yp)+ _COST_Q_ZP*(var_and_z_i_temp[12]-target_state_device.zp)*(var_and_z_i_temp[12]-target_state_device.zp)    // xp, yp, zp
                 +  _COST_Q_E1*(var_and_z_i_temp[1] -target_state_device.e1)*(var_and_z_i_temp[1] -target_state_device.e1)+ _COST_Q_E2*(var_and_z_i_temp[2] -target_state_device.e2)*(var_and_z_i_temp[2] -target_state_device.e2)+ _COST_Q_E3*(var_and_z_i_temp[3] -target_state_device.e3)*(var_and_z_i_temp[3] -target_state_device.e3)         // e1, e2, e3
                 +  _COST_Q_WX*(var_and_z_i_temp[4] -target_state_device.wx)*(var_and_z_i_temp[4] -target_state_device.wx)+ _COST_Q_WY*(var_and_z_i_temp[5] -target_state_device.wy)*(var_and_z_i_temp[5] -target_state_device.wy)+ _COST_Q_WZ*(var_and_z_i_temp[6] -target_state_device.wz)*(var_and_z_i_temp[6] -target_state_device.wz)          // wx, wy, wz
                 +  _COST_Q_ZI*var_and_z_i_temp[_N_OF_ODES]*var_and_z_i_temp[_N_OF_ODES]                                                                                                  // z_i
                 +  _COST_R_VZ*decoupled_rps[i][vz]*decoupled_rps[i][vz]
                 +  _COST_R_VWX*decoupled_rps[i][vwx]*decoupled_rps[i][vwx]
                 +  _COST_R_VWY*decoupled_rps[i][vwy]*decoupled_rps[i][vwy]
                 +  _COST_R_VWZ*decoupled_rps[i][vwz]*decoupled_rps[i][vwz]
            );
        }
        // 衝突に対して制約を与えたい場合はここに記述
#ifdef PREDICTABLE_COLLISION_WITH_WALL
        // if ( col_constraint_flag )  cost += 5;
#endif
    }
}