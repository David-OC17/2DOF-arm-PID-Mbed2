#ifndef __KF_2DOF_H__
#define __KF_2DOF_H__

#include <arm_math.h>
#include "common.h"

static void print_matrix_f32(const arm_matrix_instance_f32 *mat, const char *name);

// Predict state estimate (x_check_k)
// x_check_k = F * x_hat_prev + B * v_prev
// Where:
// x_hat_prev is the previous state estimate: [theta, dot_theta]^T
// F is the state transition matrix:
//     [ 1   T  ]
//     [ 0  1-b/J*T ]
// B is the control input matrix:
//     [ 0 ]
//     [ K/J*T ]
// v_prev is the previous control input (voltage)

// Predict error covariance (P_check_k)
// P_check_k = F * P_hat_prev * F^T + Q
// Where:
// P_hat_prev is the previous error covariance matrix:
//     [ p1  p2 ]
//     [ p3  p4 ]
// Q is the process noise covariance matrix:
//     [ q_theta       0 ]
//     [     0   q_dot_theta ]

// Compute Kalman Gain (K_k)
// K_k = P_check_k * H^T * (H * P_check_k * H^T + 0.01)^(-1)
// Where:
// H is the observation matrix: [1  0]
// H^T is the transposed observation matrix: [1]
// 0.01 represents the measurement noise covariance

// Update state estimate (x_hat_k)
// x_hat_k = x_check_k + K_k * (Z_k - H * x_check_k)
// Where:
// Z_k is the measurement: [theta_measured]

// Update error covariance (P_hat_k)
// P_hat_k = (I - K_k * H) * P_check_k
// Where:
// I is the identity matrix:
//     [ 1  0 ]
//     [ 0  1 ]

static const float T = 0.1;
static const float b = 1;
static const float J = 1;
static const float K = 1;
static const float q_theta = 0.01;
static const float q_theta_dot = 0.01;
static const float R = 0.1;

typedef struct {
  arm_matrix_instance_f32 x_hat_prev;
  arm_matrix_instance_f32 P_hat_prev;
  arm_matrix_instance_f32 F;
  arm_matrix_instance_f32 B;
  arm_matrix_instance_f32 Q;
  arm_matrix_instance_f32 H;
  arm_matrix_instance_f32 H_transposed;
  arm_matrix_instance_f32 I;

  float32_t x_hat_prev_data[2];
  float32_t P_hat_prev_data[4];
  float32_t F_data[4], F_transposed_data[4];
  float32_t B_data[2];
  float32_t Q_data[4];
  float32_t H_data[2];
  float32_t H_transposed_data[2];
  float32_t I_data[4];
} KF_2DOF_t;

static void KF_Init(KF_2DOF_t *kf) {
  // clang-format off
  kf->x_hat_prev_data[0] = 0;
  kf->x_hat_prev_data[1] = 0;

  kf->P_hat_prev_data[0] = 1; kf->P_hat_prev_data[1] = 0;
  kf->P_hat_prev_data[2] = 0; kf->P_hat_prev_data[3] = 1;

  arm_mat_init_f32(&kf->x_hat_prev, 2, 1, kf->x_hat_prev_data);
  arm_mat_init_f32(&kf->P_hat_prev, 2, 2, kf->P_hat_prev_data);

  kf->F_data[0] = 1; kf->F_data[1] = T;
  kf->F_data[2] = 0; kf->F_data[3] = 1 - (b / J) * T;
  arm_mat_init_f32(&kf->F, 2, 2, kf->F_data);

  kf->B_data[0] = 0; kf->B_data[1] = (K / J) * T;
  arm_mat_init_f32(&kf->B, 2, 1, kf->B_data);

  kf->Q_data[0] = q_theta; kf->Q_data[1] = 0;
  kf->Q_data[2] = 0; kf->Q_data[3] = q_theta_dot;
  arm_mat_init_f32(&kf->Q, 2, 2, kf->Q_data);

  kf->H_data[0] = 1; kf->H_data[1] = 0;
  arm_mat_init_f32(&kf->H, 1, 2, kf->H_data);

  kf->H_transposed_data[0] = 1; kf->H_transposed_data[1] = 0;
  arm_mat_init_f32(&kf->H_transposed, 2, 1, kf->H_transposed_data);

  kf->I_data[0] = 1; kf->I_data[1] = 0;
  kf->I_data[2] = 0; kf->I_data[3] = 1;
  arm_mat_init_f32(&kf->I, 2, 2, kf->I_data);
  // clang-format on
}

static void KF_Update(KF_2DOF_t *kf, float32_t u_prev, float32_t Z_k) {
  float32_t x_check_data[2], P_check_data[4];
  float32_t temp1_data[4], temp2_data[4], control_data[2];
  float32_t temp_H_P_data[2], HPH_data[1], numerator_data[2];
  float32_t K_gain_data[2], K_times_residual_data[2], KH_data[4],
      I_minus_KH_data[4], temp_H_X_data[1];

  arm_matrix_instance_f32 x_check, P_check, temp1, temp2, control;
  arm_matrix_instance_f32 temp_H_P, temp_H_X, HPH, numerator, K_gain, K_times_residual;
  arm_matrix_instance_f32 KH, I_minus_KH, x_hat, P_hat;

  arm_mat_init_f32(&x_check, 2, 1, x_check_data);
  arm_mat_init_f32(&P_check, 2, 2, P_check_data);
  arm_mat_init_f32(&temp1, 2, 2, temp1_data);
  arm_mat_init_f32(&temp2, 2, 2, temp2_data);
  arm_mat_init_f32(&control, 2, 1, control_data);
  arm_mat_init_f32(&temp_H_P, 1, 2, temp_H_P_data);
  arm_mat_init_f32(&temp_H_X, 1, 1, temp_H_X_data);
  arm_mat_init_f32(&HPH, 1, 1, HPH_data);
  arm_mat_init_f32(&numerator, 2, 1, numerator_data);
  arm_mat_init_f32(&K_gain, 2, 1, K_gain_data);
  arm_mat_init_f32(&K_times_residual, 2, 1, K_times_residual_data);
  arm_mat_init_f32(&KH, 2, 2, KH_data);
  arm_mat_init_f32(&I_minus_KH, 2, 2, I_minus_KH_data);
  arm_mat_init_f32(&x_hat, 2, 1, kf->x_hat_prev_data);
  arm_mat_init_f32(&P_hat, 2, 2, kf->P_hat_prev_data);

  // clang-format off
  // x_check_k = F * x_hat_prev + B * v_prev
  arm_mat_mult_f32(&kf->F, &kf->x_hat_prev, &x_check); // F * x_hat_prev
  arm_mat_scale_f32(&kf->B, u_prev, &control);         // B * v_prev
  arm_mat_add_f32(&x_check, &control, &x_check);       // F * x_hat_prev + B * v_prev

  // P_check_k = F * P_hat_prev * F^T + Q
  arm_mat_mult_f32(&kf->F, &kf->P_hat_prev, &temp1); // F * P_hat_prev
  arm_mat_trans_f32(&kf->F, &temp2);                 // F^T
  arm_mat_mult_f32(&temp1, &temp2, &P_check);        // F * P_hat_prev * F^T
  arm_mat_add_f32(&P_check, &kf->Q, &P_check);       // F * P_hat_prev * F^T + Q

  print_matrix_f32(&P_check, "p_check");

  // K_k = P_check_k * H^T * (H * P_check_k * H^T + 0.01)^(-1)
  arm_mat_mult_f32(&P_check, &kf->H_transposed, &numerator); // P_check_k * H^T
  arm_mat_mult_f32(&kf->H, &P_check, &temp_H_P);             // H * P_check_k
  arm_mat_mult_f32(&temp_H_P, &kf->H_transposed, &HPH);      // H * P_check_k * H^T

  print_matrix_f32(&kf->H, "Kalman");

  print_matrix_f32(&HPH, "HPH");

  float32_t denominator = HPH_data[0] + R;                    // (...) + 0.01
  denominator = (denominator < 1e-6f) ? 1e-6f : denominator;  // Avoid division by zero
  arm_mat_scale_f32(&numerator, 1.0f / denominator, &K_gain); // (H * P_check_k * H^T + 0.01)^(-1)

  // x_hat_k = x_check_k + K_k * (Z_k - H * x_check_k)
  arm_mat_mult_f32(&kf->H, &x_check, &temp_H_X);           // H * x_check_k
  float32_t residual = Z_k - temp_H_X_data[0];             // (Z_k - H * x_check_k)
  arm_mat_scale_f32(&K_gain, residual, &K_times_residual); // K_k * (Z_k - H * x_check_k)
  arm_mat_add_f32(&x_check, &K_times_residual, &x_hat);    // x_check_k + K_k * (Z_k - H * x_check_k)

  print_matrix_f32(&x_hat, "x_hat");

  // P_hat_k = (I - K_k * H) * P_check_k
  arm_mat_mult_f32(&K_gain, &kf->H, &KH);          // K_k * H
  arm_mat_sub_f32(&kf->I, &KH, &I_minus_KH);       // (I - K_k * H)
  arm_mat_mult_f32(&I_minus_KH, &P_check, &P_hat); // (I - K_k * H) * P_check_k

  print_matrix_f32(&P_hat, "P_hat");
  // clang-format on
}

static float32_t KF_get_pos_estimate(KF_2DOF_t *kf) {
  return kf->x_hat_prev_data[0];
}

static float32_t KF_get_vel_estimate(KF_2DOF_t *kf) {
  return kf->x_hat_prev_data[1];
}

static float32_t* KF_get_covariance_mat(KF_2DOF_t *kf) {
  return kf->P_hat_prev_data;
}

static void print_matrix_f32(const arm_matrix_instance_f32 *mat, const char *name) {
    printf("Matrix %s (%d x %d):\n", name, mat->numRows, mat->numCols);
    
    for (int i = 0; i < mat->numRows; i++) {
        for (int j = 0; j < mat->numCols; j++) {
            printf("%8.4f ", mat->pData[i * mat->numCols + j]); // Print with 4 decimal places
        }
        printf("\n");
    }
    
    printf("\n"); // Extra newline for readability
}

#endif // __KF_2DOF_H__
