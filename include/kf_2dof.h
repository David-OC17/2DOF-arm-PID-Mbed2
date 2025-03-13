#ifndef __KF_2DOF_H__
#define __KF_2DOF_H__

#include <arm_math.h>

static const float T =  0.01;
static const float b = 0.1;
static const float J = 0.01;
static const float K = 0.2;
static const float q_theta = 0.01;
static const float q_theta_dot = 0.01;
static const float R = 0.01;

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
  kf->x_hat_prev_data[0] = 0;
  kf->x_hat_prev_data[1] = 0;
  kf->P_hat_prev_data[0] = 1;
  kf->P_hat_prev_data[1] = 0;
  kf->P_hat_prev_data[2] = 0;
  kf->P_hat_prev_data[3] = 1;

  arm_mat_init_f32(&kf->x_hat_prev, 2, 1, kf->x_hat_prev_data);
  arm_mat_init_f32(&kf->P_hat_prev, 2, 2, kf->P_hat_prev_data);

  kf->F_data[0] = 1;
  kf->F_data[1] = T;
  kf->F_data[2] = 0;
  kf->F_data[3] = 1 - (b / J) * T;
  arm_mat_init_f32(&kf->F, 2, 2, kf->F_data);

  kf->B_data[0] = 0;
  kf->B_data[1] = (K / J) * T;
  arm_mat_init_f32(&kf->B, 2, 1, kf->B_data);

  kf->Q_data[0] = q_theta;
  kf->Q_data[1] = 0;
  kf->Q_data[2] = 0;
  kf->Q_data[3] = q_theta_dot;
  arm_mat_init_f32(&kf->Q, 2, 2, kf->Q_data);

  kf->H_data[0] = 1;
  kf->H_data[1] = 0;
  arm_mat_init_f32(&kf->H, 1, 2, kf->H_data);

  kf->H_transposed_data[0] = 1;
  kf->H_transposed_data[1] = 0;
  arm_mat_init_f32(&kf->H_transposed, 2, 1, kf->H_transposed_data);

  kf->I_data[0] = 1;
  kf->I_data[1] = 0;
  kf->I_data[2] = 0;
  kf->I_data[3] = 1;
  arm_mat_init_f32(&kf->I, 2, 2, kf->I_data);
}

static void KF_Update(KF_2DOF_t *kf, float32_t u_prev, float32_t Z_k) {
  float32_t x_check_data[2], P_check_data[4];
  float32_t temp1_data[4], temp2_data[4], control_data[2];
  float32_t temp_H_P_data[2], HPH_data[1], numerator_data[2];
  float32_t K_gain_data[2], K_times_residual_data[2], KH_data[4],
      I_minus_KH_data[4];

  arm_matrix_instance_f32 x_check, P_check, temp1, temp2, control;
  arm_matrix_instance_f32 temp_H_P, HPH, numerator, K_gain, K_times_residual;
  arm_matrix_instance_f32 KH, I_minus_KH, x_hat, P_hat;

  arm_mat_init_f32(&x_check, 2, 1, x_check_data);
  arm_mat_init_f32(&P_check, 2, 2, P_check_data);
  arm_mat_init_f32(&temp1, 2, 2, temp1_data);
  arm_mat_init_f32(&temp2, 2, 2, temp2_data);
  arm_mat_init_f32(&control, 2, 1, control_data);
  arm_mat_init_f32(&temp_H_P, 1, 2, temp_H_P_data);
  arm_mat_init_f32(&HPH, 1, 1, HPH_data);
  arm_mat_init_f32(&numerator, 2, 1, numerator_data);
  arm_mat_init_f32(&K_gain, 2, 1, K_gain_data);
  arm_mat_init_f32(&K_times_residual, 2, 1, K_times_residual_data);
  arm_mat_init_f32(&KH, 2, 2, KH_data);
  arm_mat_init_f32(&I_minus_KH, 2, 2, I_minus_KH_data);
  arm_mat_init_f32(&x_hat, 2, 1, kf->x_hat_prev_data);
  arm_mat_init_f32(&P_hat, 2, 2, kf->P_hat_prev_data);

  arm_mat_mult_f32(&kf->F, &kf->x_hat_prev, &x_check);
  arm_mat_scale_f32(&kf->B, u_prev, &control);
  arm_mat_add_f32(&x_check, &control, &x_check);

  arm_mat_mult_f32(&kf->F, &kf->P_hat_prev, &temp1);
  arm_mat_mult_f32(&temp1, &kf->F, &P_check);
  arm_mat_add_f32(&P_check, &kf->Q, &P_check);

  arm_mat_mult_f32(&P_check, &kf->H_transposed, &numerator);
  arm_mat_mult_f32(&kf->H, &P_check, &temp_H_P);
  arm_mat_mult_f32(&temp_H_P, &kf->H_transposed, &HPH);

  float32_t denominator = HPH_data[0] + R;
  denominator = (denominator < 1e-6f) ? 1e-6f : denominator;
  arm_mat_scale_f32(&numerator, 1.0f / denominator, &K_gain);

  float32_t residual = Z_k - temp_H_P_data[0];
  arm_mat_scale_f32(&K_gain, residual, &K_times_residual);
  arm_mat_add_f32(&x_check, &K_times_residual, &x_hat);

  arm_mat_mult_f32(&K_gain, &kf->H, &KH);
  arm_mat_sub_f32(&kf->I, &KH, &I_minus_KH);
  arm_mat_mult_f32(&I_minus_KH, &P_check, &P_hat);
}

static float32_t KF_get_pos_estimate(KF_2DOF_t *kf) { return kf->x_hat_prev_data[0]; }

static float32_t KF_get_vel_estimate(KF_2DOF_t *kf) { return kf->x_hat_prev_data[1]; }

#endif // __KF_2DOF_H__
