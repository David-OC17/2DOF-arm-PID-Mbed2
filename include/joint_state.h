#ifndef __JOINT_STATE_H__
#define __JOINT_STATE_H__

#include "kf_2dof.h"
#include "common.h"

#define ENCODER_INTERRUPT_INTERVAL 1000

static float KALMAN_DT_MS = 0.01; // Time step in ms
static float KALMAN_Q_NOISE = 0.001; // Process noise (control - motor driver)
static float KALMAN_R_NOISE = 0.1; // Measurement noise (encoders)

extern KF_2DOF_t _joint1_kalman_filter;
extern KF_2DOF_t _joint2_kalman_filter;

extern joint_state _measured_joint_state;
extern full_joint_state _approximate_joint_state;

joint_state take_measurement_encoders();

void init_joint_state_values();
void init_joint_state_kalman(float encoder1_init_pos, float encoder2_init_pos);
void calibrate_joint_state_kalman(uint8_t iterations);

void init_encoder_interrupt();

void joint_state_callback();
void update_encoder_motor(uint gpio, uint32_t events);

#endif // __JOINT_STATE_H__