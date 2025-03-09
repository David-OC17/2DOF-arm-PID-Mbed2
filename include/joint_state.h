#ifndef __JOINT_STATE_H__
#define __JOINT_STATE_H__

#include <std_msgs/msg/int16_multi_array.h>

#include "kf_2dof.h"
#include "common.h"

#define ENCODER_INTERRUPT_INTERVAL 1000

static float KALMAN_DT_MS = 0.01; // Time step in ms
static float KALMAN_Q_NOISE = 0.001; // Process noise (control - motor driver)
static float KALMAN_R_NOISE = 0.1; // Measurement noise (encoders)

static KF_2DOF joint_state_kalman_filter;

extern joint_state _measured_joint_state;
extern end_efector_state _estimated_end_efector_state;

joint_state take_measurement_encoders();

void init_joint_state_values(float link1_len, float link2_len);
void init_joint_state_kalman(float encoder1_init_pos, float encoder2_init_pos);
void calibrate_joint_state_kalman(uint8_t iterations);

void init_encoder_interrupt();

void joint_state_callback();
void update_encoder_motor1();
void update_encoder_motor2();

#endif // __JOINT_STATE_H__