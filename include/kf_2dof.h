#ifndef __KF_2DOF_H__
#define __KF_2DOF_H__

#include <math.h>
#include <stdio.h>

typedef struct {
    float dt;      // Time step
    float L1, L2;  // Link lengths
    float Q_noise; // Changing covariance Q matrix
    float R_noise; // Changing covariance R matrix

    // State vector [x_e, y_e, vx_e, vy_e]
    float x[4];

    // State covariance matrix
    float P[4][4];

    // State transition matrix A
    float A[4][4];

    // Process noise covariance Q
    float Q[4][4];

    // Measurement noise covariance R
    float R[2][2];
} KF_2DOF;

void KF_2DOF_init(KF_2DOF *kf, float dt, float L1, float L2, float Q_noise, float R_noise) {
    kf->dt = dt;
    kf->L1 = L1;
    kf->L2 = L2;
    kf->Q_noise = Q_noise;
    kf->R_noise = R_noise;

    // Init A matrix
    kf->A[0][0] = 1; kf->A[0][1] = 0; kf->A[0][2] = dt; kf->A[0][3] = 0;
    kf->A[1][0] = 0; kf->A[1][1] = 1; kf->A[1][2] = 0; kf->A[1][3] = dt;
    kf->A[2][0] = 0; kf->A[2][1] = 0; kf->A[2][2] = 1; kf->A[2][3] = 0;
    kf->A[3][0] = 0; kf->A[3][1] = 0; kf->A[3][2] = 0; kf->A[3][3] = 1;

    // Init covariance Q matrix
    kf->Q[0][0] = Q_noise; kf->Q[0][1] = 0;       kf->Q[0][2] = 0;       kf->Q[0][3] = 0;
    kf->Q[1][0] = 0;       kf->Q[1][1] = Q_noise; kf->Q[1][2] = 0;       kf->Q[1][3] = 0;
    kf->Q[2][0] = 0;       kf->Q[2][1] = 0;       kf->Q[2][2] = Q_noise; kf->Q[2][3] = 0;
    kf->Q[3][0] = 0;       kf->Q[3][1] = 0;       kf->Q[3][2] = 0;       kf->Q[3][3] = Q_noise;

    // Init covariance R matrix
    kf->R[0][0] = 0.01; kf->R[0][1] = 0;
    kf->R[1][0] = 0;    kf->R[1][1] = 0.01;

    // Init state vector x
    kf->x[0] = 0; kf->x[1] = 0; kf->x[2] = 0; kf->x[3] = 0;

    // Init covariance P matrix
    kf->P[0][0] = 1; kf->P[0][1] = 0; kf->P[0][2] = 0; kf->P[0][3] = 0;
    kf->P[1][0] = 0; kf->P[1][1] = 1; kf->P[1][2] = 0; kf->P[1][3] = 0;
    kf->P[2][0] = 0; kf->P[2][1] = 0; kf->P[2][2] = 1; kf->P[2][3] = 0;
    kf->P[3][0] = 0; kf->P[3][1] = 0; kf->P[3][2] = 0; kf->P[3][3] = 1;
}

void KF_2DOF_predict(KF_2DOF *kf) {
    // Predict state: x = A * x
    float x_new[4];
    x_new[0] = kf->A[0][0] * kf->x[0] + kf->A[0][2] * kf->x[2];
    x_new[1] = kf->A[1][1] * kf->x[1] + kf->A[1][3] * kf->x[3];
    x_new[2] = kf->x[2];
    x_new[3] = kf->x[3];

    // Update state vector
    kf->x[0] = x_new[0];
    kf->x[1] = x_new[1];
    kf->x[2] = x_new[2];
    kf->x[3] = x_new[3];

    // Predict covariance: P = A * P * A^T + Q
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            kf->P[i][j] += kf->Q[i][j];
        }
    }
}

void KF_2DOF_update(KF_2DOF *kf, float theta1, float theta2) {
    // Compute measurement prediction
    float hx[2];
    hx[0] = kf->L1 * cos(theta1) + kf->L2 * cos(theta1 + theta2);
    hx[1] = kf->L1 * sin(theta1) + kf->L2 * sin(theta1 + theta2);

    // Compute measurement residual
    float y[2] = {hx[0] - kf->x[0], hx[1] - kf->x[1]};

    // Compute Kalman Gain K = P * H^T * (H * P * H^T + R)^-1
    float S[2][2] = {{kf->P[0][0] + kf->R[0][0], kf->P[0][1]},
                     {kf->P[1][0], kf->P[1][1] + kf->R[1][1]}};
    float K[4][2] = {{kf->P[0][0] / S[0][0], kf->P[0][1] / S[1][1]},
                     {kf->P[1][0] / S[0][0], kf->P[1][1] / S[1][1]},
                     {kf->P[2][0] / S[0][0], kf->P[2][1] / S[1][1]},
                     {kf->P[3][0] / S[0][0], kf->P[3][1] / S[1][1]}};

    // Update state: x = x + K * y
    kf->x[0] += K[0][0] * y[0] + K[0][1] * y[1];
    kf->x[1] += K[1][0] * y[0] + K[1][1] * y[1];
    kf->x[2] += K[2][0] * y[0] + K[2][1] * y[1];
    kf->x[3] += K[3][0] * y[0] + K[3][1] * y[1];

    // Update covariance: P = (I - K * H) * P
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            kf->P[i][j] -= K[i][0] * kf->P[0][j] + K[i][1] * kf->P[1][j];
        }
    }
}

float KF_2DOF_getX(KF_2DOF *kf) {
    return kf->x[0];
}

float KF_2DOF_getY(KF_2DOF *kf) {
    return kf->x[1];
}

float KF_2DOF_getVX(KF_2DOF *kf) {
    return kf->x[2];
}

float KF_2DOF_getVY(KF_2DOF *kf) {
    return kf->x[3];
}


#endif // __KF_2DOF_H__