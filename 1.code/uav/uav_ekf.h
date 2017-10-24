#ifndef  UAV_EKF_H
#define  UAV_EKF_H
#include "stm32f4xx.h"
#include "uav_matrix.h"

// Structure for storing AHRS states
typedef struct __AHRS_states {

    // Orientation states
    float psi;
    float theta;
    float phi;

    // Orientation rate states
    float psi_dot;
    float theta_dot;
    float phi_dot;

    // Process noise matrix
    fmat3x3 Q;

    // Measurement noise matrix
    fmat3x3 Racc;
    fmat3x3 Rmag;

    // EKF covariance
    fmat3x3 P;

} AHRS_states;

extern AHRS_states gEstimatedStates;

extern uint8_t EKF_Initialized;

// Function declarations

void EKF_Init(float axSensor, float aySensor, float azSensor, float mxSensor, float mySensor, float mzSensor);

void EKF_Predict(float p, float q, float r, float T);

void EKF_Update(float axSensor, float aySensor, float azSensor, float mxSensor, float mySensor, float mzSensor, float accelCutoff, uint8_t newMagData);


#endif