/*
 * File      : buffer.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2026, Fy Development Team
 *
 * Thread-safe byte ring buffer for RT-Thread.
 *
 * 用陀螺仪预测姿态，使用加速度修正姿态的四元数拓展卡尔曼滤波，同时估计陀螺仪的零漂，支持多传感器实例。
 */


#ifndef QUATERNION_EKF_INTEGRATED_H
#define QUATERNION_EKF_INTEGRATED_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "my_math.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * The application may redirect these macros to a deterministic memory pool.
 * Kalman_Filter_Init() allocates its working matrices at runtime.
 */
#ifndef user_malloc
#define user_malloc malloc
#endif

#define RAD_TO_DEG 57.295779513f

typedef struct kalman_filter_s
{
    float *FilteredValue;		/* 滤波输出的数组 */
    float *MeasuredVector;		/* 原始测量值 */
    float *ControlVector;

    uint8_t xhatSize;			/* 状态数 */
    uint8_t uSize;				/* 控制输入数 */
    uint8_t zSize;				/* 观测维数 */	
    uint8_t Initialized;

    uint8_t UseAutoAdjustment;
    uint8_t MeasurementValidNum;

    uint8_t *MeasurementMap;
    float *MeasurementDegree;
    float *MatR_DiagonalElements;
    float *StateMinVariance;
    uint8_t *temp;

    /* Allow callbacks to recover their owning EKF instance. */
    void *UserData;

    math_matrixf_t xhat;        /* 本轮修正后的状态 */
    math_matrixf_t xhatminus;   /* 本轮预测的状态 */
    math_matrixf_t u;
    math_matrixf_t z;
    math_matrixf_t P;           /* 状态估计误差协方差 */
    math_matrixf_t Pminus;      /* 预测协方差 */
    math_matrixf_t F;           /* 状态转移矩阵 */
    math_matrixf_t FT;
    math_matrixf_t B;
    math_matrixf_t H;           /* 观测函数的雅可比矩阵 */
    math_matrixf_t HT;
    math_matrixf_t Q;           /* 过程噪声协方差 */
    math_matrixf_t R;           /* 测量噪声协方差 */
    math_matrixf_t K;           /* 卡尔曼增益 */
    math_matrixf_t S;           /* 新息协方差 */
    math_matrixf_t temp_matrix;
    math_matrixf_t temp_matrix1;
    math_matrixf_t temp_vector;
    math_matrixf_t temp_vector1;

    int8_t MatStatus;

    void (*User_Callback)(struct kalman_filter_s *kf);
    void (*User_Callback1)(struct kalman_filter_s *kf);
    void (*User_Callback2)(struct kalman_filter_s *kf);
    void (*User_Callback3)(struct kalman_filter_s *kf);
    void (*User_Callback4)(struct kalman_filter_s *kf);
    void (*User_Callback5)(struct kalman_filter_s *kf);
    void (*User_Callback6)(struct kalman_filter_s *kf);

    float *xhat_data;
    float *xhatminus_data;          
    float *u_data;
    float *z_data;
    float *P_data;
    float *Pminus_data;
    float *F_data;
    float *FT_data;
    float *B_data;
    float *H_data;
    float *HT_data;
    float *Q_data;
    float *R_data;
    float *K_data;
    float *S_data;
    float *temp_matrix_data;
    float *temp_matrix_data1;
    float *temp_vector_data;
    float *temp_vector_data1;
} kalman_filter_t;

/*
 * Sensor callback contract:
 *   context        user-provided context
 *   sensor_index   gyro or accelerometer instance index
 *   data           output x/y/z in SI units
 *   return         0 on success, non-zero on failure
 */
typedef int8_t (*QEKF_SensorReadFn)(void *context,
                                    uint8_t sensor_index,
                                    float data[3]);

 /* 完整的四元数 EKF 实例 */
typedef struct QEKF_INS_s {
    
    uint8_t Initialized;
    uint8_t Index;

    struct kalman_filter_s IMU_QuaternionEKF;

    uint8_t ConvergeFlag;        /* 收敛标志 */
    uint8_t StableFlag;          /* 稳定标志 */
    uint32_t ErrorCount;
    uint32_t UpdateCount;

    float q[4];
    float GyroBias[3];

    float Gyro[3];
    float Accel[3];
    float OrientationCosine[3];

    float accLPFcoef;           /* 加速度低通滤波器参数 */
    float gyro_norm;            /* 角速度向量模长 */
    float accl_norm;            /* 加速度向量模长 */
    float Scale;

    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;

    float Q1;
    float Q2;
    float R;                         /* 观测噪声参数 */

    float dt;
    math_matrixf_t ChiSquare;        /* 卡方 */
    float ChiSquare_Data[1];
    float ChiSquareThreshold;    /* 卡方检验阈值 */
    float lambda;                    /* 遗忘因子 */

    int16_t YawRoundCount;
    float YawAngleLast;

    /* Per-instance diagnostic snapshots, avoiding shared global matrices. */
    float P_snap[36];
    float K_snap[18];
    float H_snap[18];

    /* Per-instance sensors and adapter callbacks. */
    uint8_t GyroInstance;
    uint8_t AccelInstance;
    void *SensorContext;
} QEKF_INS_t;

/* Multi-instance API. */
bool IMU_QuaternionEKF_InitInstance(QEKF_INS_t *qe,
                                    uint8_t instance_index,
                                    float process_noise1,
                                    float process_noise2,
                                    float measure_noise,
                                    float lambda,
                                    float lpf);

void IMU_QuaternionEKF_DeinitInstance(QEKF_INS_t *qe);

/* 姿态更新 */
bool IMU_QuaternionEKF_UpdateInstance(QEKF_INS_t *qe,
                                      float gx,
                                      float gy,
                                      float gz,
                                      float ax,
                                      float ay,
                                      float az,
                                      float dt);

/* 绑定传感器读取方法 */
bool IMU_QuaternionEKF_SetSensorProvider(QEKF_INS_t *qe,
                                         uint8_t gyro_instance,
                                         uint8_t accel_instance,
                                         void *sensor_context);

/*
 * Reads the configured gyro and accelerometer instances and performs one EKF
 * update. This is the multi-IMU integration entry point.
 */
bool IMU_QuaternionEKF_SensorsInstance(QEKF_INS_t *qe, float dt);

#ifdef __cplusplus
}
#endif

#endif
