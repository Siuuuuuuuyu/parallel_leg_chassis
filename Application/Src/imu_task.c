//
// Created by sy on 2025/12/16.
//
#include "imu_task.h"

#define TEMPERATURE_PID_KP 1600.0f
#define TEMPERATURE_PID_KI 0.2f
#define TEMPERATURE_PID_KD 0.0f
#define TEMPERATURE_PID_MAX_OUT 4500.0f
#define TEMPERATURE_PID_MAX_IOUT 4400.0f

INS_t INS;
const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
uint8_t ins_debug_mode = 0;

float gyro[3], accel[3], temp;
float angle[3];
const float gravity[3] = {0, 0, 9.81f};

const float imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};

pid_type_def imu_temp_pid;

void get_angle(float q0, float q1, float q2, float q3, float angle[3])
{
    angle[0] = atan2f(2.0f*(q0*q3+q1*q2), 2.0f*(q0*q0+q1*q1)-1.0f);
    angle[1] = asinf(-2.0f*(q1*q3-q0*q2));
    angle[2] = atan2f(2.0f*(q0*q1+q2*q3),2.0f*(q0*q0+q3*q3)-1.0f);
}

void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

void INS_init(void)
{
    IMU_QuaternionEKF_Init(10, 0.001f, 10000000, 1, 0);
    // imu heat init
    PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    INS.AccelLPF = 0.0085f;
}

void imu_task(void const * argument)
{
    INS_init();
    static uint32_t count = 0;
    BMI088_init();

    while (1)
    {
        dt = DWT_GetDeltaT(&INS_DWT_Count);
        t += dt;

        BMI088_read(gyro, accel, &temp);
        INS.Accel[0] = accel[0];
        INS.Accel[1] = accel[1];
        INS.Accel[2] = accel[2];
        INS.Gyro[0] = gyro[0];
        INS.Gyro[1] = gyro[1];
        INS.Gyro[2] = gyro[2];

        // EKF更新四元数
        IMU_QuaternionEKF_Update(INS.Gyro[0], INS.Gyro[1], INS.Gyro[2],
                                INS.Accel[0], INS.Accel[1], INS.Accel[2], dt);
        // 复制更新后的四元数
        memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

        // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
        BodyFrameToEarthFrame(xb, INS.xn, INS.q);
        BodyFrameToEarthFrame(yb, INS.yn, INS.q);
        BodyFrameToEarthFrame(zb, INS.zn, INS.q);

        // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
        float gravity_b[3];
        EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
        // 从加速度计数据中减去重力分量，得到运动加速度
        for (uint8_t i = 0; i < 3; i++)
        {
            // 一阶低通滤波器滤波
            INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
        }
        // 将运动加速度转换回导航坐标系
        BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q);

        INS.Yaw = QEKF_INS.Yaw;
        INS.Pitch = QEKF_INS.Pitch;
        INS.Roll = QEKF_INS.Roll;
        INS.YawTotalAngle = QEKF_INS.YawTotalAngle;

        if (count % 2 == 0)
        {
            //500hz频率
            PID_calculate(&imu_temp_pid, 40.0f, temp);
            if (imu_temp_pid.out < 0.0f)
            {
                imu_temp_pid.out = 0.0f;
            }
            bsp_pwm_set(&htim3, TIM_CHANNEL_4, (uint16_t)imu_temp_pid.out);

            // for (uint8_t i = 0; i < 4; i++)
            // {
            //     UART1_Tx_Data[i + 1] = *((uint8_t *)&INS.Yaw + i);
            //     UART1_Tx_Data[i + 5] = *((uint8_t *)&INS.Pitch + i);
            //     UART1_Tx_Data[i + 9] = *((uint8_t *)&INS.Roll + i);
            //     UART1_Tx_Data[i + 13] = *((uint8_t *)&temp + i);
            // }
            // bsp_uart_send(&huart1, UART1_Tx_Data, 17);
        }
        count ++;
    }
}
