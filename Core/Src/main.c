/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "bsp_uart.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "pid.h"
#include "dji_motor_ctrl.h"
#include "dm_motor_ctrl.h"
#include "imu_task.h"
#include "chassis_ctrl.h"
#include "remote_ctrl.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern float gyro[3], accel[3], temp;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float dt = 0.001f;
uint32_t main_count = 0;
uint32_t main_dwt_count = 0;
uint32_t imu_count = 0;
float task_time = 0;
float wait_time = 0;
double l_phi1;
double l_phi2;
double r_phi1;
double r_phi2;
double l_d_phi1;
double l_d_phi2;
double r_d_phi1;
double r_d_phi2;
double l_t1;
double l_t2;
double r_t1;
double r_t2;

double last_lws;
double last_rws;
double s;

float vel_cmd;
float yaw_cmd;
float roll_cmd;
float leg_length_cmd;
uint8_t state_cmd;

void uart1_callback_function()
{
  bsp_uart_send(&huart1, UART1_Rx_Buff, 256);
}

void uart5_callback_function()
{
  rc_ctrl_fbdata(&rc_ctrl, UART5_Rx_Buff);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_FDCAN2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  DWT_Init(480);
  bsp_uart_init(&huart1, UART1_Rx_Buff, 256, uart1_callback_function);
  bsp_uart_init(&huart5, UART5_Rx_Buff, 256, uart5_callback_function);
  UART1_Tx_Data[0] = 0xAB;
  BMI088_init();
  bsp_can_init();
  INS_init();
  leg_controller_init(&leg_l, &leg_r, &leg_jl, &leg_jr, &k);
  velocity_kf_init(1.0f, 1000.0f, 100.0f);

  DM8009_init(&DM8009_1, &hfdcan1, TO4_MODE, 0x3FE, 0x301, -2.0795648111053424, 0.01, 1);
  DM8009_init(&DM8009_2, &hfdcan1, TO4_MODE, 0x3FE, 0x302, -2.1555061302862457, 0.01, 2);
  DM8009_init(&DM8009_3, &hfdcan1, TO4_MODE, 0x3FE, 0x303, -1.0547405441792126, 0.01, 3);
  DM8009_init(&DM8009_4, &hfdcan1, TO4_MODE, 0x3FE, 0x304, -2.0511827019165203, 0.01, 4);
  M3508_init(&M3508_1, 1, 268.0 / 17.0, 0.02, 5);
  M3508_init(&M3508_2, 2, 268.0 / 17.0, 0.02, 6);
  rc_ctrl_init(&rc_ctrl, 0.3, 7);

  // 等待imu温度达到目标
  while (imu_count < 1500)
  {
    imu_temp_ctrl(40.0f);
    if (temp >= 40.0f)
      imu_count ++;
    HAL_Delay(5);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 1000Hz循环
    DWT_GetDeltaT(&main_dwt_count);

    // 接收遥控器命令
    state_cmd = rc_ctrl.rc.s[0]; // 右拨杆s2

    vel_cmd = rc_ctrl.rc.ch[3] * 0.01f;
    if (vel_cmd > 5.0f)
      vel_cmd = 5.0f;
    else if (vel_cmd < - 5.0f)
      vel_cmd = - 5.0f;

    //左腿路程计算
    double lws = M3508_1.m.para.pos_fb;
    double delta_lws = lws - last_lws;
    //跨越编码器零点处理
    if (delta_lws < -PI)
      delta_lws += 2 * PI;
    else if (delta_lws > PI)
      delta_lws -= 2 * PI;
    double delta_ls = - delta_lws / (M3508_1.speed_ratio) * rw;
    last_lws = lws;
    //右腿路程计算
    double rws = M3508_2.m.para.pos_fb;
    double delta_rws = rws - last_rws;
    //跨越编码器零点处理
    if (delta_rws < -PI)
      delta_rws += 2 * PI;
    else if (delta_rws > PI)
      delta_rws -= 2 * PI;
    double delta_rs = delta_rws / (M3508_2.speed_ratio) * rw;
    last_rws = rws;
    s = (delta_ls + delta_rs) / 2;

    yaw_cmd = - rc_ctrl.rc.ch[0] * 0.02f;
    if (yaw_cmd > 8.0f)
      yaw_cmd = 8.0f;
    else if (yaw_cmd < - 8.0f)
      yaw_cmd = - 8.0f;

    leg_length_cmd = rc_ctrl.rc.ch[1] * 0.001f;
    if (leg_length_cmd > 0.15f)
      leg_length_cmd = 0.15f;
    else if (leg_length_cmd < -0.08f)
      leg_length_cmd = -0.08f;

    roll_cmd = rc_ctrl.rc.ch[2] * 0.001f;
    if (roll_cmd > 0.5f)
      roll_cmd = 0.5f;
    else if (roll_cmd < -0.5f)
      roll_cmd = -0.5f;

    rc_ctrl_refresh(&rc_ctrl);

    // 关节电机反馈数据处理
    l_phi1 = (5.0f * PI / 4.0f) - DM8009_1.m.para.pos_fb;
    l_phi2 = (5.0f * PI / 4.0f) + DM8009_2.m.para.pos_fb;
    r_phi1 = (5.0f * PI / 4.0f) + DM8009_3.m.para.pos_fb;
    r_phi2 = (5.0f * PI / 4.0f) - DM8009_4.m.para.pos_fb;
    l_d_phi1 = - DM8009_1.m.para.vel_fb;
    l_d_phi2 = DM8009_2.m.para.vel_fb;
    r_d_phi1 = DM8009_3.m.para.vel_fb;
    r_d_phi2 = - DM8009_4.m.para.vel_fb;
    l_t1 = - DM8009_1.m.para.tor_fb;
    l_t2 = DM8009_2.m.para.tor_fb;
    r_t1 = DM8009_3.m.para.tor_fb;
    r_t2 = - DM8009_4.m.para.tor_fb;

    // 姿态解算
    imu_task(dt);
    // 腿部运动学、支持力解算与机体速度卡尔曼滤波
    plc_obs(l_phi1, l_phi2, r_phi1, r_phi2, l_d_phi1, l_d_phi2, r_d_phi1, r_d_phi2, l_t1, l_t2, r_t1, r_t2, dt);

    k.Xd_data[1] = vel_cmd;
    k.Xd_data[2] += yaw_cmd * dt;
    k.Xd_data[3] = yaw_cmd;

    switch (state_cmd)
    {
      // 右拨杆：上1 中3 下2
      case 1: // 跨越地形模式
        if (leg_l.Fi > - 7.0f || leg_r.Fi > - 7.0f)
          // 离地处理
          plc_handler4(&leg_l, &leg_r, &leg_tl, &leg_tr, &leg_jl, &leg_jr, INS.Roll, dt);
        else
          plc_handler1(&leg_l, &leg_r, &leg_tl, &leg_tr, &leg_jl, &leg_jr, &k,
                      INS.YawTotalAngle, INS.Gyro[2], INS.Roll, INS.Gyro[1], INS.Pitch, INS.Gyro[0], dt);
        break;
      case 3: // 无位移闭环站立模式
        if (leg_l.Fi > - 7.0f || leg_r.Fi > - 7.0f)
          // 离地处理
          plc_handler4(&leg_l, &leg_r, &leg_tl, &leg_tr, &leg_jl, &leg_jr, INS.Roll, dt);
        else
          plc_handler2(&leg_l, &leg_r, &leg_tl, &leg_tr, &leg_jl, &leg_jr, &k,
                      INS.YawTotalAngle, INS.Gyro[2], INS.Roll, INS.Gyro[1], INS.Pitch, INS.Gyro[0], dt);
        break;
      case 2: // 所有电机失能
        plc_handler3(&leg_l, &leg_r, &leg_tl, &leg_tr, &leg_jl, &leg_jr);
        break;
      default:
        break;
    }

    M3508_cmd_upgrade(&M3508_1, - leg_tl.Tw);
    M3508_cmd_upgrade(&M3508_2, leg_tr.Tw);
    DM8009_cmd_upgrade(&DM8009_1, - leg_tl.T1);
    DM8009_cmd_upgrade(&DM8009_2, leg_tl.T2);
    DM8009_cmd_upgrade(&DM8009_3, leg_tr.T1);
    DM8009_cmd_upgrade(&DM8009_4, - leg_tr.T2);

    dji_motor_cmd_send(0x200, &hfdcan2, M3508_1.m.cmd.cmd_signal, M3508_2.m.cmd.cmd_signal, 0, 0);
    DM8009_send_1to4(0x3FE, &hfdcan1, DM8009_1.m.cmd.cmd_signal, DM8009_2.m.cmd.cmd_signal,
            DM8009_3.m.cmd.cmd_signal, DM8009_4.m.cmd.cmd_signal);

    // for (uint8_t i = 0; i < 4; i++)
    // {
    //     UART1_Tx_Data[i + 1] = *((uint8_t *)&INS.Accel[0] + i);
    //     UART1_Tx_Data[i + 5] = *((uint8_t *)&INS.Accel[1] + i);
    //     UART1_Tx_Data[i + 9] = *((uint8_t *)&INS.Accel[2] + i);
    //     UART1_Tx_Data[i + 13] = *((uint8_t *)&INS.temp + i);
    // }
    // bsp_uart_send(&huart1, UART1_Tx_Data, 17);

    task_time = DWT_GetDeltaT(&main_dwt_count);
    wait_time = dt - task_time;
    main_count ++;
    DWT_Delay(wait_time);
}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
