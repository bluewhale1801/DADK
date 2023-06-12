/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "pid.h"
#include "stdio.h"
#include "serial.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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
Motor_t motor1, motor2;
PID_CONTROL_t pid1,pid2,pid3;
PROCESS_t process;


extern uint8_t g_nRxBuff[MAX_LEN];
extern uint8_t g_strCommand[4];
extern uint8_t  g_nRxData[16];
extern uint8_t g_kp[4];
extern uint8_t g_ki[4];
extern uint8_t g_kd[4];
extern uint8_t g_Setpoint[4];
extern bool g_bDataAvailable;


uint8_t g_strTxCommand[4];
uint8_t g_nTxData[16];
uint8_t* g_vel1;
uint8_t* g_pos;
uint8_t* g_vel2;

uint16_t k = 0;
uint16_t c = 0;
float velocitySetpoint;



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
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

// cho vao motor init
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);

  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_2);


  HAL_TIM_Base_Start_IT(&htim3);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  HAL_UART_Receive_IT(&huart3, (uint8_t *)g_nRxBuff, MAX_LEN);
  SerialInit();

  pid3.dKp = 0.35;
  pid3.dKi = 0.9;
  pid3.dKd = 0.0001;





  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(g_bDataAvailable == true)
		  	      {
		  	        if(StrCompare(g_strCommand, (uint8_t*)"SPID", 4))
		  	        {
		  	          process = SPID;
		  	        }
		  	        else if(StrCompare(g_strCommand, (uint8_t*)"VTUN", 4))
		  	        {
		  	          process = VTUN;
		  	        }
		  	        else if(StrCompare(g_strCommand, (uint8_t*)"PTUN", 4))
		  	        {
		  	          process = PTUN;
		  	        }
		  	      else if(StrCompare(g_strCommand, (uint8_t*)"STOP", 4))
		  	       {
		  	      	 process = STOP;
		  	       }
		  	        else
		  	        {
		  	          process = NONE;
		  	        }
		  	        g_bDataAvailable = false;
		  	      }
	  switch(process)
	 	  	      {
	 	  	        case NONE:
	 	  	          SerialAcceptReceive();
	 	  	          break;
	 	  	        case SPID:
	 	  	        	PIDReset(&pid1);
	 	  	        	PIDReset(&pid2);
	 	  	        	//get parameter
	 	  	        	pid1.dKp = (*(float*)g_kp);
	 	  	        	pid1.dKi = (*(float*)g_ki);
	 	  	        	pid1.dKd = (*(float*)g_kd);

	 	  	        	pid2.dKp = pid1.dKp;
	 	  	        	pid2.dKi = pid1.dKi;
	 	  	        	pid2.dKd = pid1.dKd ;
	 	  	        	//get setPoint
	 	  	        	motor1.setPoint=(*(float*)g_Setpoint);
	 	  	        	motor2.setPoint=motor1.setPoint;

	 	  	        	process= NONE;
	 	  	          break;
	 	  	        case VTUN:

	 	  	        	break;
	 	  	        case PTUN:

	 	  	        	break;
	 	  	        case STOP:

	 	  	        	 PIDReset(&pid1);
	 	  	        	 PIDReset(&pid2);
	 	  	        	 PIDReset(&pid3);

	 	  	        	 MotorReset(&motor1);
	 	  	        	 MotorReset(&motor2);

	 	  	        	 htim4.Instance->CNT=0;
	 	  	        	 htim1.Instance->CNT=0;

	 	  	        	 MotorSetDuty1(0);
	 	  	        	 MotorSetDuty2(0);

	 	  	        	 k = 0;
	 	  	        	 c = 0;

	 	  	        	process= NONE;
	 	  	        	break;
	 	  	         SerialAcceptReceive();
	 	  	      }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	  if(htim->Instance == htim3.Instance)
	  {

		  switch(process)
		  	 	  	      {
		  	 	  	        case NONE:
		  	 	  	          break;
		  	 	  	        case SPID:
		  	 	  	          break;
		  	 	  	        case VTUN:
							   k++;
//							   if(k % 700 == 0){
//								   motor1.setPoint+=50;
//								   motor2.setPoint+=50;
//							   }

							   ReadEncoder(&motor1, &htim4);
							   ReadEncoder(&motor2, &htim1);

							   MotorSetDuty1((int) MotorTuningVelocity(&pid1, &motor1, motor1.setPoint));
							   MotorSetDuty2((int) MotorTuningVelocity(&pid2, &motor2, motor2.setPoint));
							//cast data

							   g_vel2 = (uint8_t*)(&motor2.velocity);
							   g_vel1 = (uint8_t*)(&motor1.velocity);
							   g_pos = (uint8_t*)(&motor1.position);



							   memcpy(g_nTxData, g_vel2, 4);
							   memcpy(g_nTxData+4, g_vel1, 4);
							   memcpy(g_nTxData+8, g_pos, 4);



							//send data
							   SerialWriteComm(g_strCommand, g_nTxData);
		  	 	  	        	break;

		  	 	  	        case PTUN:

							   ReadEncoder(&motor1, &htim4);
							   ReadEncoder(&motor2, &htim1);
							   if(c==1)
							   {
								   motor1.setPoint = 60;
							   }

							   else if(c == 85)
							   {
								   motor1.setPoint = 0;
							   }
							   else if(c==170)
							   {
								   c=0;
							   }

							   if(k == 0){
								   velocitySetpoint =  MotorTuningPosition(&pid1, &motor1, motor1.setPoint);
							   }
							   MotorSetDuty1( MotorTuningVelocity(&pid3, &motor1, velocitySetpoint) );

							   k++;
							   if(k == 3){
								   k = 0;
							   }

							   c++;

		  	 	  	       //cast data
							   g_vel2= (uint8_t*)(&motor2.velocity);
							   g_vel1 = (uint8_t*)(&motor1.velocity);
							   g_pos = (uint8_t*)(&motor1.position);


							   memcpy(g_nTxData, g_vel2, 4);
							   memcpy(g_nTxData+4, g_vel1, 4);
							   memcpy(g_nTxData+8, g_pos, 4);

							//send data
							   SerialWriteComm(g_strCommand, g_nTxData);
		  	 	  	       	   break;
		  	 	  	        case STOP:
		  	 	  	        	break;
		  	 	  	      }
	  }
	  }

/* USER CODE END 4 */

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

#ifdef  USE_FULL_ASSERT
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
