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
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "MPU6050.h"
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
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	uint16_t str2int(uint8_t *str); //字符数字转换函数声明
	
	extern DMA_HandleTypeDef hdma_usart1_rx;
	uint8_t Unsure_Message[16];
		QueueHandle_t xUartQueue;
    QueueHandle_t xParamQueue; 




//////////////////////////////////////////////////////////任务函数实现区///////////////////////////////////////////////////////////////////////////////
///////////////////////////////Tips：查了configTICK_RATE_HZ值为默认1000 也就是1Tick为1ms///////////////////////////////////////////////////////////////
void vPWMTask(void *pvParameters) {
	uint16_t MAXCCRX = 499;				//最大CCRX
	uint16_t CCRX = 0;						//占空比调节
	uint8_t Direction = 0;				//方向 0为增1为减
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // 启动TIM3_CH1的PWM输出
	uint8_t Zero_Flag = 0;
	
	TickType_t xLastWakeTime;  // 存储上一次唤醒时间
	xLastWakeTime = xTaskGetTickCount();  // 初始化：获取当前系统节拍数
	
	while (1)
  {	
		Zero_Flag = 0;
		if(xQueueReceive(xParamQueue, &MAXCCRX, 0) == pdTRUE) {
		CCRX = 0;	//每次接受就重启一下
		Direction = 0;	
		printf("Received MAXCCRX=%d\n", MAXCCRX);
}
		if(CCRX==MAXCCRX && Direction==0) Direction=1;
		if(CCRX==0 && Direction==1) Direction=0;
		if(CCRX==MAXCCRX && Direction==0 && CCRX==0) Zero_Flag = 1;	//MAXCCRX = 0的情况有点无解 只能写个标识来防了

		if(Zero_Flag == 0){
		if(Direction==0){
			CCRX += 1;
		}
		else{
			CCRX -= 1;
		}
	}
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,CCRX);
		vTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(2));
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void vUARTTask(void *pvParameters) {			//UART任务
    uint8_t UART_QueneMessage[16];
    uint16_t bright;

    while(1) {
        if (xQueueReceive(xUartQueue, UART_QueneMessage, portMAX_DELAY) == pdPASS) {
            // 直接把数字字符串转成整数
						printf("xUartQueue is Receive\n");
            bright = str2int(UART_QueneMessage);
						printf("Received bright=%d\n", bright);

            // 发送到亮度队列
            xQueueSend(xParamQueue, &bright, 0);
        }
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void vMPU6050Task(void *pvParameters) {
	float accel[3], gyro[3];			//角速度 加速度
	float pitch,roll,yaw;
	int ret = 0;
	vTaskDelay(pdMS_TO_TICKS(5000));
	printf("MPU6050 initing......\n");
	do{
		ret = MPU6050_DMP_init();
	}while(ret);
	if(MPU6050_DMP_init()==0) printf("MPU6050 init success!\n");

	uint8_t Check_MODE = 0;				//查阅模式 0为六轴 1为欧拉角

	TickType_t xLastWakeTime;  // 存储上一次唤醒时间
	xLastWakeTime = xTaskGetTickCount();  // 初始化：获取当前系统节拍数
	
  while (1){
	if(Check_MODE == 0){
		if (MPU6050_DMP_Get6Axis_f(accel, gyro) == 0){
    printf("AX:%.3f AY:%.3f AZ:%.3f  GX:%.3f GY:%.3f GZ:%.3f\n",
           accel[0], accel[1], accel[2],
           gyro[0], gyro[1], gyro[2]);
			}
		vTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(100));
		}
	if(Check_MODE == 1){
		if (MPU6050_DMP_Get_Date(&pitch, &roll, &yaw) == 0){
    printf("Pitch: %.2f, Roll: %.2f, Yaw: %.2f\r\n", pitch, roll, yaw);
			}
		vTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(100));
		}
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	
	
	
	
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
//////////////////////////////////////////////////////////DMA初始化//////////////////////////////////////////////////////////////
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,Unsure_Message,sizeof(Unsure_Message));
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////任务区/////////////////////////////////////////////////////////////
		TaskHandle_t PWM_TaskHandle = NULL;
		TaskHandle_t UART_TaskHandle = NULL;
		TaskHandle_t MPU6050_TaskHandle = NULL;
		xUartQueue  = xQueueCreate(10, sizeof(Unsure_Message));
    xParamQueue = xQueueCreate(10, sizeof(uint16_t));

	
	
		xTaskCreate(	
        vPWMTask,       // 任务函数
        "PWM_LED",    // 任务名称
        128,            // 栈大小（128字=512字节）
        NULL,           // 不传递参数
        1,              // 优先级1
        &PWM_TaskHandle          
    );
	xTaskCreate(	
        vUARTTask,       // 任务函数
        "UART",    // 任务名称
        128,            // 栈大小（128字=512字节）
        NULL,           // 不传递参数
        1,              // 优先级1
        &UART_TaskHandle           
    );
			xTaskCreate(	
        vMPU6050Task,       // 任务函数
        "MPU6050",    // 任务名称
        128,            // 栈大小（128字=512字节）
        NULL,           // 不传递参数
        2,              // 优先级1
        &MPU6050_TaskHandle         
    );
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	
	
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
/////////////////////////////////////////////////////DMA中断实现/////////////////////////////////////////////////////////////////////////////////////////////
	void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size){
			if(huart == &huart1){
			printf("huart1 is Receive\n");
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      xQueueSendFromISR(xUartQueue, Unsure_Message, &xHigherPriorityTaskWoken);		//有数据进来 就传到UART队列
				
      HAL_UARTEx_ReceiveToIdle_DMA(&huart1,Unsure_Message,sizeof(Unsure_Message));	//接受下一轮数据
			__HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
/////////////////////////////////////////////////////printf重写实现/////////////////////////////////////////////////////////////////////////////////////////
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1 , (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint16_t str2int(uint8_t *str)
{
    uint16_t num = 0;
    while (*str >= '0' && *str <= '9')  // 遍历数字字符
    {
        num = num * 10 + (*str - '0');  // 累加成整数
        str++;
    }
    return num;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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