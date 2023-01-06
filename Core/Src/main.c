/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "usbd_cdc_if.h"
#include "message_buffer.h"
#include "FreeRTOS_CLI.h"
#include "string.h"
//#include "arm_const_structs.h"
#include "semphr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_INPUT_LENGTH    50
#define MAX_OUTPUT_LENGTH   512

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
TaskHandle_t task_server;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t adcBuffer[256];
float ReIm[256*2];
float mod[256];
xSemaphoreHandle sem;
uint16_t sin_wave[256] = {2048, 2098, 2148, 2199, 2249, 2299, 2349, 2399, 2448, 2498, 2547, 2596, 2644, 2692, 2740,
		2787, 2834, 2880, 2926, 2971, 3016, 3060, 3104, 3147, 3189, 3230, 3271, 3311, 3351, 3389,
		3427, 3464, 3500, 3535, 3569, 3602, 3635, 3666, 3697, 3726, 3754, 3782, 3808, 3833, 3857,
		3880, 3902, 3923, 3943, 3961, 3979, 3995, 4010, 4024, 4036, 4048, 4058, 4067, 4074, 4081,
		4086, 4090, 4093, 4095, 4095, 4094, 4092, 4088, 4084, 4078, 4071, 4062, 4053, 4042, 4030,
		4017, 4002, 3987, 3970, 3952, 3933, 3913, 3891, 3869, 3845, 3821, 3795, 3768, 3740, 3711,
		3681, 3651, 3619, 3586, 3552, 3517, 3482, 3445, 3408, 3370, 3331, 3291, 3251, 3210, 3168,
		3125, 3082, 3038, 2994, 2949, 2903, 2857, 2811, 2764, 2716, 2668, 2620, 2571, 2522, 2473,
		2424, 2374, 2324, 2274, 2224, 2174, 2123, 2073, 2022, 1972, 1921, 1871, 1821, 1771, 1721,
		1671, 1622, 1573, 1524, 1475, 1427, 1379, 1331, 1284, 1238, 1192, 1146, 1101, 1057, 1013,
		970, 927, 885, 844, 804, 764, 725, 687, 650, 613, 578, 543, 509, 476, 444,
		414, 384, 355, 327, 300, 274, 250, 226, 204, 182, 162, 143, 125, 108, 93,
		78, 65, 53, 42, 33, 24, 17, 11, 7, 3, 1, 0, 0, 2, 5,
		9, 14, 21, 28, 37, 47, 59, 71, 85, 100, 116, 134, 152, 172, 193,
		215, 238, 262, 287, 313, 341, 369, 398, 429, 460, 493, 526, 560, 595, 631,
		668, 706, 744, 784, 824, 865, 906, 948, 991, 1035, 1079, 1124, 1169, 1215, 1261,
		1308, 1355, 1403, 1451, 1499, 1548, 1597, 1647, 1696, 1746, 1796, 1846, 1896, 1947, 1997,
		2047};

uint16_t sin_wave_3rd_harmonic[256] = {2048, 2136, 2224, 2311, 2398, 2484, 2569, 2652, 2734, 2814, 2892, 2968, 3041, 3112, 3180,
		3245, 3308, 3367, 3423, 3476, 3526, 3572, 3615, 3654, 3690, 3723, 3752, 3778, 3800, 3819,
		3835, 3848, 3858, 3866, 3870, 3872, 3871, 3869, 3864, 3857, 3848, 3838, 3827, 3814, 3801,
		3786, 3771, 3756, 3740, 3725, 3709, 3694, 3679, 3665, 3652, 3639, 3628, 3617, 3608, 3600,
		3594, 3589, 3585, 3584, 3583, 3584, 3587, 3591, 3597, 3604, 3613, 3622, 3633, 3645, 3658,
		3672, 3686, 3701, 3717, 3732, 3748, 3764, 3779, 3794, 3808, 3821, 3833, 3844, 3853, 3860,
		3866, 3870, 3872, 3871, 3868, 3862, 3854, 3842, 3828, 3810, 3789, 3765, 3738, 3707, 3673,
		3635, 3594, 3549, 3501, 3450, 3396, 3338, 3277, 3213, 3146, 3077, 3005, 2930, 2853, 2774,
		2693, 2611, 2527, 2441, 2355, 2268, 2180, 2092, 2003, 1915, 1827, 1740, 1654, 1568, 1484,
		1402, 1321, 1242, 1165, 1090, 1018, 949, 882, 818, 757, 699, 645, 594, 546, 501,
		460, 422, 388, 357, 330, 306, 285, 267, 253, 241, 233, 227, 224, 223, 225,
		229, 235, 242, 251, 262, 274, 287, 301, 316, 331, 347, 363, 378, 394, 409,
		423, 437, 450, 462, 473, 482, 491, 498, 504, 508, 511, 512, 511, 510, 506,
		501, 495, 487, 478, 467, 456, 443, 430, 416, 401, 386, 370, 355, 339, 324,
		309, 294, 281, 268, 257, 247, 238, 231, 226, 224, 223, 225, 229, 237, 247,
		260, 276, 295, 317, 343, 372, 405, 441, 480, 523, 569, 619, 672, 728, 787,
		850, 915, 983, 1054, 1127, 1203, 1281, 1361, 1443, 1526, 1611, 1697, 1784, 1871, 1959,
		2047};

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
//	xSemaphoreGiveFromISR(sem, pdFALSE);
//}

volatile int counter = 0;
volatile int flag = 0;
volatile float fund_phase;
//void HAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef* hadc){
//	counter++;
//	if (counter >= 120){
//		counter = 0;
//		HAL_TIM_Base_Stop(&htim2);
//		HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_2);
//
//		//if (flag == 0){
//			HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, (uint32_t*)sin_wave_3rd_harmonic, 256, DAC_ALIGN_12B_R);
//			flag = 1;
//		//}else{
//			//HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, (uint32_t*)sin_wave, 256, DAC_ALIGN_12B_R);
//			//flag = 0;
//		//}
//	    HAL_TIM_Base_Start(&htim2);
//	}
//}

//void task_harmonics(void *param){
//	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuffer, 256);
//    HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, (uint32_t*)sin_wave_3rd_harmonic, 256, DAC_ALIGN_12B_R);
//
//	HAL_TIM_Base_Start(&htim8);
//    HAL_TIM_Base_Start(&htim2);
//
//	while(1){
//		xSemaphoreTake(sem, portMAX_DELAY);
//
//		int k = 0;
//		for(int i = 0; i < 256; i++){
//			ReIm[k] = (float)adcBuffer[i] * 0.0007326007;
//			ReIm[k+1] = 0.0;
//			k += 2;
//		}
//
//		arm_cfft_f32(&arm_cfft_sR_f32_len256,ReIm,0,1);
//		arm_cmplx_mag_f32(ReIm,mod,256);
//		arm_scale_f32(mod, 0.0078125, mod, 128);
//
//		fund_phase = atan2f(ReIm[3],ReIm[2])*180/M_PI;	//Fase R da harmonica fundamental
//		(void)fund_phase;
//	}
//}

uint32_t is_usb_on(void);
typedef struct led_t_ {
	GPIO_TypeDef *port;
	uint16_t pin;
	int timeout;
}led_t;

led_t green_led;
led_t red_led;

void task_led(void *param){
	led_t *led = (led_t *)param;
	while(1){
		if (led->timeout == 0){
			vTaskDelay(100);

		}
		else{
			HAL_GPIO_TogglePin(led->port, led->pin);
			vTaskDelay(led->timeout);
		}
	}
}


void task_usb(void *param){
	uint8_t text[] = "\n\rTeste de USB 2\n\r";
	uint16_t size;
	size = sizeof(text);
	while(!is_usb_on()){
		vTaskDelay(100);
	}
	while(1){
		vTaskDelay(1000);
		queue_print((uint8_t*)text, size);
	}
}

void task_print(void *param){
	uint8_t buf[512];
	while(!is_usb_on()) vTaskDelay(100);
	while(1){
		Print_Task((uint8_t *)&buf);
	}
}


static BaseType_t prvTaskStatsCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    ( void ) xWriteBufferLen;
    vTaskList(pcWriteBuffer);
    return pdFALSE;
}

static BaseType_t prvLedCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    ( void ) xWriteBufferLen;
    led_t* led;
    const char* parameters;
    char parameter1[10], parameter2[10], parameter3[10];

    BaseType_t xParameterStringLength;
    GPIO_PinState pin_state;

    parameters = FreeRTOS_CLIGetParameter( pcCommandString,
    				1,
					&xParameterStringLength );
    strncpy(parameter1, parameters, xParameterStringLength);
    parameter1[xParameterStringLength] = '\0';

    if (strcmp("green",parameter1) == 0){
    	led = &green_led;
    }
	else if(strcmp("red",parameter1) == 0){

		led = &red_led;
	}

	else {
		strncpy( pcWriteBuffer, "invalid parameter. should be red or green", 1 + strlen("invalid parameter. should be red or green") );
		return pdFALSE;
	}
	parameters = FreeRTOS_CLIGetParameter( pcCommandString,
					2,
					&xParameterStringLength );
	strncpy(parameter2, parameters, xParameterStringLength);
	parameter2[xParameterStringLength] = '\0';

	if (strcmp("on",parameter2) == 0){
		led->timeout = 0;
		pin_state = HAL_GPIO_ReadPin(led->port, led->pin);
		if (!pin_state){
			HAL_GPIO_TogglePin(led->port, led->pin);
			strncpy( pcWriteBuffer, "led on", 1 + strlen("led on"));
		}
	}
	else if (strcmp("off",parameter2) == 0){
		led->timeout = 0;
		pin_state = HAL_GPIO_ReadPin(led->port, led->pin);
		if (pin_state){
			HAL_GPIO_TogglePin(led->port, led->pin);
			strncpy( pcWriteBuffer, "led off", 1 + strlen("led off"));

		}
		else{
			strncpy( pcWriteBuffer, "led already off", 1 + strlen("led already off"));

		}

	}
	else if(strcmp("toggle",parameter2) == 0){
		parameters = FreeRTOS_CLIGetParameter( pcCommandString,
				3,
				&xParameterStringLength );
		strncpy(parameter3, parameters, xParameterStringLength);
		parameter3[xParameterStringLength] = '\0';
		if (parameter3[0] == '\0'){
			led->timeout = 0;
			HAL_GPIO_TogglePin(led->port, led->pin);
			strncpy( pcWriteBuffer, "led toggle", 1 + strlen("led toggle"));
		}
		else {
			int32_t num = atoi(parameter3);
			if (num < 0){
				strncpy( pcWriteBuffer, "invalid parameter. should be 0 or higher", 1 + strlen("invalid parameter. should be 0 or higher"));
			}
			else{
				led->timeout = num;
				strncpy( pcWriteBuffer, "led toggle interval set", 1 + strlen("led toggle interval set"));
			}

		}

	}
	else{
		strncpy( pcWriteBuffer, "invalid parameter. should be on, off, or toggle xx", 1 + strlen("invalid parameter. should be on, off, or toggle xx"));
	}
    return pdFALSE;
}

static BaseType_t prvHarmonicCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    ( void ) xWriteBufferLen;
    const char* parameters;
    char parameter1[10], parameter2[10];
    BaseType_t xParameterStringLength;

    parameters = FreeRTOS_CLIGetParameter( pcCommandString,
    				1,
					&xParameterStringLength );
    strncpy(parameter1, parameters, xParameterStringLength);
    parameter1[xParameterStringLength] = '\0';

    if (strcmp("cc",parameter1) == 0){
    	sprintf(pcWriteBuffer, "%f", 0.5*mod[0]);
    }
    else{
    	int16_t init = atoi(parameter1);
    	if ((init > 255) || (init < 0)){
    		strncpy( pcWriteBuffer, "invalid parameter. should be 0<=x<255", 1 + strlen("invalid parameter. should be 0<=x<255"));
    		return pdFALSE;
    	}
    	else{
    	    int16_t amount;
    		parameters = FreeRTOS_CLIGetParameter( pcCommandString,
    	    				2,
    						&xParameterStringLength );
    	    strncpy(parameter2, parameters, xParameterStringLength);
    	    parameter2[xParameterStringLength] = '\0';

    	    amount = atoi(parameter2);
    	    if ((amount > 50) || (amount < 0)){
    	        strncpy( pcWriteBuffer, "invalid parameter. should be 0<=y<50", 1 + strlen("invalid parameter. should be 0<=y<50"));
    	        return pdFALSE;
    	    }
    	    if ((init + amount) > 255){
    	    	amount = 255 - init;
    	    }
    	    for (uint8_t i = init; i <= init + amount; i++){
    	    	sprintf(pcWriteBuffer + strlen(pcWriteBuffer), "mod[%d]*0.5: %f\n\r", i, 0.5*mod[i]);
//    	    	sprintf(pcWriteBuffer + strlen(pcWriteBuffer), "fund phase %d: %f\n\r", i, atan2f(ReIm[2*i+1],ReIm[2*i])*180/M_PI);
    	    	sprintf(pcWriteBuffer + strlen(pcWriteBuffer), "fund phase %d: %f\n\r", i, atan2f(ReIm[2*i+1],ReIm[2*i])*180/3.14);

    	    }


    	}
    }

    return pdFALSE;
}

static BaseType_t prvTaskStatsTexto(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{

  strcpy(pcWriteBuffer, (char *)"Isto é apenas um teste, testando 1 2 3...\r\n");
  return pdFALSE;
}


static const CLI_Command_Definition_t xTasksTexto =
    {
        "texto",
        "\r\ntexto:\r\n Quer testar o texte?\r\n\r\n",
        prvTaskStatsTexto,
        0};

static const CLI_Command_Definition_t xTasksCommand =
{
    "tasks",
    "tasks: show tasks\n\r",
	prvTaskStatsCommand,
    0
};

static const CLI_Command_Definition_t xHarmonicCommand =
{
    "harmonic",
    "\n\r cc or x y. x being initial index and y being the wanted amount\n\r",
	prvHarmonicCommand,
    -1
};

static const CLI_Command_Definition_t xLedCommand =
{
    "led",
    "led:\n\rgreen/red: choose green or red \n\ron: turn selected led on\n\r"
    "off: turn selected led off\n\rtoggle: used with a positive integer parameter to set the toggling interval\n\ror 0 to toggle and stop toggling periodicly",
	prvLedCommand,
    -1
};

char pcOutputString[ MAX_OUTPUT_LENGTH ], pcInputString[ MAX_INPUT_LENGTH ];

int8_t cInputIndex = 0;

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  green_led.timeout = 500;
  green_led.port = GPIOD;
  green_led.pin = GPIO_PIN_13;

  red_led.timeout = 250;
  red_led.port = GPIOD;
  red_led.pin = GPIO_PIN_14;

  xTaskCreate(task_led, "green LED task", 256, &green_led, 1, NULL);
  xTaskCreate(task_led, "red LED task", 256, &red_led, 2, NULL);
  //xTaskCreate(task_usb, "Tarefa USB", 256, NULL, 2, NULL);
  xTaskCreate(task_print, "print task", 512, NULL, 2, NULL);
//  xTaskCreate(task_harmonics, "harmonics task", 512, NULL, 2, NULL);
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD13 PD14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void init_usb_rtos_obj(void);
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  init_usb_rtos_obj();
  //char data[32];

  /* Infinite loop */
	//Peripheral_Descriptor_t xConsole;
	int8_t cInputIndex = 0;

	FreeRTOS_CLIRegisterCommand( &xTasksCommand );
	FreeRTOS_CLIRegisterCommand( &xTasksTexto );
    FreeRTOS_CLIRegisterCommand(&xLedCommand);
//  FreeRTOS_CLIRegisterCommand(&xHarmonicCommand);
    char data[128];
      int amount = 0;
      BaseType_t xMoreDataToFollow;

      for(;;){
    	  amount = CDC_Receivem_FS((uint8_t*)&data, portMAX_DELAY);
    	  for (int i = 0; i<amount; i++){
    		  if (data[i] == '\r'){
    			  queue_print((uint8_t*)"\n\r", 2);
    			  if (cInputIndex != 0){

    				  do{
    					  xMoreDataToFollow = FreeRTOS_CLIProcessCommand(pcInputString, pcOutputString, MAX_OUTPUT_LENGTH);
    					  if (strlen(pcOutputString)){
    						  sprintf(pcOutputString + strlen(pcOutputString), "\n\r");
    						  queue_print((uint8_t*)pcOutputString, strlen(pcOutputString));
    					  	  memset(pcOutputString, 0, MAX_OUTPUT_LENGTH);
    					  }
    				  }while (xMoreDataToFollow != pdFALSE);

    				  cInputIndex = 0;
    				  memset(pcInputString, 0, MAX_INPUT_LENGTH);
    			  }
    		  }
    		  else{
    			  if (data[i] == 127){ //backspace
    				  if (cInputIndex > 0){
    					  cInputIndex--;
    					  pcInputString[cInputIndex] = 0;
    					  queue_print((uint8_t*)&data, amount);
    				  }
    			  }
    			  else{
    				  if (cInputIndex <MAX_INPUT_LENGTH){
    					  pcInputString[cInputIndex] = data[i];
    					  cInputIndex++;
    					  queue_print((uint8_t*)&data, amount);
    				  }
    			  }
    		  }
    	  }
      }

  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
