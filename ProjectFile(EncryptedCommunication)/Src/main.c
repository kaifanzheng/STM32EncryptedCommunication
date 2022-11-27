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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <testFunctionalities.h>
#include "OLEDScreenDriver.h"
#define ARM_MATH_CM4
#include "arm_math.h"
#include "keypadDriver.h"
#include "cryptosystem.h"
#include "stm32l4s5i_iot01_hsensor.h"
#include "stm32l4s5i_iot01_tsensor.h"

#include "../Components/hts221/hts221.c"
#include "stm32l4s5i_iot01_tsensor.h"
#include "stm32l4s5i_iot01_hsensor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define time_between_note 130
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch2;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

OSPI_HandleTypeDef hospi1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart4;

osThreadId screenTaskHandle;
osThreadId pollingTaskHandle;
/* USER CODE BEGIN PV */
uint8_t ringtone[22932]; // All notes in one array, DMA can be performed in one go
uint8_t mainMenuPage = 0;
const uint8_t maxMenuPageNum = 5;
float t;
float h;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_OCTOSPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI3_Init(void);
static void MX_UART4_Init(void);
static void MX_I2C2_Init(void);
void StartScreenTask(void const * argument);
void StartPollingTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float read_temperature(){
	return BSP_TSENSOR_ReadTemp();
}

float read_humidity(){
	return BSP_HSENSOR_ReadHumidity();
}


void generate_ringtone(uint8_t pDest[], uint8_t harmonic, uint8_t note_sizes[]){
	  //================================= GENERATE SINE WAVES FOR RINGTONE =================================
	  float offset = ((2.0/3.0)*256)/2.0; // Add first sine wave and 2nd harmonic
	  float offset_harmonic = ((2.0/3.0)*256)/8.0;
	  int note_sample_length = 22932/4;  // = 5733
	  float input = 0;
	  float step_size;
	  float input_harmonic_1 = 0;
	  float step_size_harmonic_1;
	  // Fill array with values:
	  for(int i = 0; i < 22932; i++){
		  if(i == 0){
			  input = 0;
			  step_size = (2.0*PI)/note_sizes[0];
			  step_size_harmonic_1 = (2.0*PI)/(note_sizes[0] * 2);
		  } else if(i == note_sample_length){
			  input = 0;
			  step_size = (2.0*PI)/note_sizes[1];
			  step_size_harmonic_1 = (2.0*PI)/(note_sizes[1] * 2);
		  } else if(i == note_sample_length * 2){
			  input = 0;
			  step_size = (2.0*PI)/note_sizes[2];
			  step_size_harmonic_1 = (2.0*PI)/(note_sizes[2] * 2);
		  } else if(i == note_sample_length * 3){
			  input = 0;
			  step_size = (2.0*PI)/note_sizes[3];
			  step_size_harmonic_1 = (2.0*PI)/(note_sizes[3] * 2);
		  }
		  pDest[i] = offset + arm_sin_f32(input)*offset;
		  if(harmonic) pDest[i] += offset_harmonic + arm_sin_f32(input_harmonic_1)*offset_harmonic;
		  // pDest[i] += offset_harmonic + arm_sin_f32(input_harmonic_2)*offset_harmonic;
		  input += step_size;
		  input_harmonic_1 += step_size_harmonic_1;
	  }
}

void play_ringtone(){
	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_2, ringtone, 22932, DAC_ALIGN_8B_R);
}

//ui start###################################################################
void clearBufferString(uint8_t *buffer, uint8_t bufferSize){
	for(uint8_t i=0;i<bufferSize;i++){
		buffer[i] = 0;
	}
}
void hardcopy_char(uint8_t *buffer, uint8_t bufferSize, uint8_t *bufferCopy){
	for(uint8_t i =0;i<bufferSize;i++){
		bufferCopy[i] = buffer[i];
	}
}
void displayMainPageOne(){
	printToScreen("main menu");
	printToScreen(" ");
	printToScreen(" ");
	printToScreen("   Wi-Fi option  >");
}
void displayMainPageTwo(){
	printToScreen("main menu");
	printToScreen(" ");
	printToScreen(" ");
	printToScreen("<  send message >");
}
void displayMainPageThree(){
	printToScreen("main menu");
	printToScreen(" ");
	printToScreen(" ");
	printToScreen("< receive message>");
}
void displayMainPageFour(){
	printToScreen("main menu");
	printToScreen(" ");
	printToScreen(" ");
	printToScreen("<  test speaker  >");
}
void displayMainPageFive(){
	printToScreen("main menu");
	printToScreen(" ");
	printToScreen(" ");
	printToScreen("<show temperature ");
}
void changePageMainPage(){
	  clearScreen();
	  if(mainMenuPage == 0){
		  displayMainPageOne();
	  }else if(mainMenuPage == 1){
		  displayMainPageTwo();
	  }else if(mainMenuPage == 2){
		  displayMainPageThree();
	  }else if(mainMenuPage == 3){
		  displayMainPageFour();
	  }else if(mainMenuPage == 4){
		  displayMainPageFive();
	  }
}
void ExitToMain(){
	mainMenuPage = 0;
	clearScreen();
	displayMainPageOne();
}
void wifiOptionPage(){
	printToScreen("Wi-Fi option");
	printToScreen("");
	printToScreen("stop dreaming");
	printToScreen("no wi-fi");
	while(1){
		if(getOneCharFromKeypad() == 'A'){
			ExitToMain();
			break;
		}
		HAL_Delay(100);
		//TODO
	}
}
void sendMessagePage(){
	//inite for buffer senting
	const int sendBufferMaxSize = 15;
	uint8_t sendBuffer[sendBufferMaxSize];
	int BufferLen = 0;
	clearBufferString(sendBuffer,sendBufferMaxSize);

	printToScreen("send Message");
	while(1){
		char keyPadReading = getOneCharFromKeypad();
		if(keyPadReading == 'A'){
			ExitToMain();
			break;
		}
		HAL_Delay(100);
		//
		if(keyPadReading == '#'){
			HAL_UART_Init(&huart4);
			HAL_UART_Transmit(&huart4, sendBuffer,sendBufferMaxSize , 10000);
			clearScreen();
			clearBufferString(sendBuffer,sendBufferMaxSize);
			BufferLen = 0;
			printToScreen("send Message");
		}

		if(keyPadReading != 'n' && BufferLen <= 14 && keyPadReading != '#'){
			sendBuffer[BufferLen] = keyPadReading;
			BufferLen += 1;
			clearScreen();
			printToScreen("send Message");
			printToScreen(" ");
			printToScreen((char *)sendBuffer);
		}

	}
}
void getMessagePage(){
	uint8_t getBuffer[15];
	uint8_t getBufferPrev[15];
	printToScreen("get Message");
	uint8_t quit = 1;
	while(quit){
		HAL_Delay(5);
		//TODO
		hardcopy_char(getBuffer,15,getBufferPrev);
		clearBufferString(getBuffer,15);
		while(getBuffer[0] == 0){
			HAL_UART_Init(&huart4);
			HAL_UART_Receive(&huart4,getBuffer, 15, 5000);
			HAL_Delay(10);
		}
		if(strcmp((char *)getBuffer,(char *)getBufferPrev)!=0){
			clearScreen();
			printToScreen("get Message");
			printToScreen(" ");
			printToScreen((char *)getBuffer);
			printToScreen("press 1 quit");
			printToScreen("press 3 keep");
		}
		while(1){
			if(getOneCharFromKeypad() == '3'){
				clearScreen();
				printToScreen("get Message");
				printToScreen(" ");
				//printToScreen((char *)getBuffer);
				break;
			}else if(getOneCharFromKeypad() == '1'){
				quit = 0;
				break;
			}
			HAL_Delay(50);
		}
	}
}
void testSpeakerPage(){
	printToScreen("testing speaker");
	printToScreen(" ");
	printToScreen("press A exit");
	while(1){
		if(getOneCharFromKeypad() == 'A'){
			ExitToMain();
			break;
		}
		play_ringtone();
		HAL_Delay(100);
	}
}
void getTemperaturePage(){
	float tem =0;
	float hum =0;
	float temp = 0;
	float hump = 0;
	while(1){
		if(getOneCharFromKeypad() == 'A'){
			ExitToMain();
			break;
		}
		HAL_Delay(100);
		temp = tem;
		hump = hum;
		tem = read_temperature();
		hum = read_humidity();
		if(temp != tem || hump != hum){
			clearScreen();
			char temBuf[15];
			char humBuf[15];
			gcvt(tem, 8, temBuf);
			gcvt(hum, 8, humBuf);
			printToScreen("t and h are:");
			printToScreen("");
			printToScreen(temBuf);
			printToScreen( humBuf);

		}
	}
}
void enterPageMainPage(){
	  clearScreen();
	  if(mainMenuPage == 0){
		  wifiOptionPage();
	  }else if(mainMenuPage == 1){
		  sendMessagePage();
	  }else if(mainMenuPage == 2){
		  getMessagePage();
	  }else if(mainMenuPage == 3){
		  testSpeakerPage();
	  }else if(mainMenuPage == 4){
		  getTemperaturePage();
	  }
}
void UILogic(){
	char keypadInputMain = getOneCharFromKeypad();
	if(keypadInputMain == '1'){
		if(mainMenuPage>0){
			mainMenuPage -= 1;
			changePageMainPage();
		}
	}else if(keypadInputMain == '3'){
		if(mainMenuPage<4){
			mainMenuPage += 1;
			changePageMainPage();
		}
	}else if(keypadInputMain == 'A'){
		HAL_Delay(100);
		enterPageMainPage();
	}
}
//test for data transfer----
void testSend(){
	//HAL_UART_Init(&huart4);
	uint8_t buf[5];
	uint8_t counter = 0;
	while(1){
		clearBufferString(buf,5);
		if(counter % 2 == 0){
			sprintf(buf, "hello");
			HAL_UART_Transmit(&huart4, (uint8_t*) buf, (uint16_t) strlen(buf), 10000);
			printToScreen("sent hello");
		}else{
			sprintf(buf, "happy");
			HAL_UART_Transmit(&huart4, (uint8_t*) buf, (uint16_t) strlen(buf), 10000);
			printToScreen("sent happy");
		}
//		sprintf(buf, "hello");
//		HAL_UART_Transmit(&huart4, (uint8_t*) buf, (uint16_t) strlen(buf), 10000);
		counter = (counter+1)%4;
		HAL_Delay(2000);
	}
}
void testGet(){
	//HAL_UART_Init(&huart4);
	uint8_t buf[5];
	while(1){
//		printToScreen("start to get:");
//		printToScreen("");
		clearBufferString(buf,5);
		HAL_UART_Receive(&huart4, (uint8_t*) buf, (uint16_t) 5, 10000);
		printToScreen((char *) buf);
		HAL_Delay(1000);
	}
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
  InitScreen();
  HAL_TIM_Base_Start_IT(&htim2);
  BSP_HSENSOR_Init();
  BSP_TSENSOR_Init();
  HAL_UART_Init(&huart4);

  uint8_t note_sizes[] = {32, 36, 38, 34};
  generate_ringtone(ringtone, 1, note_sizes);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_OCTOSPI1_Init();
  MX_I2C1_Init();
  MX_SPI3_Init();
  MX_UART4_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

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
  /* definition and creation of screenTask */
  osThreadDef(screenTask, StartScreenTask, osPriorityNormal, 0, 128);
  screenTaskHandle = osThreadCreate(osThread(screenTask), NULL);

  /* definition and creation of pollingTask */
  osThreadDef(pollingTask, StartPollingTask, osPriorityNormal, 0, 128);
  pollingTaskHandle = osThreadCreate(osThread(pollingTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  t = read_temperature();
	  h = read_humidity();

	  HAL_Delay(100);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00702991;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10909CEC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief OCTOSPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI1_Init(void)
{

  /* USER CODE BEGIN OCTOSPI1_Init 0 */

  /* USER CODE END OCTOSPI1_Init 0 */

  OSPIM_CfgTypeDef OSPIM_Cfg_Struct = {0};

  /* USER CODE BEGIN OCTOSPI1_Init 1 */

  /* USER CODE END OCTOSPI1_Init 1 */
  /* OCTOSPI1 parameter configuration*/
  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThreshold = 1;
  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_MACRONIX;
  hospi1.Init.DeviceSize = 32;
  hospi1.Init.ChipSelectHighTime = 1;
  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi1.Init.ClockPrescaler = 1;
  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
  hospi1.Init.ChipSelectBoundary = 0;
  hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
  if (HAL_OSPI_Init(&hospi1) != HAL_OK)
  {
    Error_Handler();
  }
  OSPIM_Cfg_Struct.ClkPort = 1;
  OSPIM_Cfg_Struct.NCSPort = 1;
  OSPIM_Cfg_Struct.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
  if (HAL_OSPIM_Config(&hospi1, &OSPIM_Cfg_Struct, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI1_Init 2 */

  /* USER CODE END OCTOSPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1814;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, C3_Pin|C4_Pin|C2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, C1_Pin|GPIO_PIN_12|GPIO_PIN_13|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8|GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : C3_Pin C4_Pin C2_Pin */
  GPIO_InitStruct.Pin = C3_Pin|C4_Pin|C2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : R2_Pin R4_Pin */
  GPIO_InitStruct.Pin = R2_Pin|R4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : R1_Pin R3_Pin */
  GPIO_InitStruct.Pin = R1_Pin|R3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : C1_Pin PB12 PB13 LED_Pin */
  GPIO_InitStruct.Pin = C1_Pin|GPIO_PIN_12|GPIO_PIN_13|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PE8 PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartScreenTask */
/**
  * @brief  Function implementing the screenTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartScreenTask */
void StartScreenTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  //initeScreen and UI
  displayMainPageOne();
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
    UILogic();
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartPollingTask */
/**
* @brief Function implementing the pollingTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPollingTask */
void StartPollingTask(void const * argument)
{
  /* USER CODE BEGIN StartPollingTask */
  /*
   * TASK #2: Polling for the keyboard and message polling
   */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartPollingTask */
}

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
