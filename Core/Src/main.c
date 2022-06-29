/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for BMSv1.0 Hardware Unit Tests
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
#include "fatfs.h"
#include "driverSWLTC6804.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
CAN_HandleTypeDef hcan;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_CAN_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/*File system declerations */
FATFS fs;  // file system
FIL fil; // File
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw;  // File read/write count

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t dummy_timer, dummy_cell_votlages, dummy_pack_voltage, dummy_pack_current, dummy_temperature;
uint8_t char_data[] = "Xanadu BMS v1.0 Unit Test in progress \r\n";

/**** SD card capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

//UART2 parameters
#define BUFFER_SIZE 1024
char buffer[BUFFER_SIZE];  // to store strings..
int i=0; //for general buffer functions
uint8_t uart_rx_data[10];  //  uart receive buffer of 10 bytes
int uart_rx_flag = 0;


/*CAN1 parameters*/
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;
int CAN_data_checkFlag = 0;


//LTC681x parameters
#define CS_PORT GPIOA
#define CS_PIN GPIO_PIN_4
#define SPI1_TIMEOUT 100
#define NoOfCellMonitorsPossibleOnBMS 1
#define cellMonitorICCount 1
uint8_t SPI1_pTxData[8];
uint8_t SPI1_pRxData[8];
float cellModuleVoltages[1][18];
uint32_t cellModuleBalanceResistorEnableMask[NoOfCellMonitorsPossibleOnBMS];
uint32_t cellModuleBalanceResistorEnableMaskTest[NoOfCellMonitorsPossibleOnBMS];


/*******************************************************************************/
int bufsize (char *buf)
{
	int i=0;
	while (*buf++ != '\0') i++;
	return i;
}

void clear_buffer (void)
{
	for (int i=0; i<BUFFER_SIZE; i++) buffer[i] = '\0';
}

void send_uart (char *string)
{
	uint8_t len = strlen (string);
	HAL_UART_Transmit(&huart2, (uint8_t *) string, len, HAL_MAX_DELAY);  // transmit in blocking mode
}

void write_to_csvfile (void)
{

		  dummy_timer += 1;
		  dummy_cell_votlages += 1;
		  dummy_pack_voltage += 11;
		  dummy_pack_current += 1;
		  dummy_temperature += 5;

		  fresult = f_open(&fil, "file3.csv", FA_OPEN_EXISTING | FA_READ | FA_WRITE);
		  /* Move to offset to the end of the file */
		  fresult = f_lseek(&fil, f_size(&fil));
		  sprintf(buffer, "%d,%d,%d,%d,%d\r\n", dummy_timer, dummy_cell_votlages, dummy_pack_voltage, dummy_pack_current, dummy_temperature);
		  fresult = f_write(&fil, buffer, bufsize(buffer), &bw);
		  //send_uart(buffer);
		  f_close (&fil);

		  clear_buffer();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	if(RxHeader.DLC == 2 )
	{
		CAN_data_checkFlag = 1;
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_UART_Receive_IT(&huart2, uart_rx_data, 4);
  uart_rx_flag = 1;

}



void set_time(void)
{
	  RTC_TimeTypeDef sTime = {0};
	  RTC_DateTypeDef sDate = {0};
	  /** Initialize RTC and set the Time and Date
	  */
	  sTime.Hours = 0x0;
	  sTime.Minutes = 0x53;
	  sTime.Seconds = 0x0;
	  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
	  sDate.Month = RTC_MONTH_JUNE;
	  sDate.Date = 0x22;
	  sDate.Year = 0x22;

	  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /* USER CODE BEGIN RTC_Init 2 */
	  HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR1, 0x32F2);

	  /* USER CODE END RTC_Init 2 */
}


void get_time(void)
{
 RTC_DateTypeDef gDate;
 RTC_TimeTypeDef gTime;
/* Get the RTC current Time */
 HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
/* Get the RTC current Date */
 HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);
/* Display time Format: hh:mm:ss */
 sprintf(buffer,"Time is: %02d:%02d:%02d\r\n",gTime.Hours, gTime.Minutes, gTime.Seconds);
 send_uart(buffer);
 clear_buffer();
/* Display date Format: dd-mm-yy */
 sprintf(buffer,"Date is : %02d-%02d-%2d\r\n",gDate.Date, gDate.Month, 2000 + gDate.Year);
 send_uart(buffer);
 clear_buffer();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)  //interrupt callback function for Charger detect
{
    if(GPIO_Pin == GPIO_PIN_5) // If The INT Source Is EXTI5 (PB5 Pin)
    {
    	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5))
    	{
    		sprintf(buffer, "Charger detected\r\n");
    		send_uart(buffer);
    		clear_buffer();
    	}
    	else
    	{
    		sprintf(buffer, "Charger has been disconnected\r\n");
    		send_uart(buffer);
    		clear_buffer();
    	}
    }
}

//Generic function for wake up serial interface of AFE (LTC681x)

void wakeup_idle(uint8_t total_ic) //Number of ICs in the system
{
	for (int i =0; i<total_ic; i++)
	{
		SPI1_pTxData[0] = 0xFF;
		HAL_GPIO_WritePin(CS_PORT, CS_PIN, RESET);
		HAL_SPI_TransmitReceive(&hspi1,SPI1_pTxData, SPI1_pRxData, 8,SPI1_TIMEOUT);//Guarantees the isoSPI will be in ready mode
		HAL_GPIO_WritePin(CS_PORT, CS_PIN, SET);
	}
}

/* Generic wakeup command to wake the LTC681x from sleep state */
void wakeup_sleep(uint8_t total_ic) //Number of ICs in the system
{
	for (int i =0; i<total_ic; i++)
	{
		HAL_GPIO_WritePin(CS_PORT, CS_PIN, RESET);
		HAL_Delay(0.3); // Guarantees the LTC681x will be in standby
		HAL_GPIO_WritePin(CS_PORT, CS_PIN, SET);
		HAL_Delay(0.01);
	}
}

void sd_init(void)
{
	  //mount SD card and check SD card mounting status
	  fresult = f_mount(&fs, "/", 1);
	  	if (fresult != FR_OK)
	  	{
	  		send_uart ("ERROR!!! in mounting SD CARD...\n\n");

	  	}
	  	else
	  	{
	  		send_uart("SD CARD mounted successfully...\r\n");
	  	}

	  	/*************** Card capacity details ********************/

	  	/* Check free space */
	  	f_getfree("", &fre_clust, &pfs);

	  	total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
	  	sprintf (buffer, "SD CARD Total Size: \t%lu\r\n",total);
	  	send_uart(buffer);
	  	clear_buffer();
	  	free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
	  	sprintf (buffer, "SD CARD Free Space: \t%lu\r\n",free_space);
	  	send_uart(buffer);
	  	clear_buffer();



	  	/************* The following operation is using PUTS and GETS *********************/

	  	/* Open file to write/ create a file if it doesn't exist */
	    fresult = f_open(&fil, "file1.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	  	/* Writing text */
	  	f_puts("This data is written to FILE1.txt and it was written using f_puts ", &fil);
	  	/* Close file */
	  	fresult = f_close(&fil);

	  	if (fresult == FR_OK)
	  	{
	  		send_uart ("File1.txt created and the data is written \r\n");
	  	}

	  	/* Open file to read */
	  	fresult = f_open(&fil, "file1.txt", FA_READ);

	  	/* Read string from the file */
	  	f_gets(buffer, f_size(&fil), &fil);

	  	send_uart("File1.txt is opened and it contains the data as shown below\r\n");
	  	send_uart(buffer);
	  	send_uart("\r\n");
	  	/* Close file */
	  	f_close(&fil);
	  	clear_buffer();
	  	/**************** The following operation is using f_write and f_read **************************/

	  	/* Create second file with read write access and open it */
	  	fresult = f_open(&fil, "file2.txt", FA_CREATE_ALWAYS | FA_WRITE);

	  	/* Writing text */
	  	strcpy (buffer, "This is File2.txt, written using f_write and it says SD card unit test for BMS\r\n");

	  	fresult = f_write(&fil, buffer, bufsize(buffer), &bw);
	  	if (fresult == FR_OK)
	  	{
	  		send_uart ("File2.txt created and the data is written \r\n");
	  	}

	  	/* Close file */
	  	f_close(&fil);
	  	// clearing buffer to show that result obtained is from the file
	  	clear_buffer();
	  	/* Open second file to read */
	  	fresult = f_open(&fil, "file2.txt", FA_READ);
	  	if (fresult == FR_OK){
	  		send_uart ("file2.txt is open and the data is shown below\r\n");
	  	}

	  	/* Read data from the file
	  	 * Please see the function details for the arguments */
	  	f_read (&fil, buffer, f_size(&fil), &br);
	  	send_uart(buffer);
	  	send_uart("\r\n");

	  	/* Close file */
	  	f_close(&fil);

	  	clear_buffer();


	  	/*********************UPDATING an existing file ***************************/

	  	/* Open the file with write access */
	  	fresult = f_open(&fil, "file2.txt", FA_OPEN_EXISTING | FA_READ | FA_WRITE);

	  	/* Move to offset to the end of the file */
	  	fresult = f_lseek(&fil, f_size(&fil));

	  	if (fresult == FR_OK)
	  	{
	  		send_uart ("About to update the file2.txt\r\n");
	  	}

	  	/* write the string to the file */
	  	fresult = f_puts("This is updated data and it should be in the end", &fil);
	  	f_close (&fil);
	  	clear_buffer();

	  	/* Open to read the file */
	  	fresult = f_open (&fil, "file2.txt", FA_READ);

	  	/* Read string from the file */
	  	fresult = f_read (&fil, buffer, f_size(&fil), &br);
	  	if (fresult == FR_OK)
	  	{
	  		send_uart ("Below is the data from updated file2.txt\r\n");
	  		send_uart(buffer);
	  		send_uart("\r\n");
	  	}

	  	/* Close file */
	  	f_close(&fil);

	  	clear_buffer();


	  	/*Create csv file to log random data*/
	  	fresult = f_open(&fil, "file3.csv", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	  	/* Writing text */
	  	f_puts("Timer(s), Cell_Voltages, Pack_Voltage, Pack_Current, Temperature\r\n ", &fil);
	  	/* Close file */
	  	fresult = f_close(&fil);
	  	if (fresult == FR_OK)
	  	{
	  		send_uart ("File3.csv created and header is written \r\n");
	  	}


	  	/*************************REMOVING FILES FROM THE DIRECTORY ****************************/

	//  	fresult = f_unlink("/file1.txt");
	//  	if (fresult == FR_OK) send_uart("file1.txt removed successfully...\n");
	//
	//  	fresult = f_unlink("/file2.txt");
	//  	if (fresult == FR_OK) send_uart("file2.txt removed successfully...\n");

	  	/* Unmount SDCARD */
	//  	fresult = f_mount(NULL, "/", 1);
	//  	if (fresult == FR_OK)
	//  	{
	//  		send_uart ("SD CARD UNMOUNTED successfully...\r\n");
	//  	}

}

//void CellMonitorsArrayTranslate(void) {
//	uint8_t individualCellPointer = 0;
//
//  for(uint8_t modulePointer = 0; modulePointer < cellMonitorICCount; modulePointer++) {
//		if((modulePointer+1) % (cellMonitorICCount/noOfParallelModules)==0 && modulePointer != 0){ // If end of series string, use lastICNoOfCells instead of noOfCellsPerModule
//			for(uint8_t modulePointerCell = 0; modulePointerCell < modPowerElectronicsGeneralConfigHandle->lastICNoOfCells; modulePointerCell++) {
//				modPowerElectronicsPackStateHandle->cellVoltagesIndividual[individualCellPointer].cellVoltage = modPowerElectronicsPackStateHandle->cellModuleVoltages[modulePointer][modulePointerCell];
//				modPowerElectronicsPackStateHandle->cellVoltagesIndividual[individualCellPointer].cellNumber = individualCellPointer++;
//			}
//		}else{ // use noOfCellsPerModule as usually
//			for(uint8_t modulePointerCell = 0; modulePointerCell < modPowerElectronicsGeneralConfigHandle->noOfCellsPerModule; modulePointerCell++) {
//				modPowerElectronicsPackStateHandle->cellVoltagesIndividual[individualCellPointer].cellVoltage = modPowerElectronicsPackStateHandle->cellModuleVoltages[modulePointer][modulePointerCell];
//				modPowerElectronicsPackStateHandle->cellVoltagesIndividual[individualCellPointer].cellNumber = individualCellPointer++;
//			}
//		};
//	}
//}


void init_LTC6813(void)
{
	driverLTC6804ConfigStructTypedef configStruct;
	configStruct.GPIO1                    = true;																														// Do not pull down this pin (false = pull down)
	configStruct.GPIO2                    = true;																														//
	configStruct.GPIO3                    = true;																														//
	configStruct.GPIO4                    = true;																														//
	configStruct.GPIO5                    = true;																														//
	configStruct.GPIO6                    = true;																														//
	configStruct.GPIO7                    = true;																														//
	configStruct.GPIO8                    = true;																														//
	configStruct.GPIO9                    = true;																														//
	configStruct.ReferenceON              = true;																														// Reference ON
	configStruct.ADCOption                = true;																											  		// ADC Option register for configuration of over sampling ratio
	configStruct.noOfCells                = 16;			// Number of cells to monitor (that can cause interrupt)
	configStruct.DisChargeEnableMask      = 0x00000000;	// Set enable state of discharge, 1=EnableDischarge, 0=DisableDischarge
	configStruct.DischargeTimout          = 0;		// Discharge timout value / limit
	configStruct.CellUnderVoltageLimit    = 2.80f; // Undervoltage level, cell voltages under this limit will cause interrupt
	configStruct.CellOverVoltageLimit     = 4.20f;

	driverSWLTC6804Init(configStruct, 1, 18, 7,CELL_MON_LTC6813_1);

	for( uint8_t modulePointer = 0; modulePointer < NoOfCellMonitorsPossibleOnBMS; modulePointer++) {
		for(uint8_t cellPointer = 0; cellPointer < 18; cellPointer++)
			cellModuleVoltages[modulePointer][cellPointer] = 0.0f;

		cellModuleBalanceResistorEnableMask[modulePointer] = 0;
		cellModuleBalanceResistorEnableMaskTest[modulePointer] = 0;
	}

}
void unit_test_LTC6813(void)
{
	driverSWLTC6804ResetCellVoltageRegisters();
	driverSWLTC6804StartCellVoltageConversion(MD_FILTERED,DCP_DISABLED,CELL_CH_ALL);
	//driverSWLTC6804StartCellAndAuxVoltageConversion(MD_FILTERED, DCP_DISABLED);
	//HAL_Delay(300);
	if(driverSWLTC6804ReadCellVoltagesArray(cellModuleVoltages))
	{
		sprintf(buffer,"C1:%f,C2:%f,C3:%f,C4:%f,C5:%f,C6:%f,C7:%f,C8:%f,C9:%f,C10:%f,C11:%f,C12:%f,C13:%f,C14:%f,C15:%f,C16:%f,C17:%f,C18:%f,\r\n",
				cellModuleVoltages[0][1],cellModuleVoltages[0][2],cellModuleVoltages[0][3],cellModuleVoltages[0][4],cellModuleVoltages[0][5],cellModuleVoltages[0][6],
				cellModuleVoltages[0][7],cellModuleVoltages[0][8],cellModuleVoltages[0][9],cellModuleVoltages[0][10],cellModuleVoltages[0][11],cellModuleVoltages[0][12],
				cellModuleVoltages[0][13],cellModuleVoltages[0][14],cellModuleVoltages[0][15],cellModuleVoltages[0][16],cellModuleVoltages[0][17],cellModuleVoltages[0][18]);
		send_uart(buffer);
		clear_buffer();
	}
}
/*******************************************************************************/
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
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_CAN_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  //char buf[100];
  //HAL_Delay(250);

  sprintf(buffer, "Xanadu BMS v1.0 Unit Test in Progress\r\n");
  send_uart(buffer);
  clear_buffer();

  if(HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR1) != 0x32F2)
  	  {
	  	  set_time(); //set RTC init value
  	  }

  sd_init();

  /*CAN Initializations*/
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); //using FIFO0 for RX callback reception
  TxHeader.DLC = 2; //data
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.StdId = 0x446;  //id
  //populate data to Txdata bytes
  TxData[0] = 11;
  TxData[1] = 100;
  //send CAN message // TO DO:check CAN message reception on BluePill
  //HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);

  /*UART2 Interrupt*/
  HAL_UART_Receive_IT (&huart2, uart_rx_data, 4); //set interrupt for uart rx

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, SET); //turn ON precharge relay
  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, RESET); //turn OFF precharge relay
  HAL_Delay(250);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, SET); //turn ON HV+ contactor

  	//TO DO:add LTC6811 library files/use driverSWLTC6804 functions
  //wakeup_sleep(1);
  //wakeup_idle(1);
  init_LTC6813();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //HAL_UART_Receive (&huart2, Rx_data, 4, 1000);
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3); //toggle status LED
	  write_to_csvfile();
	  HAL_Delay(250);
	  //send CAN message // TO DO:check CAN message reception on BluePill
	  HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);

	  //wakeup_idle(1);
	  unit_test_LTC6813();

	  if(CAN_data_checkFlag) //check if CAN RX flag is set in HAL_CAN_RxFifo0MsgPendingCallback
	  {
		  sprintf(buffer, "CAN Message values received is:%d, %d\r\n", RxData[0], RxData[1]);
		  send_uart(buffer);
		  clear_buffer();
		  CAN_data_checkFlag = 0;
		  //HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
	  }

	  if(uart_rx_flag)
	  {
		  sprintf(buffer, "RX Message is: %c, %c, %c\r\n", uart_rx_data[0], uart_rx_data[1], uart_rx_data[2]);
		  send_uart(buffer);
		  clear_buffer();
		  uart_rx_flag = 0;

	  }

	  get_time();  //print RTC
	  HAL_Delay(250);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 18;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  //CAN filter settings
  CAN_FilterTypeDef canfilterConfig;

  canfilterConfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterConfig.FilterBank = 11;
  canfilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterConfig.FilterIdHigh = 0x103<<5;
  canfilterConfig.FilterIdLow = 0;
  canfilterConfig.FilterMaskIdHigh = 0x103<<5;
  canfilterConfig.FilterMaskIdLow = 0x0000;
  canfilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterConfig.SlaveStartFilterBank = 0;

  HAL_CAN_ConfigFilter(&hcan, &canfilterConfig);

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x23;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
  sDate.Month = RTC_MONTH_JUNE;
  sDate.Date = 0x22;
  sDate.Year = 0x22;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
	  HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR1, 0x32F2);

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB12 PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

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

