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
#include "stdio.h"

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

int bufsize(char *buf);
void clear_buffer(void);
void send_uart (char *string); //debug print statements function prototype
void sd_init(void); //init and mount SD card, create files and headers
void write_to_csvfile (void); //function to log to csv file on SD card
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan); //Set CAN RX interrupt handler
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart); //set UART interrupt handler
void set_time(void); //RTC related set initial time and date
void get_time(void); //RTC related get real time data
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin); //function for Charger Detect Interrupt
void wakeup_idle(uint8_t total_ic); //Generic function to wake up serial interface of AFE from idle state(LTC681x)
void wakeup_sleep(uint8_t total_ic); //Generic function to wake up serial interface of AFE from sleep state (LTC681x)
void CellMonitorsArrayTranslate(void);
void AuxMonitorsArrayTranslate(void);
void calculateMaxandMinCellVoltages(void);
void cellBalancingTask(void);
void cellBalancingUnitTest(void);
void init_cell_asic_structure(uint8_t total_ic, cell_asic *ic);
void init_LTC6813(void);
void unit_test_LTC6813(void);
void zeroCurrentCalibration(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*File system declerations */
FATFS fs;  // file system
FIL fil; // File
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw;  // File read/write count


uint8_t dummy_timer, dummy_cell_votlages, dummy_pack_voltage;
float dummy_pack_current, dummy_temperature;
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

#define noOfTotalCells 18
#define NoOfCellMonitorsPossibleOnBMS 1
#define cellMonitorICCount 1

#define noOfTempSensorPerModule 10
#define NTCnominalResistance 10000
#define NTCbetaFactor 3435
#define NTCseriesResistor 10000

#define cellBalanceUpdateTime 4*1000
#define cellBalanceThreshold 0.010f
#define noOfCellsSeries 18
#define noOfParallelModules 1
#define noOfCellsPerModule 18

#define ext_LTC681x_lib 0
#define zeroCurrentCalibrationTime 50

bool cellBalancingEnable = true;

uint8_t SPI1_pTxData[8];
uint8_t SPI1_pRxData[8];
float cellModuleVoltages[1][noOfTotalCells];
float auxModuleVoltages[1][9];
float packCurrentVoltage[1][12]; //ADC voltage value for GPIO1
float packCurrentVREF[1][12]; //ADC voltage value for GPIO2
float zeroCurrentVoltage[1][12]; //zero current ADC voltage value for GPIO1
float zeroCurrentVREF[1][12]; //zero current ADC voltage value for GPIO2
uint32_t cellModuleBalanceResistorEnableMask[NoOfCellMonitorsPossibleOnBMS];
uint32_t cellModuleBalanceResistorEnableMaskTest[NoOfCellMonitorsPossibleOnBMS];
uint32_t CellBalanceUpdateLastTick;
cellMonitorCellsTypeDef cellVoltagesIndividual[noOfTotalCells]; //18:Total no of cells possible
auxMonitorTypeDef auxVoltagesIndividual[9];
cell_asic BMS_IC[cellMonitorICCount];
float packVoltage, cellVoltageHigh, cellVoltageLow, maxImbalanceVoltage;
int32_t packCurrent, currentOffset;
uint8_t rxConfig_A[1][8], rxConfig_B[1][8];

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

		  //dummy_timer += 1;
		  //dummy_cell_votlages += 1;
		  //dummy_pack_voltage += 11;
		  //dummy_pack_current += 0.1;
		  //dummy_temperature += 1.0;

		  fresult = f_open(&fil, "file3.csv", FA_OPEN_EXISTING | FA_READ | FA_WRITE);
		  /* Move to offset to the end of the file */
		  fresult = f_lseek(&fil, f_size(&fil));
		  sprintf(buffer, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%ld,%.3f,%.3f,%.3f,%.3f \r\n",
				  	(HAL_GetTick()/ 1000.0),cellModuleVoltages[0][0],cellModuleVoltages[0][1],cellModuleVoltages[0][2],cellModuleVoltages[0][3],cellModuleVoltages[0][4],cellModuleVoltages[0][5],cellModuleVoltages[0][6],
					cellModuleVoltages[0][7],cellModuleVoltages[0][8],cellModuleVoltages[0][9],cellModuleVoltages[0][10],cellModuleVoltages[0][11],cellModuleVoltages[0][12],
					cellModuleVoltages[0][13],cellModuleVoltages[0][14],cellModuleVoltages[0][15],cellModuleVoltages[0][16],cellModuleVoltages[0][17], packVoltage, packCurrent ,auxVoltagesIndividual[4].auxVoltage, auxVoltagesIndividual[5].auxVoltage, cellVoltageHigh, cellVoltageLow );
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
	  	f_puts("Timer(s), Cell_Voltage_1,Cell_Voltage_2,Cell_Voltage_3,Cell_Voltage_4,Cell_Voltage_5,Cell_Voltage_6,Cell_Voltage_7,Cell_Voltage_8,Cell_Voltage_9,Cell_Voltage_10,Cell_Voltage_11,Cell_Voltage_12,Cell_Voltage_13,Cell_Voltage_14,Cell_Voltage_15,Cell_Voltage_16,Cell_Voltage_17,Cell_Voltage_18, Pack_Voltage, Pack_Current, Temperature_3, Temperature_4, Max_cell_voltage, Min_cell_voltage \r\n ", &fil);
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

void CellMonitorsArrayTranslate(void)
{
	uint8_t individualCellPointer = 0;

  for(uint8_t modulePointer = 0; modulePointer < cellMonitorICCount; modulePointer++) {
		if((modulePointer+1) % (cellMonitorICCount/1)==0 && modulePointer != 0)
		{ // If end of series string, use lastICNoOfCells instead of noOfCellsPerModule
			for(uint8_t modulePointerCell = 0; modulePointerCell < noOfTotalCells; modulePointerCell++)
			{
				cellVoltagesIndividual[individualCellPointer].cellVoltage = cellModuleVoltages[modulePointer][modulePointerCell];
				cellVoltagesIndividual[individualCellPointer].cellNumber = individualCellPointer++;
			}
		}
		else
		{ // use noOfCellsPerModule as usually
			for(uint8_t modulePointerCell = 0; modulePointerCell < noOfTotalCells; modulePointerCell++) {
				cellVoltagesIndividual[individualCellPointer].cellVoltage = cellModuleVoltages[modulePointer][modulePointerCell];
				cellVoltagesIndividual[individualCellPointer].cellNumber = individualCellPointer++;
			}
		};
	}
}

void AuxMonitorsArrayTranslate(void) {
	uint8_t individualAuxPointer = 0;

  for(uint8_t modulePointer = 0; modulePointer < cellMonitorICCount; modulePointer++) {
	  for(uint8_t modulePointerAux = 0;modulePointerAux < noOfTempSensorPerModule; modulePointerAux++) {
			if(modulePointerAux < 5)
			{
				auxVoltagesIndividual[individualAuxPointer].auxVoltage = auxModuleVoltages[modulePointer][modulePointerAux];
				auxVoltagesIndividual[individualAuxPointer].auxNumber = individualAuxPointer++;
			}
			else
			{ // when above 5, remove reference voltage measurement from Aux register group B : AVBR4 & AVBR5 for LTC6812 & LTC6813
				auxVoltagesIndividual[individualAuxPointer].auxVoltage = auxModuleVoltages[modulePointer][modulePointerAux+1];
				auxVoltagesIndividual[individualAuxPointer].auxNumber = individualAuxPointer++;
			}
		}
	}
}

void calculateMaxandMinCellVoltages(void)
{
	cellVoltageHigh = 0.0f;
	cellVoltageLow = 10.0f;
	for(uint8_t cellPointer = 0; cellPointer < noOfTotalCells; cellPointer++)
	{
		if(cellVoltagesIndividual[cellPointer].cellVoltage > cellVoltageHigh)
		{
			cellVoltageHigh = cellVoltagesIndividual[cellPointer].cellVoltage;
		}
		if(cellVoltagesIndividual[cellPointer].cellVoltage < cellVoltageLow && cellVoltagesIndividual[cellPointer].cellVoltage > 0.5f)
		{
			cellVoltageLow = cellVoltagesIndividual[cellPointer].cellVoltage;
		}
	}
	maxImbalanceVoltage = cellVoltageHigh - cellVoltageLow;
	sprintf(buffer, "Max Cell Voltage = %.3f, Min Cell Voltage = %.3f , Max Imbalance Voltage = %.3f\r\n",cellVoltageHigh, cellVoltageLow, maxImbalanceVoltage);
	send_uart(buffer);
	clear_buffer();

}

void cellBalancingTask(void)
{
	static uint32_t delayTimeHolder = 100;
	static bool     delaytoggle = false;
	uint8_t modulePointer = 0;
	uint8_t cellInMaskPointer = 0;
	uint8_t seriesCount = 0;
	uint8_t moduleCount = 0;

	if(modDelayTick1ms(&CellBalanceUpdateLastTick,delayTimeHolder))
	{
		delaytoggle ^= true;
		delayTimeHolder = delaytoggle ? cellBalanceUpdateTime : 200;

		if(delaytoggle)
		{

			for(uint8_t cellPointer = 0; cellPointer< noOfTotalCells ; cellPointer += 2)
			{
				if(cellVoltagesIndividual[cellPointer].cellVoltage > (cellVoltageLow + cellBalanceThreshold) && cellBalancingEnable == true)
				{
					cellVoltagesIndividual[cellPointer].cellBleedActive = true;
				}
				else
				{
					cellVoltagesIndividual[cellPointer].cellBleedActive = false;
				}
			}
			if(cellBalancingEnable == false)
			{
				for(uint8_t cellPointer = 0; cellPointer< noOfTotalCells ; cellPointer++)
					cellVoltagesIndividual[cellPointer].cellBleedActive = false;

			}
		}
	}

	// Clear array
	for(uint8_t moduleClearPointer = 0; moduleClearPointer < NoOfCellMonitorsPossibleOnBMS; moduleClearPointer++)
	{
		cellModuleBalanceResistorEnableMask[moduleClearPointer] = 0;
	}
	for(uint8_t cellPointer = 0; cellPointer < noOfCellsSeries*noOfParallelModules; cellPointer++)
	{
		seriesCount = cellPointer/noOfCellsSeries;
		moduleCount = seriesCount*(cellMonitorICCount/noOfParallelModules);
		modulePointer = moduleCount + (cellPointer % noOfCellsSeries)/noOfCellsPerModule;
		cellInMaskPointer = (cellPointer - (seriesCount*noOfCellsSeries)) % noOfCellsPerModule;

		if(cellVoltagesIndividual[cellPointer].cellBleedActive)
			cellModuleBalanceResistorEnableMask[modulePointer] |= (1 << cellInMaskPointer);
		else
			cellModuleBalanceResistorEnableMask[modulePointer] &= ~(1 << cellInMaskPointer);
	}

	driverSWLTC6804EnableBalanceResistorsArray(cellModuleBalanceResistorEnableMask, CELL_MON_LTC6811_1);
}



void init_cell_asic_structure(uint8_t total_ic, cell_asic *ic)
{
	/*************************************************************************
	 Set configuration register. Refer to the data sheet
	**************************************************************************/
	bool REFON = true; //!< Reference Powered Up Bit
	bool ADCOPT = false; //!< ADC Mode option bit
	bool GPIOBITS_A[5] = {false,false,true,true,true}; //!< GPIO Pin Control // Gpio 1,2,3,4,5
	bool GPIOBITS_B[4] = {false,false,false,false}; //!< GPIO Pin Control // Gpio 6,7,8,9
	uint16_t UV= 42000; //!< Under voltage Comparison Voltage
	uint16_t OV= 28000; //!< Over voltage Comparison Voltage
	bool DCCBITS_A[12] = {false,false,false,false,false,false,false,false,false,false,false,false}; //!< Discharge cell switch //Dcc 1,2,3,4,5,6,7,8,9,10,11,12
	bool DCCBITS_B[7]= {false,false,false,false,false,false,false}; //!< Discharge cell switch //Dcc 0,13,14,15
	bool DCTOBITS[4] = {true,false,true,false}; //!< Discharge time value //Dcto 0,1,2,3  // Programed for 4 min
	/*Ensure that Dcto bits are set according to the required discharge time. Refer to the data sheet */
	bool FDRF = false; //!< Force Digital Redundancy Failure Bit
	bool DTMEN = true; //!< Enable Discharge Timer Monitor
	bool PSBITS[2]= {false,false}; //!< Digital Redundancy Path Selection//ps-0,1

	for (uint8_t current_ic = 0; current_ic<total_ic;current_ic++)
	{
		for (int j =0; j<6; j++)
		{
		  ic[current_ic].config.tx_data[j] = 0;
		  ic[current_ic].configb.tx_data[j] = 0;
		}

		LTC6813_set_cfgr(current_ic,BMS_IC,REFON,ADCOPT,GPIOBITS_A,DCCBITS_A, DCTOBITS, UV, OV);
		LTC6813_set_cfgrb(current_ic,BMS_IC,FDRF,DTMEN,PSBITS,GPIOBITS_B,DCCBITS_B);


		ic[current_ic].crc_count.pec_count = 0;
		ic[current_ic].crc_count.cfgr_pec = 0;
		for (int i=0; i<6; i++)
		{
			ic[current_ic].crc_count.cell_pec[i]=0;

		}
		for (int i=0; i<4; i++)
		{
			ic[current_ic].crc_count.aux_pec[i]=0;
		}
		for (int i=0; i<2; i++)
		{
			ic[current_ic].crc_count.stat_pec[i]=0;
		}

        ic[current_ic].ic_reg.cell_channels=18;
        ic[current_ic].ic_reg.stat_channels=4;
        ic[current_ic].ic_reg.aux_channels=9;
        ic[current_ic].ic_reg.num_cv_reg=6;
        ic[current_ic].ic_reg.num_gpio_reg=4;
        ic[current_ic].ic_reg.num_stat_reg=2;
	}

}


void cellBalancingUnitTest(void)
{
	if(cellBalancingEnable)
	{
		driverSWLTC6804WakeIC();
	    LTC681x_clear_discharge(cellMonitorICCount,BMS_IC);
	    LTC681x_wrcfg(cellMonitorICCount,BMS_IC);
	    LTC681x_wrcfgb(cellMonitorICCount,BMS_IC);

	    //Enable S1, S7 and S13 and measure C1, C7 and C13 with DCP enabled
		for(uint8_t S_pin = 1 ; S_pin < noOfTotalCells+1; S_pin += 6)
		{
			LTC6813_set_discharge(S_pin,cellMonitorICCount,BMS_IC);
		}
		LTC681x_wrcfg(cellMonitorICCount,BMS_IC);
		LTC681x_wrcfgb(cellMonitorICCount,BMS_IC);
		driverSWLTC6804ResetCellVoltageRegisters();
		driverSWLTC6804StartCellVoltageConversion(MD_FILTERED,DCP_ENABLED,CELL_CH_ALL);
		if(driverSWLTC6804ReadCellVoltagesArray(cellModuleVoltages))
		{
			CellMonitorsArrayTranslate();
			sprintf(buffer,"Enabled S1, S7 and S13; C1:%f,C2:%f,C3:%f,C4:%f,C5:%f,C6:%f,C7:%f,C8:%f,C9:%f,C10:%f,C11:%f,C12:%f,C13:%f,C14:%f,C15:%f,C16:%f,C17:%f,C18:%f,\r\n",
					cellModuleVoltages[0][0],cellModuleVoltages[0][1],cellModuleVoltages[0][2],cellModuleVoltages[0][3],cellModuleVoltages[0][4],cellModuleVoltages[0][5],cellModuleVoltages[0][6],
					cellModuleVoltages[0][7],cellModuleVoltages[0][8],cellModuleVoltages[0][9],cellModuleVoltages[0][10],cellModuleVoltages[0][11],cellModuleVoltages[0][12],
					cellModuleVoltages[0][13],cellModuleVoltages[0][14],cellModuleVoltages[0][15],cellModuleVoltages[0][16],cellModuleVoltages[0][17]);
			send_uart(buffer);
			clear_buffer();
		}
		LTC681x_clear_discharge(cellMonitorICCount,BMS_IC);
	    LTC681x_wrcfg(cellMonitorICCount,BMS_IC);
	    LTC681x_wrcfgb(cellMonitorICCount,BMS_IC);

	    //Enable S2, S8 and S14 and measure C2, C8 and C14 with DCP enabled
		for(uint8_t S_pin = 2 ; S_pin < noOfTotalCells+1; S_pin += 6)
		{
			LTC6813_set_discharge(S_pin,cellMonitorICCount,BMS_IC);
		}
		LTC681x_wrcfg(cellMonitorICCount,BMS_IC);
		LTC681x_wrcfgb(cellMonitorICCount,BMS_IC);
		driverSWLTC6804ResetCellVoltageRegisters();
		driverSWLTC6804StartCellVoltageConversion(MD_FILTERED,DCP_ENABLED,CELL_CH_ALL);
		if(driverSWLTC6804ReadCellVoltagesArray(cellModuleVoltages))
		{
			CellMonitorsArrayTranslate();
			sprintf(buffer,"Enabled S2, S8 and S14; C1:%f,C2:%f,C3:%f,C4:%f,C5:%f,C6:%f,C7:%f,C8:%f,C9:%f,C10:%f,C11:%f,C12:%f,C13:%f,C14:%f,C15:%f,C16:%f,C17:%f,C18:%f,\r\n",
					cellModuleVoltages[0][0],cellModuleVoltages[0][1],cellModuleVoltages[0][2],cellModuleVoltages[0][3],cellModuleVoltages[0][4],cellModuleVoltages[0][5],cellModuleVoltages[0][6],
					cellModuleVoltages[0][7],cellModuleVoltages[0][8],cellModuleVoltages[0][9],cellModuleVoltages[0][10],cellModuleVoltages[0][11],cellModuleVoltages[0][12],
					cellModuleVoltages[0][13],cellModuleVoltages[0][14],cellModuleVoltages[0][15],cellModuleVoltages[0][16],cellModuleVoltages[0][17]);
			send_uart(buffer);
			clear_buffer();
		}
		LTC681x_clear_discharge(cellMonitorICCount,BMS_IC);
	    LTC681x_wrcfg(cellMonitorICCount,BMS_IC);
	    LTC681x_wrcfgb(cellMonitorICCount,BMS_IC);

	    //Enable S3, S9 and S15 and measure C3, C9 and C15 with DCP enabled
		for(uint8_t S_pin = 3 ; S_pin < noOfTotalCells+1; S_pin += 6)
		{
			LTC6813_set_discharge(S_pin,cellMonitorICCount,BMS_IC);
		}
		LTC681x_wrcfg(cellMonitorICCount,BMS_IC);
		LTC681x_wrcfgb(cellMonitorICCount,BMS_IC);
		driverSWLTC6804ResetCellVoltageRegisters();
		driverSWLTC6804StartCellVoltageConversion(MD_FILTERED,DCP_ENABLED,CELL_CH_ALL);
		if(driverSWLTC6804ReadCellVoltagesArray(cellModuleVoltages))
		{
			CellMonitorsArrayTranslate();
			sprintf(buffer,"Enabled S3, S9 and S15; C1:%f,C2:%f,C3:%f,C4:%f,C5:%f,C6:%f,C7:%f,C8:%f,C9:%f,C10:%f,C11:%f,C12:%f,C13:%f,C14:%f,C15:%f,C16:%f,C17:%f,C18:%f,\r\n",
					cellModuleVoltages[0][0],cellModuleVoltages[0][1],cellModuleVoltages[0][2],cellModuleVoltages[0][3],cellModuleVoltages[0][4],cellModuleVoltages[0][5],cellModuleVoltages[0][6],
					cellModuleVoltages[0][7],cellModuleVoltages[0][8],cellModuleVoltages[0][9],cellModuleVoltages[0][10],cellModuleVoltages[0][11],cellModuleVoltages[0][12],
					cellModuleVoltages[0][13],cellModuleVoltages[0][14],cellModuleVoltages[0][15],cellModuleVoltages[0][16],cellModuleVoltages[0][17]);
			send_uart(buffer);
			clear_buffer();
		}
		LTC681x_clear_discharge(cellMonitorICCount,BMS_IC);
	    LTC681x_wrcfg(cellMonitorICCount,BMS_IC);
	    LTC681x_wrcfgb(cellMonitorICCount,BMS_IC);


	    //Enable S4, S10 and S16 and measure C4, C10 and C16 with DCP enabled
		for(uint8_t S_pin = 4 ; S_pin < noOfTotalCells+1; S_pin += 6)
		{
			LTC6813_set_discharge(S_pin,cellMonitorICCount,BMS_IC);
		}
		LTC681x_wrcfg(cellMonitorICCount,BMS_IC);
		LTC681x_wrcfgb(cellMonitorICCount,BMS_IC);
		driverSWLTC6804ResetCellVoltageRegisters();
		driverSWLTC6804StartCellVoltageConversion(MD_FILTERED,DCP_ENABLED,CELL_CH_ALL);
		if(driverSWLTC6804ReadCellVoltagesArray(cellModuleVoltages))
		{
			CellMonitorsArrayTranslate();
			sprintf(buffer,"Enabled S4, S10 and S16; C1:%f,C2:%f,C3:%f,C4:%f,C5:%f,C6:%f,C7:%f,C8:%f,C9:%f,C10:%f,C11:%f,C12:%f,C13:%f,C14:%f,C15:%f,C16:%f,C17:%f,C18:%f,\r\n",
					cellModuleVoltages[0][0],cellModuleVoltages[0][1],cellModuleVoltages[0][2],cellModuleVoltages[0][3],cellModuleVoltages[0][4],cellModuleVoltages[0][5],cellModuleVoltages[0][6],
					cellModuleVoltages[0][7],cellModuleVoltages[0][8],cellModuleVoltages[0][9],cellModuleVoltages[0][10],cellModuleVoltages[0][11],cellModuleVoltages[0][12],
					cellModuleVoltages[0][13],cellModuleVoltages[0][14],cellModuleVoltages[0][15],cellModuleVoltages[0][16],cellModuleVoltages[0][17]);
			send_uart(buffer);
			clear_buffer();
		}
		LTC681x_clear_discharge(cellMonitorICCount,BMS_IC);
	    LTC681x_wrcfg(cellMonitorICCount,BMS_IC);
	    LTC681x_wrcfgb(cellMonitorICCount,BMS_IC);


	    //Enable S5, S11 and S17 and measure C5, C11 and C17 with DCP enabled
		for(uint8_t S_pin = 5 ; S_pin < noOfTotalCells+1; S_pin += 6)
		{
			LTC6813_set_discharge(S_pin,cellMonitorICCount,BMS_IC);
		}
		LTC681x_wrcfg(cellMonitorICCount,BMS_IC);
		LTC681x_wrcfgb(cellMonitorICCount,BMS_IC);
		driverSWLTC6804ResetCellVoltageRegisters();
		driverSWLTC6804StartCellVoltageConversion(MD_FILTERED,DCP_ENABLED,CELL_CH_ALL);
		if(driverSWLTC6804ReadCellVoltagesArray(cellModuleVoltages))
		{
			CellMonitorsArrayTranslate();
			sprintf(buffer,"Enabled S5, S11 and S17; C1:%f,C2:%f,C3:%f,C4:%f,C5:%f,C6:%f,C7:%f,C8:%f,C9:%f,C10:%f,C11:%f,C12:%f,C13:%f,C14:%f,C15:%f,C16:%f,C17:%f,C18:%f,\r\n",
						cellModuleVoltages[0][0],cellModuleVoltages[0][1],cellModuleVoltages[0][2],cellModuleVoltages[0][3],cellModuleVoltages[0][4],cellModuleVoltages[0][5],cellModuleVoltages[0][6],
						cellModuleVoltages[0][7],cellModuleVoltages[0][8],cellModuleVoltages[0][9],cellModuleVoltages[0][10],cellModuleVoltages[0][11],cellModuleVoltages[0][12],
						cellModuleVoltages[0][13],cellModuleVoltages[0][14],cellModuleVoltages[0][15],cellModuleVoltages[0][16],cellModuleVoltages[0][17]);
			send_uart(buffer);
			clear_buffer();
		}
		LTC681x_clear_discharge(cellMonitorICCount,BMS_IC);
	    LTC681x_wrcfg(cellMonitorICCount,BMS_IC);
	    LTC681x_wrcfgb(cellMonitorICCount,BMS_IC);

	    //Enable S6, S12 and S18 and measure C6, C12 and C18 with DCP enabled
		for(uint8_t S_pin = 6 ; S_pin < noOfTotalCells+1; S_pin += 6)
		{
			LTC6813_set_discharge(S_pin,cellMonitorICCount,BMS_IC);
		}
		LTC681x_wrcfg(cellMonitorICCount,BMS_IC);
		LTC681x_wrcfgb(cellMonitorICCount,BMS_IC);
		driverSWLTC6804ResetCellVoltageRegisters();
		driverSWLTC6804StartCellVoltageConversion(MD_FILTERED,DCP_ENABLED,CELL_CH_ALL);
		if(driverSWLTC6804ReadCellVoltagesArray(cellModuleVoltages))
		{
			CellMonitorsArrayTranslate();
			sprintf(buffer,"Enabled S6, S12 and S18; C1:%f,C2:%f,C3:%f,C4:%f,C5:%f,C6:%f,C7:%f,C8:%f,C9:%f,C10:%f,C11:%f,C12:%f,C13:%f,C14:%f,C15:%f,C16:%f,C17:%f,C18:%f,\r\n",
						cellModuleVoltages[0][0],cellModuleVoltages[0][1],cellModuleVoltages[0][2],cellModuleVoltages[0][3],cellModuleVoltages[0][4],cellModuleVoltages[0][5],cellModuleVoltages[0][6],
						cellModuleVoltages[0][7],cellModuleVoltages[0][8],cellModuleVoltages[0][9],cellModuleVoltages[0][10],cellModuleVoltages[0][11],cellModuleVoltages[0][12],
						cellModuleVoltages[0][13],cellModuleVoltages[0][14],cellModuleVoltages[0][15],cellModuleVoltages[0][16],cellModuleVoltages[0][17]);
			send_uart(buffer);
			clear_buffer();
		}
		LTC681x_clear_discharge(cellMonitorICCount,BMS_IC);
	    LTC681x_wrcfg(cellMonitorICCount,BMS_IC);
	    LTC681x_wrcfgb(cellMonitorICCount,BMS_IC);
	}
	else
	{
	      LTC681x_clear_discharge(cellMonitorICCount,BMS_IC);
	      LTC681x_wrcfg(cellMonitorICCount,BMS_IC);
	      LTC681x_wrcfgb(cellMonitorICCount,BMS_IC);
	}
}

void zeroCurrentCalibration(void)
{
	driverSWLTC6804ReadPackCurrent(zeroCurrentVoltage);
	driverSWLTC6804ReadVREFvoltage(zeroCurrentVREF);
	if(driverSWLTC6804ReadPackCurrent(zeroCurrentVoltage) && driverSWLTC6804ReadVREFvoltage(zeroCurrentVREF))
	{
		for(int i = 0; i <zeroCurrentCalibrationTime; i++)
		{
			currentOffset = ((zeroCurrentVoltage[0][0]*1000.0f) - (zeroCurrentVREF[0][1]*1000.0f))/0.00625; //mA
			driverSWLTC6804ResetAuxRegisters();
			driverSWLTC6804StartCellAndAuxVoltageConversion(MD_FILTERED, DCP_DISABLED);
			sprintf(buffer, "Zero Current calibration in progress, Zero Current(mA) = %ld \r\n", currentOffset);
			send_uart(buffer);
			clear_buffer();
			HAL_Delay(100);
			driverSWLTC6804ReadPackCurrent(zeroCurrentVoltage);
			driverSWLTC6804ReadVREFvoltage(zeroCurrentVREF);
			if((-300 < currentOffset) && (currentOffset < 300))
			{
				sprintf(buffer, "Current Offset(mA) : %ld \r\n", currentOffset);
				send_uart(buffer);
				clear_buffer();
				break;
			}
		}
	}
}

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
	configStruct.noOfCells                = 18;			// Number of cells to monitor (that can cause interrupt)
	configStruct.DisChargeEnableMask      = 0x00000000;	// Set enable state of discharge, 1=EnableDischarge, 0=DisableDischarge
	configStruct.DischargeTimout          = 0;		// Discharge timout value / limit
	configStruct.CellUnderVoltageLimit    = 2.80f; // Undervoltage level, cell voltages under this limit will cause interrupt
	configStruct.CellOverVoltageLimit     = 4.20f;

	driverSWLTC6804Init(configStruct, NoOfCellMonitorsPossibleOnBMS, noOfTotalCells, noOfTempSensorPerModule,CELL_MON_LTC6813_1);

	for( uint8_t modulePointer = 0; modulePointer < NoOfCellMonitorsPossibleOnBMS; modulePointer++)
	{
		for(uint8_t cellPointer = 0; cellPointer < noOfTotalCells; cellPointer++)
			cellModuleVoltages[modulePointer][cellPointer] = 0.0f;

		cellModuleBalanceResistorEnableMask[modulePointer] = 0;
		cellModuleBalanceResistorEnableMaskTest[modulePointer] = 0;
	}
	for( uint8_t modulePointer = 0; modulePointer < NoOfCellMonitorsPossibleOnBMS; modulePointer++)
	{
		for(uint8_t auxPointer = 0; auxPointer < noOfTempSensorPerModule; auxPointer++)
			auxModuleVoltages[modulePointer][auxPointer] = 0.0f;
	}

	//driverSWLTC6804StartCellVoltageConversion(MD_FILTERED,DCP_DISABLED,CELL_CH_ALL);
	//driverSWLTC6804StartCellVoltageConversion(MD_FILTERED,DCP_ENABLED,CELL_CH_ALL);
	driverSWLTC6804ResetCellVoltageRegisters();
	driverSWLTC6804ResetAuxRegisters();
	//LTC6813_mute();
	driverSWLTC6804StartCellAndAuxVoltageConversion(MD_FILTERED, DCP_DISABLED);
	HAL_Delay(250);
	zeroCurrentCalibration();
	driverSWLTC6804ResetAuxRegisters();
	driverSWLTC6804StartAuxVoltageConversion(MD_FILTERED, AUX_CH_ALL);
	//driverSWLTC6804StartLoadedCellVoltageConversion(MD_FILTERED,DCP_ENABLED,CELL_CH_ALL,true);
	//driverSWLTC6804ResetAuxRegisters();
	//driverSWLTC6804StartCellAndAuxVoltageConversion(MD_FILTERED, DCP_DISABLED);
	driverSWLTC6804ReadConfigRegister(1,rxConfig_A);
	sprintf(buffer, "Read ConfigRegisterA : %#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x \r\n",
			rxConfig_A[0][0],rxConfig_A[0][1],rxConfig_A[0][2],rxConfig_A[0][3],
			rxConfig_A[0][4],rxConfig_A[0][5],rxConfig_A[0][6],rxConfig_A[0][7]);
	send_uart(buffer);
	clear_buffer();
	HAL_Delay(100);
	driverSWLTC6804ReadConfigRegisterB(1,rxConfig_B);
	sprintf(buffer, "Read ConfigRegisterB : %#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x \r\n",
			rxConfig_B[0][0],rxConfig_B[0][1],rxConfig_B[0][2],rxConfig_B[0][3],
			rxConfig_B[0][4],rxConfig_B[0][5],rxConfig_B[0][6],rxConfig_B[0][7]);
	send_uart(buffer);
	clear_buffer();
}




void unit_test_LTC6813(void)
{

	//driverSWLTC6804StartCellAndAuxVoltageConversion(MD_FILTERED, DCP_DISABLED);
	//HAL_Delay(300);
	driverSWLTC6804ResetCellVoltageRegisters();
	driverSWLTC6804ResetAuxRegisters();
	LTC6813_mute();
	driverSWLTC6804StartCellAndAuxVoltageConversion(MD_FILTERED, DCP_DISABLED);
	HAL_Delay(250);
	if(driverSWLTC6804ReadCellVoltagesArray(cellModuleVoltages))
	{
		CellMonitorsArrayTranslate();
		sprintf(buffer,"C1:%f,C2:%f,C3:%f,C4:%f,C5:%f,C6:%f,C7:%f,C8:%f,C9:%f,C10:%f,C11:%f,C12:%f,C13:%f,C14:%f,C15:%f,C16:%f,C17:%f,C18:%f,\r\n",
				cellModuleVoltages[0][0],cellModuleVoltages[0][1],cellModuleVoltages[0][2],cellModuleVoltages[0][3],cellModuleVoltages[0][4],cellModuleVoltages[0][5],cellModuleVoltages[0][6],
				cellModuleVoltages[0][7],cellModuleVoltages[0][8],cellModuleVoltages[0][9],cellModuleVoltages[0][10],cellModuleVoltages[0][11],cellModuleVoltages[0][12],
				cellModuleVoltages[0][13],cellModuleVoltages[0][14],cellModuleVoltages[0][15],cellModuleVoltages[0][16],cellModuleVoltages[0][17]);
		send_uart(buffer);
		clear_buffer();
		packVoltage = cellModuleVoltages[0][0] + cellModuleVoltages[0][1] + cellModuleVoltages[0][2] + cellModuleVoltages[0][3] + cellModuleVoltages[0][4] + cellModuleVoltages[0][5] + cellModuleVoltages[0][6] +
				cellModuleVoltages[0][7] + cellModuleVoltages[0][8] + cellModuleVoltages[0][9] + cellModuleVoltages[0][10] + cellModuleVoltages[0][11] + cellModuleVoltages[0][12] +
				cellModuleVoltages[0][13] + cellModuleVoltages[0][14] + cellModuleVoltages[0][15] + cellModuleVoltages[0][16] + cellModuleVoltages[0][17] ;
	}

	if(driverSWLTC6804ReadPackCurrent(packCurrentVoltage) && driverSWLTC6804ReadVREFvoltage(packCurrentVREF))
	{
		sprintf(buffer, "Pack Current ADC voltage(V) : %f , Current VREF voltage(V) : %f \r\n", packCurrentVoltage[0][0], packCurrentVREF[0][1]);
		send_uart(buffer);
		clear_buffer();
		packCurrent = ((packCurrentVoltage[0][0]*1000.0f) - (packCurrentVREF[0][1]*1000.0f))/0.00625; //Vout = Vref +/- (1.25xIp / Ipn)
																//Ip -> packCurrent , Vref = 2.5V
		packCurrent = packCurrent - currentOffset;
		sprintf(buffer, "Pack Current is : %ld (mA)\r\n", packCurrent); //TO DO: moving average filter
		send_uart(buffer);
		clear_buffer();
	}

	driverSWLTC6804ResetAuxRegisters();
	driverSWLTC6804StartAuxVoltageConversion(MD_FILTERED, AUX_CH_ALL);
	HAL_Delay(250);

	if(driverSWLTC6804ReadAuxVoltagesArray(auxModuleVoltages,NTCnominalResistance, NTCseriesResistor, NTCbetaFactor, 25.0f))
	{
		AuxMonitorsArrayTranslate();
		sprintf(buffer,"T1:%f,T2:%f,T3:%f,T4:%f,T5:%f,T6:%f,T7:%f\r\n",
				auxVoltagesIndividual[2].auxVoltage,auxVoltagesIndividual[3].auxVoltage,auxVoltagesIndividual[4].auxVoltage,
				auxVoltagesIndividual[5].auxVoltage,auxVoltagesIndividual[6].auxVoltage,auxVoltagesIndividual[7].auxVoltage,
				auxVoltagesIndividual[8].auxVoltage);
				send_uart(buffer);
				clear_buffer();
	}


	//driverSWLTC6804ResetCellVoltageRegisters();
	//driverSWLTC6804StartCellVoltageConversion(MD_FILTERED,DCP_DISABLED,CELL_CH_ALL);
	//driverSWLTC6804StartCellVoltageConversion(MD_FILTERED,DCP_ENABLED,CELL_CH_ALL);
	//driverSWLTC6804StartCellAndAuxVoltageConversion(MD_FILTERED, DCP_DISABLED);
//	driverSWLTC6804ResetCellVoltageRegisters();
//	driverSWLTC6804ResetAuxRegisters();
//	driverSWLTC6804StartCellAndAuxVoltageConversion(MD_FILTERED, DCP_DISABLED);
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

  init_LTC6813(); //init LTC6813 parameters, write config registers

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

  //wakeup_sleep(1);
  //wakeup_idle(1);
#ifdef ext_LTC681x_lib
  //init_cell_asic_structure(cellMonitorICCount, BMS_IC);
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3); //toggle status LED
	  HAL_Delay(250);
	  //send CAN message // TO DO:check CAN message reception on BluePill
	  HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);

	  //wakeup_idle(1);
	  unit_test_LTC6813();
	  calculateMaxandMinCellVoltages();
	  //cellBalancingTask();


#ifdef ext_LTC681x_lib
	  //cellBalancingUnitTest();
#endif

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

	  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4))
	  {
		  sprintf(buffer,"STAT input pin detected\n");
		  send_uart(buffer);
		  clear_buffer();
	  }

	  get_time();  //print RTC
	  write_to_csvfile();
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
  sTime.Hours = 0x02;
  sTime.Minutes = 0x03;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_SUNDAY;
  sDate.Month = RTC_MONTH_JULY;
  sDate.Date = 0x10;
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

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
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

