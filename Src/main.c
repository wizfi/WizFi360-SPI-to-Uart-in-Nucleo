/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define SPI_RX_DESC_NUM			10

#define SPI_REG_TIMEOUT			200
#define SPI_TIMEOUT				100		// 200mS

#define SPI_REG_INT_STTS		0x06
#define SPI_REG_RX_DAT_LEN		0x02
#define SPI_REG_TX_BUFF_AVAIL	0x03
#define SPI_CMD_RX_DATA			0x10
#define SPI_CMD_TX_CMD			0x91
#define SPI_CMD_TX_DATA			0x90
#define SPI_CS_OFF				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET)
#define SPI_CS_ON				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET)
#define SPI_INT_STTS			HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)


#define debug 					0
#define debug1 					0
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KEIL //KEIL ,True_STD
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
uint8_t RX_BUFFER[512];
uint8_t RX_BUFFER_U6[512];
uint8_t SPI_TX_BUFF[1048];
uint8_t SPI_RX_BUFF[1048];
uint8_t RX_Flag = 0;
uint16_t RX_Index = 0;
uint8_t RX_Flag_U6 = 0;
uint16_t RX_Index_U6 = 0;
uint8_t Spi_rx_flag = 0;
uint8_t transmode = 0;
uint8_t sendmode = 0;
int sendsize = 0;
uint8_t WizFi360_Boot = 0;
uint16_t TransBootCnt = 0;
uint16_t recv_cnt = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
int Int2StrNull(char *data);
void SPI_SEND(uint8_t type, uint8_t *data, uint16_t len);
int CMD_check_mode(void);
int recv_check_message(void);
int SPI_RECV(void);
uint16_t SPI_Read_Register(uint8_t CMD);
uint16_t  SPI_Read_STTS(void);
#ifdef KEIL
     #ifdef __GNUC__
     //With GCC, small printf (option LD Linker->Libraries->Small printf
     //set to 'Yes') calls __io_putchar()
         #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
		#else
				 #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
		#endif /* __GNUC__*/
#if 1
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}
				 #endif
#endif			 

#ifdef True_STD
int _write(int fd, char *str, int len)
{
	for(int i=0; i<len; i++)
	{
		HAL_UART_Transmit(&huart2, (uint8_t *)&str[i], 1, 0xFFFF);
	}
	return len;
}
#endif
uint8_t rxData;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	/*
			This will be called once data is received successfully,
			via interrupts.
	*/

	 /*
		 loop back received data
	 */
	 //uart2_recv_func();
	 //massage input end enter key flag
	#if 0
	 URX_BUF[URX_BUF_cnt++] = rxData;
	 if((rxData == '\r') ||(URX_BUF_cnt > 512))
	 {
		 URX_BUF_Flag = 1;
	 }
	#endif
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_8)
	{
		Spi_rx_flag = 1;
    //SPI_RECV_Proc();
	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void uart2_recv_func(uint8_t Data)
{
  //HAL_UART_Receive_IT(&huart2, &rxData, 1);
	 //HAL_UART_Transmit(&huart2, &rxData, 1, 1000);
	RX_BUFFER[RX_Index++] = Data;
	if(Data == '\n')
	{
    RX_BUFFER[RX_Index] = 0;
		RX_Flag = 1;
	}
  else if (RX_Index==3&&(RX_BUFFER[0]=='+')&&(RX_BUFFER[1]=='+')&&(RX_BUFFER[2]=='+'))
  {
    transmode = 0;
    RX_BUFFER[RX_Index] = 0;
		RX_Flag = 1;
  }  
  else if((sendmode == 1)&&(RX_Index >= sendsize))
  {
    RX_BUFFER[RX_Index] = 0;
		RX_Flag = 1;
    sendmode = 0;
    sendsize = 0;
  }
}
void uart6_recv_func(uint8_t Data)
{
  RX_BUFFER_U6[RX_Index_U6++] = Data;
	if(Data == '\r')
	{
    RX_BUFFER_U6[RX_Index_U6] = 0;
		RX_Flag_U6 = 1;
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
  int temp_delay = 60000;
  int temp_cnt = 0;
  int SPI_RECV_LEN = 0;
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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart2, &rxData, 1);
  HAL_UART_Receive_IT(&huart6, &rxData, 1);
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//while(SPI_INT_STTS == 1);
  while(temp_cnt-->0)
  {
    if(SPI_INT_STTS==0)
		{
			if(temp_cnt>10000)
			{
				break;
			}
		}
  }
  temp_cnt = 100000;

	printf("----------------------------------------------------\r\n");
	printf("||         WizFi360 SPI to UART Test Program       ||\r\n");
	printf("||                   Ver 0.1                       ||\r\n");
	printf("----------------------------------------------------\r\n");
  while (1)
  {
    if(RX_Flag == 1)
    {
      printf("%s",RX_BUFFER);
      if(transmode == 1)
      {
        //
      }else if (sendmode == 1)
      {
        //
      }
      else
      {
        /* code */
        CMD_check_mode();
      }
      SPI_SEND(0, RX_BUFFER, RX_Index);
      RX_Flag = 0;
      RX_Index = 0;
    }
    
    if(Spi_rx_flag||(SPI_INT_STTS == 0))
    {
			//printf("spi input %d\r\n", Spi_rx_flag);
      SPI_RECV_LEN = SPI_RECV();
      if(SPI_RECV_LEN>0)
      //if(SPI_RECV()>0)
      {
        //printf("spi DATA LEN %d\r\n", SPI_RECV_LEN);
        recv_check_message();
      }
      Spi_rx_flag = 0;
    }
    if(RX_Flag_U6 == 1)
    {
      printf("%s",RX_BUFFER_U6);
      if(strncmp(RX_BUFFER_U6, "check", 5) == 0)
      {
        SPI_Read_STTS();
      }
      RX_Flag_U6 = 0;
      RX_Index_U6 = 0;
    }
    if(--temp_cnt <= 0)
    {
      temp_cnt = 900000;
      //printf("idle SPI STATUS[%d]\r\n", SPI_INT_STTS);
    }
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

// int to string untill null
int Int2StrNull(char *data)
{
  char temp_data[5];
  int temp_cnt = 0;
  int result = 0;
  while(!((*data == NULL)||(*data == '\r')||(*data == '\n')))
  {
    //str to int
    temp_data[temp_cnt++] = *data;
    data++;
    if(temp_cnt>5)
      return -1;
  }
  temp_data[temp_cnt]=0;
  result = atoi(temp_data);
  return result;
}
//SPI Read Register
uint16_t SPI_Read_Register(uint8_t CMD)
{
  uint8_t dum = 0xFF,rx_temp[2]={0,};
  uint16_t ret = 0;
  HAL_SPI_TransmitReceive(&hspi1, &CMD, &dum, 1, 10);
  dum = 0xFF;
  #if 1
  HAL_SPI_TransmitReceive(&hspi1, &dum, &rx_temp[0], 1, 10);
  HAL_SPI_TransmitReceive(&hspi1, &dum, &rx_temp[1], 1, 10);
  #else
  HAL_SPI_TransmitReceive(&hspi1, &dum, rx_temp, 2, 10);
  #endif
  ret = rx_temp[0] | (rx_temp[1] << 8);
  return ret;
}
//SPI SEND Message
void SPI_SEND(uint8_t type, uint8_t *data, uint16_t len)
{
  #if 1
  //int i;
  //HAL_UART_Transmit(&huart1, data, len, 0xFFFF);
  //read TX_BUFF_AVAIL
  uint8_t temp_CMD, retry = 0, err = 0, dum2=0x00;
  uint16_t SPI_RX_REG = 0, TX_len;
  int temp_delay = 10000;
  memset(SPI_TX_BUFF,0, sizeof(SPI_TX_BUFF));
  
  temp_CMD = SPI_REG_TX_BUFF_AVAIL;
  
  while((SPI_RX_REG != 0xffff) && (0 == (SPI_RX_REG & 0x02)))
  {
    retry++;
		SPI_CS_OFF;
    SPI_RX_REG = SPI_Read_Register(temp_CMD);
		SPI_CS_ON;
    #if 1
		printf("SPI_REG_TX_BUFF_AVAIL[%d]\r\n", SPI_RX_REG);
    #endif
    if(retry > SPI_TIMEOUT)
    {
      printf("SPI CMD timeout-------------------------------------------\r\n");
      retry = 0;
      err = 1;
      break;
    }
    #if 1
		printf("SPI SEND Retry[%d]\r\n", retry);
    #endif
    while(--temp_delay > 1);
    temp_delay = 10000;
  }
  
  TX_len = len + 2;
  if(TX_len % 4)
  {
    TX_len = ((TX_len + 3)/4) << 2;
    //TX_len = (TX_len + 3) & 0xFFFC;
  }
  //printf("SPI_REG_TX_BUFF_AVAIL[%d]\r\n", SPI_RX_REG);
#if debug
  printf("SPI_REG_TX_BUFF_AVAIL[%d]\r\n", SPI_RX_REG);
  printf("TX Hex1 : ");
  for(i=0; i<RX_Index; i++)
  {
    printf("[%d]%0.2X ",i, RX_BUFFER[i]);
  }
  printf("\r\n");
#endif
  //printf("trans[%d][%d] : %s\r\n", len, TX_len, data);
  //printf("trans[%d][%d]\r\n", len, TX_len);
  if(err==0)
  {
    SPI_CS_OFF;
    if(type)
    {
      temp_CMD = SPI_CMD_TX_DATA;
    }
    else
    {
      temp_CMD = SPI_CMD_TX_CMD;
    }
    HAL_SPI_TransmitReceive(&hspi1, &temp_CMD, &dum2, 1, 10);
    memcpy(SPI_TX_BUFF , &len, sizeof(len));
    memcpy(SPI_TX_BUFF + 2, data, len);
    HAL_SPI_TransmitReceive(&hspi1, SPI_TX_BUFF, RX_BUFFER, TX_len, 10);

    SPI_CS_ON;
    #if 1
    printf("<<%s",SPI_TX_BUFF + 2);
    
    #else
    printf("%d [%0.2X %0.2X]%s \r\n",TX_len, SPI_TX_BUFF[0], SPI_TX_BUFF[1], SPI_TX_BUFF + 2);
    #endif
#if debug
    printf("TX Hex2 : ");
    for(i=0; i<TX_len; i++)
    {
      printf("[%d]%0.2X ",i, SPI_TX_BUFF[i]);
    }
    printf("\r\n");
#endif
  }
  #else
  int i;
  //HAL_UART_Transmit(&huart1, data, len, 0xFFFF);
  //read TX_BUFF_AVAIL
  uint8_t temp_CMD, retry = 0, err = 0, dum = 0xFF, dum2=0x00, rx_temp[2];
  uint16_t spi_rx_len = 0, TX_len;
  memset(SPI_TX_BUFF,0, sizeof(SPI_TX_BUFF));
  SPI_CS_OFF;
  temp_CMD = SPI_REG_TX_BUFF_AVAIL;

  while((spi_rx_len != 0xffff) && (0 == (spi_rx_len & 0x02)))
  {
    retry++;
    HAL_SPI_TransmitReceive(&hspi1, &temp_CMD, &dum2, 1, 10);
    HAL_SPI_TransmitReceive(&hspi1, &dum, &rx_temp[0], 1, 10);
    HAL_SPI_TransmitReceive(&hspi1, &dum, &rx_temp[1], 1, 10);
    spi_rx_len = rx_temp[0] | (rx_temp[1] << 8);

    if(retry > SPI_TIMEOUT)
    {
      printf("SPI CMD timeout\r\n");
      retry = 0;
      err = 1;
      break;
    }
  }
  SPI_CS_ON;
  TX_len = len + 2;
  if(TX_len % 4)
  {
    TX_len = ((TX_len + 3)/4) << 2;
    //TX_len = (TX_len + 3) & 0xFFFC;
  }
  //printf("SPI_REG_TX_BUFF_AVAIL[%d]\r\n", spi_rx_len);
#if debug
  printf("SPI_REG_TX_BUFF_AVAIL[%d]\r\n", spi_rx_len);
  printf("TX Hex1 : ");
  for(i=0; i<RX_Index; i++)
  {
    printf("[%d]%0.2X ",i, RX_BUFFER[i]);
  }
  printf("\r\n");
#endif
  //printf("trans[%d][%d] : %s\r\n", len, TX_len, data);
  //printf("trans[%d][%d]\r\n", len, TX_len);
  if(err==0)
  {
    SPI_CS_OFF;
    if(type)
    {
      temp_CMD = SPI_CMD_TX_DATA;
    }
    else
    {
      temp_CMD = SPI_CMD_TX_CMD;
    }
    HAL_SPI_TransmitReceive(&hspi1, &temp_CMD, &dum2, 1, 10);
    memcpy(SPI_TX_BUFF , &len, sizeof(len));
    memcpy(SPI_TX_BUFF + 2, data, len);
    HAL_SPI_TransmitReceive(&hspi1, SPI_TX_BUFF, RX_BUFFER, TX_len, 10);

    SPI_CS_ON;
    //printf("%d [%0.2X %0.2X]%s \r\n",TX_len, SPI_TX_BUFF[0], SPI_TX_BUFF[1], SPI_TX_BUFF + 2);
#if debug
    printf("TX Hex2 : ");
    for(i=0; i<TX_len; i++)
    {
      printf("[%d]%0.2X ",i, SPI_TX_BUFF[i]);
    }
    printf("\r\n");
#endif
  }
  #endif
}
// SPI Receive 
int SPI_RECV(void)
{
  #if 1
  uint8_t temp_CMD, dum = 0xFF, dum2=0x00;
  uint16_t SPI_RX_REG = 0;
  
#if debug1
  printf("SPI Interrupt input\r\n");
#endif
  SPI_CS_OFF;
  temp_CMD = SPI_REG_INT_STTS;
  SPI_RX_REG = SPI_Read_Register(temp_CMD);;
  SPI_CS_ON;
  if(SPI_RX_REG == 0)
    return 0;
#if 1
  printf("SPI_REG_INT_STTS[%d]\r\n", SPI_RX_REG);
#endif
  if((SPI_RX_REG != 0xffff) && (SPI_RX_REG & 0x01))
  {
    SPI_CS_OFF;
    temp_CMD = SPI_REG_RX_DAT_LEN;
    SPI_RX_REG = SPI_Read_Register(temp_CMD);
    SPI_CS_ON;
  }
#if 1
    printf("SPI_REG_RX_DAT_LEN[%d]\r\n", SPI_RX_REG);
#endif
  if(SPI_RX_REG > 0)
  {
    SPI_CS_OFF;
    temp_CMD = SPI_CMD_RX_DATA;
    HAL_SPI_TransmitReceive(&hspi1, &temp_CMD, &dum2, 1, 10);
    HAL_SPI_TransmitReceive(&hspi1, &dum, SPI_RX_BUFF, SPI_RX_REG, 10);
    SPI_RX_BUFF[SPI_RX_REG] = 0;
    #if 0
    for(i=0; i<SPI_RX_REG; i++)
    {
      EnQueue(SPI_RX_BUFF[i]);
    }
    #endif
    SPI_CS_ON;
    #if 1 //teddy 191111
    #if 0 
    if(get_Socket_status() == 0)
    {
      for(i=0; i<SPI_RX_REG; i++)
      {
        EnQueue(SPI_RX_BUFF[i]);
      }
    }
    else
    {
      SPI_Input_Data_Proc(SPI_RX_REG, SPI_RX_BUFF);
    }
    #endif
    #else
    for(i=0; i<SPI_RX_REG; i++)
    {
      EnQueue(SPI_RX_BUFF[i]);
    }
    #endif
#if debug1
    printf("RX Hex : ");
    for(i=0; i<SPI_RX_REG; i++)
    {
      printf("[%d]%0.2X ",i, SPI_RX_BUFF[i]);
    }
    printf("\r\n");
#endif
    #if 1
    printf(">>%s",SPI_RX_BUFF);
    HAL_UART_Transmit(&huart2, (uint8_t *)SPI_RX_BUFF, SPI_RX_REG, 0xFFFF);
    #else
    printf("RX[%d]:[%s]\r\n", SPI_RX_REG, SPI_RX_BUFF);
    //printf("RX[%d]\r\n", SPI_RX_REG);
    #endif
    return SPI_RX_REG;
  }
  #else
  uint8_t temp_CMD, dum = 0xFF, dum2=0x00,rx_temp[2]={0,0};
  uint16_t spi_rx_len = 0;
  uint16_t i;
  
#if debug1
  printf("SPI Interrupt input\r\n");
#endif
  SPI_CS_OFF;
  temp_CMD = SPI_REG_INT_STTS;
  HAL_SPI_TransmitReceive(&hspi1, &temp_CMD, &dum2, 1, 10);
  HAL_SPI_TransmitReceive(&hspi1, &dum, &rx_temp[0], 1, 10);
  HAL_SPI_TransmitReceive(&hspi1, &dum, &rx_temp[1], 1, 10);
  spi_rx_len = rx_temp[0] | (rx_temp[1] << 8);
  SPI_CS_ON;
#if 0
  printf("SPI_REG_INT_STTS[%d]\r\n", spi_rx_len);
#endif
  if((spi_rx_len != 0xffff) && (spi_rx_len & 0x01))
  {
    SPI_CS_OFF;
    temp_CMD = SPI_REG_RX_DAT_LEN;
    HAL_SPI_TransmitReceive(&hspi1, &temp_CMD, &dum2, 1, 10);
    HAL_SPI_TransmitReceive(&hspi1, &dum, &rx_temp[0], 1, 10);
    HAL_SPI_TransmitReceive(&hspi1, &dum, &rx_temp[1], 1, 10);
    spi_rx_len = rx_temp[0] | (rx_temp[1] << 8);
    SPI_CS_ON;
  }
#if debug1
    printf("SPI_REG_RX_DAT_LEN[%d]\r\n", spi_rx_len);
#endif
  if(spi_rx_len > 0)
  {
    SPI_CS_OFF;
    temp_CMD = SPI_CMD_RX_DATA;
    HAL_SPI_TransmitReceive(&hspi1, &temp_CMD, &dum2, 1, 10);
    HAL_SPI_TransmitReceive(&hspi1, &dum, SPI_RX_BUFF, spi_rx_len, 10);
    SPI_RX_BUFF[spi_rx_len] = 0;
    #if 0
    for(i=0; i<spi_rx_len; i++)
    {
      EnQueue(SPI_RX_BUFF[i]);
    }
    #endif
    SPI_CS_ON;
    #if 1 //teddy 191111
    #if 0
    if(get_Socket_status() == 0)
    {
      for(i=0; i<spi_rx_len; i++)
      {
        EnQueue(SPI_RX_BUFF[i]);
      }
    }
    else
    {
      SPI_Input_Data_Proc(spi_rx_len, SPI_RX_BUFF);
    }
    #endif
    #else
    for(i=0; i<spi_rx_len; i++)
    {
      EnQueue(SPI_RX_BUFF[i]);
    }
    #endif
#if debug1
    printf("RX Hex : ");
    for(i=0; i<spi_rx_len; i++)
    {
      printf("[%d]%0.2X ",i, SPI_RX_BUFF[i]);
    }
    printf("\r\n");
#endif
		printf("%s", SPI_RX_BUFF);
    //printf("RX[%d]:[%s]\r\n", spi_rx_len, SPI_RX_BUFF);
    //printf("RX[%d]\r\n", spi_rx_len);
    return 1;
  }
  #endif
  return 0;
}
uint16_t  SPI_Read_STTS(void)
{
  uint8_t temp_CMD;
  uint16_t SPI_RX_REG = 0;
  
#if debug1
  printf("SPI Interrupt input\r\n");
#endif
  SPI_CS_OFF;
  temp_CMD = SPI_REG_INT_STTS;
  SPI_RX_REG = SPI_Read_Register(temp_CMD);;
  SPI_CS_ON;
  printf("SPI_INT_STTS %08x\r\n", SPI_RX_REG);
  SPI_RX_REG = 0;
  SPI_CS_OFF;
  temp_CMD = SPI_REG_RX_DAT_LEN;
  SPI_RX_REG = SPI_Read_Register(temp_CMD);
  SPI_CS_ON;
  printf("SPI_RXDATLEN %08x %d\r\n", SPI_RX_REG, SPI_RX_REG);
  if(SPI_RX_REG > 0)
  {
    SPI_RECV();
  }
  return 0;
}
int CMD_check_mode(void)
{
  if(strncmp(RX_BUFFER, "AT+CIPMODE=", 11) == 0)
  {
    //CIP MODE
    if(RX_BUFFER[11]=='1')
    {
      //1
      transmode = 1;
    }
    else if(RX_BUFFER[11]=='0')
    {
      //0
      transmode = 0;
    }
    else
    {
      /* error code */
      printf("CMD Value Error!!\r\n");
    }
  }
  else if(strncmp(RX_BUFFER, "AT+CIPSEND", 10) == 0)
  {
    //send func
    if(RX_BUFFER[10]=='=')
    {
      // CIPSEND
      sendsize = Int2StrNull(RX_BUFFER+11);
      if(sendsize > 0)
      {
        sendmode = 1;
      }
      else
      {
        /* error code */
        printf("CMD SEND SIZE Error!!\r\n");
      }
      printf("S:%s I:%d \r\n", RX_BUFFER+11, sendsize);
    }
    else if(strncmp(RX_BUFFER+10, "BUF=", 4) == 0)
    {
      // CIPSENDBUF
      sendsize = Int2StrNull(RX_BUFFER+14);
      if(sendsize > 0)
      {
        sendmode = 1;
      }
      else
      {
        /* error code */
        printf("CMD SEND BUF SIZE Error!!\r\n");
      }
      printf("S:%s I:%d \r\n", RX_BUFFER+11, sendsize);
    }
    else if(strncmp(RX_BUFFER+10, "EX=", 3) == 0)
    {
      // CIP SENDEX
      sendsize = Int2StrNull(RX_BUFFER+13);
      if(sendsize > 0)
      {
        sendmode = 1;
      }
      else
      {
        /* error code */
        printf("CMD SEND EX SIZE Error!!\r\n");
      }
      printf("S:%s I:%d \r\n", RX_BUFFER+11, sendsize);
    }
  }
  return 0;
}
int recv_check_message(void)
{
  //SPI_RX_BUFF, SPI_RX_REG
  if(sendmode == 1) //send mode
  {
    if(strncmp(SPI_RX_BUFF,"link is not valid", 17) == 0)
    {
      printf("send error!! \r\n");
      sendmode = 0;
    }
  }else if(transmode == 0)
  {//WizFi360_Boot
    if(strncmp(SPI_RX_BUFF,"\r\nready", 5) == 0)
    {
      WizFi360_Boot = 1;
      TransBootCnt = 0;
      printf("WizFi360 Boot\r\n");
    }
    if(WizFi360_Boot == 1)
    {
      TransBootCnt++;
      if(strncmp(SPI_RX_BUFF,"CONNECT\r\n", 9) == 0)
      {
        printf("WizFi360 Server Connect \r\n");
        if(TransBootCnt < 10)
        {
          printf("WizFi360 Transmode start \r\n");
          transmode = 1;
        }

      }
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
