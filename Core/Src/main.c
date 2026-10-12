/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DIAG_DURATION 10000 //in milliseconds
#define DIAG_ON		1
#define DIAG_OFF 	0

#define SYNC		0x55
#define PID_DIAG	0x3C
#define PID_NORM	0x10
#define PID_SLSL 	0x20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
extern UART_HandleTypeDef huart6;

// Variables para la recepción LIN (Esclavo en UART6)
volatile uint8_t lin_rx_byte;
volatile uint8_t lin_rx_buffer[8];
volatile uint8_t lin_rx_state = 0; 
volatile uint8_t lin_rx_pid = 0;
volatile uint8_t lin_rx_index = 0;

// Estados MdE Slave
#define STATE_WAITING		0
#define STATE_PROCESS_PID	1
#define STATE_NORMAL		2
#define STATE_DIAGNOSTIC	3
#define STATE_SLAVE_SLAVE	4

// Variables para Diagnóstico (Punto 2)
volatile uint8_t modo_diagnostico_activo = DIAG_OFF;
volatile uint32_t tiempo_inicio_diagnostico = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint32_t value_read_ADC;
volatile uint8_t debug_placa = 0;


void Test_Placa_Hardware(void);
uint8_t Calcular_Checksum(uint8_t *data, uint8_t length);
void Procesar_Trama_Esclavo(void);
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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  // NOTA: Los transceptores TJA1020 se inicializan por hardware de forma permanente
  // (NSLP está cableado a 3.3V y NWAKE fue físicamente desconectado de la STM32)
  
  // Iniciar la recepción por interrupción para el Esclavo (UART6)
  HAL_UART_Receive_IT(&huart6, (uint8_t *)&lin_rx_byte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(!debug_placa){
		  // --- CHEQUEO DE BOTON PARA DIAGNOSTICO (PUNTO 2) ---
		  // Usamos el boton azul de la Nucleo (PC13)
		  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
			  modo_diagnostico_activo = DIAG_ON;
			  tiempo_inicio_diagnostico = HAL_GetTick();
		  }

		  // Control de tiempo de diagnóstico (10 segundos)
		  if (modo_diagnostico_activo) {
			  if ((HAL_GetTick() - tiempo_inicio_diagnostico) > DIAG_DURATION) {
				  modo_diagnostico_activo = DIAG_OFF;
			  }
		  }

		  /* --- LOGICA DEL MASTER (UART1) --- */
		  uint8_t lin_header[2];
		  uint8_t lin_data[4];
		  uint8_t checksum;

		  if (modo_diagnostico_activo) {
			  // PUNTO 2: Trama Diagnóstico
			  HAL_LIN_SendBreak(&huart1);
			  lin_header[0] = SYNC;	// Sync 0b01010101
			  lin_header[1] = PID_DIAG; // PID Diagnostico
			  HAL_UART_Transmit(&huart1, lin_header, 2, 10);

			  lin_data[0] = 0xFF; // Comando generico
			  checksum = Calcular_Checksum(lin_data, 1);
			  HAL_UART_Transmit(&huart1, lin_data, 1, 10);
			  HAL_UART_Transmit(&huart1, &checksum, 1, 10);

		  } else {
			  // Leer sensores directos
			  HAL_ADC_Start(&hadc1);
			  HAL_ADC_PollForConversion(&hadc1, 10);
			  uint16_t pot_val = HAL_ADC_GetValue(&hadc1);

			  uint8_t sw_state = 0;
			  if (HAL_GPIO_ReadPin(GPIOC, SW1_Pin) == GPIO_PIN_RESET) sw_state |= 0x01;
			  if (HAL_GPIO_ReadPin(GPIOC, SW2_Pin) == GPIO_PIN_RESET) sw_state |= 0x02;
			  if (HAL_GPIO_ReadPin(GPIOC, SW3_Pin) == GPIO_PIN_RESET) sw_state |= 0x04;

			  // PUNTO 1: Trama Normal
			  HAL_LIN_SendBreak(&huart1);
			  lin_header[0] = SYNC;
			  lin_header[1] = PID_NORM; // PID Normal
			  HAL_UART_Transmit(&huart1, lin_header, 2, 10);

			  lin_data[0] = sw_state;
			  lin_data[1] = (uint8_t)(pot_val >> 4);
			  checksum = Calcular_Checksum(lin_data, 2);
			  HAL_UART_Transmit(&huart1, lin_data, 2, 10);
			  HAL_UART_Transmit(&huart1, &checksum, 1, 10);

			  HAL_Delay(10); // Pausa entre tramas del schedule table

			  // PUNTO 3: Trama Multi-Slave

			  HAL_LIN_SendBreak(&huart1);
			  lin_header[0] = SYNC;
			  lin_header[1] = PID_SLSL; // PID Slave-to-Slave
			  HAL_UART_Transmit(&huart1, lin_header, 2, 10);

			  lin_data[0] = (uint8_t)(pot_val >> 4);
			  checksum = Calcular_Checksum(lin_data, 1);
			  HAL_UART_Transmit(&huart1, lin_data, 1, 10);
			  HAL_UART_Transmit(&huart1, &checksum, 1, 10);
		  }

		  HAL_Delay(100); // Ciclo de 100ms
  	  }else{
  		// Opcional: Para testear directo sin LIN, descomentar la siguiente linea:
  		Test_Placa_Hardware();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_LIN_Init(&huart1, UART_LINBREAKDETECTLENGTH_10B) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart6.Init.BaudRate = 19200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_LIN_Init(&huart6, UART_LINBREAKDETECTLENGTH_10B) != HAL_OK)
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|LD_G3_Pin|LD_G2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD_R4_Pin|LD_R5_Pin|WAKE_Slave_Pin|LD_R1_Pin
                          |LD_R2_Pin|LD_R3_Pin|LD_G1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WAKE_Master_GPIO_Port, WAKE_Master_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SW1_Pin SW2_Pin SW3_Pin */
  GPIO_InitStruct.Pin = SW1_Pin|SW2_Pin|SW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LD_G3_Pin LD_G2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LD_G3_Pin|LD_G2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD_R4_Pin LD_R5_Pin WAKE_Slave_Pin LD_R1_Pin
                           LD_R2_Pin LD_R3_Pin LD_G1_Pin */
  GPIO_InitStruct.Pin = LD_R4_Pin|LD_R5_Pin|WAKE_Slave_Pin|LD_R1_Pin
                          |LD_R2_Pin|LD_R3_Pin|LD_G1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : WAKE_Master_Pin */
  GPIO_InitStruct.Pin = WAKE_Master_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WAKE_Master_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART6) {
        // Máquina de estados del Esclavo
        switch(lin_rx_state) {
            case STATE_WAITING: // Sync
                if (lin_rx_byte == 0x55) lin_rx_state = STATE_PROCESS_PID;
                break;
            case STATE_PROCESS_PID: // PID
                lin_rx_pid = lin_rx_byte;
                lin_rx_index = 0;
                if (lin_rx_pid == 0x10) lin_rx_state = STATE_NORMAL;
                else if (lin_rx_pid == 0x3C) lin_rx_state = STATE_DIAGNOSTIC;
                else if (lin_rx_pid == 0x20) lin_rx_state = STATE_SLAVE_SLAVE;
                else lin_rx_state = STATE_WAITING;
                break;
            case STATE_NORMAL: // Recibir P1
                lin_rx_buffer[lin_rx_index++] = lin_rx_byte;
                if (lin_rx_index == 3) { // 2 data + 1 chk
                    if (lin_rx_buffer[2] == Calcular_Checksum((uint8_t*)lin_rx_buffer, 2)) {
                        Procesar_Trama_Esclavo();
                    }
                    lin_rx_state = STATE_WAITING;
                }
                break;
            case STATE_DIAGNOSTIC: // Recibir P2 (Diagnóstico)
                lin_rx_buffer[lin_rx_index++] = lin_rx_byte;
                if (lin_rx_index == 2) { // 1 data + 1 chk
                    if (lin_rx_buffer[1] == Calcular_Checksum((uint8_t*)lin_rx_buffer, 1)) {
                        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7);
                        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
                        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);
                    }
                    lin_rx_state = STATE_WAITING;
                }
                break;
            case STATE_SLAVE_SLAVE: // Recibir P3 (Multi-Slave)
                lin_rx_buffer[lin_rx_index++] = lin_rx_byte;
                if (lin_rx_index == 2) { // 1 data + 1 chk
                    if (lin_rx_buffer[1] == Calcular_Checksum((uint8_t*)lin_rx_buffer, 1)) {
                        uint8_t pot = lin_rx_buffer[0];
                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,  (pot > 40)  ? GPIO_PIN_SET : GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,  (pot > 80)  ? GPIO_PIN_SET : GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,  (pot > 120) ? GPIO_PIN_SET : GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, (pot > 160) ? GPIO_PIN_SET : GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, (pot > 200) ? GPIO_PIN_SET : GPIO_PIN_RESET);
                        //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, (pot > 240) ? GPIO_PIN_SET : GPIO_PIN_RESET);
                    }
                    lin_rx_state = STATE_WAITING;
                }
                break;
            default:
                lin_rx_state = STATE_WAITING;
                break;
        }
        
        // Relanzar interrupción
        HAL_UART_Receive_IT(&huart6, (uint8_t *)&lin_rx_byte, 1);
    }
}

uint8_t Calcular_Checksum(uint8_t *data, uint8_t length) {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < length; i++) {
        sum += data[i];
        if (sum > 0xFF) sum -= 0xFF;
    }
    return (uint8_t)(~sum);
}

void Procesar_Trama_Esclavo(void) {
    uint8_t sw_state = lin_rx_buffer[0];
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, (sw_state & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, (sw_state & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, (sw_state & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Test_Placa_Hardware(void) {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    value_read_ADC = HAL_ADC_GetValue(&hadc1);
    
    uint32_t value_SW1 = HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin);
    uint32_t value_SW2 = HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin);
    uint32_t value_SW3 = HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin);
    
    uint32_t led_validation = value_read_ADC / 600;

    switch(led_validation)
    {
        case 0:
            HAL_GPIO_WritePin(LD_R1_GPIO_Port, LD_R1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LD_R2_GPIO_Port, LD_R2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LD_R3_GPIO_Port, LD_R3_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LD_R4_GPIO_Port, LD_R4_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LD_R5_GPIO_Port, LD_R5_Pin, GPIO_PIN_RESET);
            break;
        case 1:
            HAL_GPIO_WritePin(LD_R1_GPIO_Port, LD_R1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LD_R2_GPIO_Port, LD_R2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LD_R3_GPIO_Port, LD_R3_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LD_R4_GPIO_Port, LD_R4_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LD_R5_GPIO_Port, LD_R5_Pin, GPIO_PIN_RESET);
            break;
        case 2:
            HAL_GPIO_WritePin(LD_R1_GPIO_Port, LD_R1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LD_R2_GPIO_Port, LD_R2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LD_R3_GPIO_Port, LD_R3_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LD_R4_GPIO_Port, LD_R4_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LD_R5_GPIO_Port, LD_R5_Pin, GPIO_PIN_RESET);
            break;
        case 3:
            HAL_GPIO_WritePin(LD_R1_GPIO_Port, LD_R1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LD_R2_GPIO_Port, LD_R2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LD_R3_GPIO_Port, LD_R3_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LD_R4_GPIO_Port, LD_R4_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LD_R5_GPIO_Port, LD_R5_Pin, GPIO_PIN_RESET);
            break;
        case 4:
            HAL_GPIO_WritePin(LD_R1_GPIO_Port, LD_R1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LD_R2_GPIO_Port, LD_R2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LD_R3_GPIO_Port, LD_R3_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LD_R4_GPIO_Port, LD_R4_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LD_R5_GPIO_Port, LD_R5_Pin, GPIO_PIN_RESET);
            break;
        case 5:
            HAL_GPIO_WritePin(LD_R1_GPIO_Port, LD_R1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LD_R2_GPIO_Port, LD_R2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LD_R3_GPIO_Port, LD_R3_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LD_R4_GPIO_Port, LD_R4_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LD_R5_GPIO_Port, LD_R5_Pin, GPIO_PIN_SET);
            break;
    }

    if(!value_SW1)
        HAL_GPIO_WritePin(LD_G1_GPIO_Port, LD_G1_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(LD_G1_GPIO_Port, LD_G1_Pin, GPIO_PIN_RESET);

    if(!value_SW2)
        HAL_GPIO_WritePin(LD_G2_GPIO_Port, LD_G2_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(LD_G2_GPIO_Port, LD_G2_Pin, GPIO_PIN_RESET);

    if(!value_SW3)
        HAL_GPIO_WritePin(LD_G3_GPIO_Port, LD_G3_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(LD_G3_GPIO_Port, LD_G3_Pin, GPIO_PIN_RESET);

    char msg[50];
    sprintf(msg, "Valor ADC: %lu\r\n", value_read_ADC);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
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
