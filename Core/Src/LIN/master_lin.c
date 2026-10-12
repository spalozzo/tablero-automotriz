/**
 * @file master_lin.c
 * @brief Esqueleto base para el Master LIN (Gateway) usando STM32 HAL
 * 
 * UART Asignado: USART1 (TX: PA9, RX: PA10)
 * Pines: WAKE_Master (PC7), SW1-3 (PC0-PC2), ADC_Pot (PA0)
 */

#include "main.h"

// Estas variables deben coincidir con las generadas por STM32CubeMX
extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1;

// Configuración del protocolo
#define TABLERO_LIN_ID 0x10
#define TABLERO_PID    0x50 // PID ya calculado (ID + bits de paridad P0 y P1)

/**
 * @brief Inicializa el hardware necesario (post-CubeMX)
 */
void Master_LIN_Init(void) {
    // Ponemos el pin WAKE_Master en alto para despertar al TJA1021T (Modo Normal)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); 
    
    // Opcional: Calibración del ADC
    // HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
}

/**
 * @brief Calcula el Checksum Clásico (solo sobre los datos) para LIN 1.3 / 2.0
 */
uint8_t Calcular_Checksum_Clasico(uint8_t *data, uint8_t length) {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < length; i++) {
        sum += data[i];
        if (sum > 0xFF) {
            sum -= 0xFF; // Carry
        }
    }
    return (uint8_t)(~sum);
}

/**
 * @brief Lee los sensores y envía la trama LIN hacia el Tablero
 */
void Master_Enviar_Trama_LIN(void) {
    uint8_t lin_header[2];
    uint8_t lin_data[4];
    uint8_t checksum;

    /* 1. LEER ENTRADAS FÍSICAS */
    uint8_t sw_state = 0;
    
    // Leemos los DIP Switches (Asumimos que cierran a GND, entonces están en LOW cuando se presionan.
    // Ojo: En CubeMX tenés que haberles configurado el Pull-Up interno a PC0, PC1 y PC2)
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_RESET) sw_state |= 0x01; // SW1
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_RESET) sw_state |= 0x02; // SW2
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_RESET) sw_state |= 0x04; // SW3

    // Leer ADC del potenciómetro
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t pot_val = HAL_ADC_GetValue(&hadc1); // Valor 0 - 4095 (12 bits)

    /* 2. EMPAQUETAR DATOS */
    lin_data[0] = sw_state;
    lin_data[1] = (uint8_t)(pot_val >> 4); // Escalamos de 12 a 8 bits (0 - 255) para que entre en 1 byte
    lin_data[2] = 0x00; // Byte libre
    lin_data[3] = 0x00; // Byte libre

    checksum = Calcular_Checksum_Clasico(lin_data, 4);

    /* 3. ENVIAR TRAMA LIN FÍSICA */
    
    // Paso A: Generar el Break (Señal dominante larga de ~13 bits) usando hardware UART LIN
    HAL_LIN_SendBreak(&huart1);
    
    // Breve delay opcional si la interrupción LIN/UART necesita tiempo para recuperarse del break
    // HAL_Delay(1); 

    // Paso B: Enviar campo Sync (0x55) y el PID
    lin_header[0] = 0x55;
    lin_header[1] = TABLERO_PID;
    HAL_UART_Transmit(&huart1, lin_header, 2, 10);

    // Paso C: Enviar Data Payload + Checksum
    HAL_UART_Transmit(&huart1, lin_data, 4, 10);
    HAL_UART_Transmit(&huart1, &checksum, 1, 10);
}
