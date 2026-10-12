/**
 * @file slave_lin.c
 * @brief Esqueleto base para el Slave LIN (Tablero) usando STM32 HAL
 * 
 * UART Asignado: USART1 (TX: PA9, RX: PA10)
 * Pines: WAKE_Slave (PC7), LEDs IND (PA5-7), LEDs ANALOG (PB0, PB1, PB2, PB13, PB14, PB15)
 */

#include "main.h"

// Esta variable debe coincidir con la generada por STM32CubeMX
extern UART_HandleTypeDef huart1;

#define TABLERO_PID 0x50

/**
 * @brief Inicializa el hardware necesario (post-CubeMX)
 */
void Slave_LIN_Init(void) {
    // Ponemos el pin WAKE_Slave en alto para despertar al TJA1021T (Modo Normal)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
}

/**
 * @brief Calcula el Checksum Clásico
 */
uint8_t Calcular_Checksum_Clasico_RX(uint8_t *data, uint8_t length) {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < length; i++) {
        sum += data[i];
        if (sum > 0xFF) sum -= 0xFF;
    }
    return (uint8_t)(~sum);
}

/**
 * @brief Función bloqueante básica para recibir la trama (para poner en el while(1)).
 * En un sistema real se recomienda usar Interrupciones (UART RXNE) o DMA + LIN Break Detection.
 */
void Slave_Recibir_Trama_LIN(void) {
    uint8_t rx_byte = 0;
    uint8_t lin_data[4];
    uint8_t checksum;
    
    // NOTA: Para una implementación simple, esperamos el SYNC byte (0x55)
    // El break a nivel UART a veces se lee como un 0x00 con Framing Error.
    // Acá simplemente filtramos buscando el 0x55.
    
    if (HAL_UART_Receive(&huart1, &rx_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
        if (rx_byte == 0x55) { // Encontramos el byte de Sincronización
            
            // Leer PID
            if (HAL_UART_Receive(&huart1, &rx_byte, 1, 5) == HAL_OK) {
                if (rx_byte == TABLERO_PID) {
                    
                    // Es la trama destinada al tablero. Leer 4 bytes de datos y el checksum.
                    if (HAL_UART_Receive(&huart1, lin_data, 4, 10) == HAL_OK) {
                        if (HAL_UART_Receive(&huart1, &checksum, 1, 5) == HAL_OK) {
                            
                            // Validar el Checksum
                            if (checksum == Calcular_Checksum_Clasico_RX(lin_data, 4)) {
                                
                                /* APLICAR DATOS A LA LÓGICA DEL TABLERO */
                                uint8_t sw_state = lin_data[0];
                                uint8_t pot_val  = lin_data[1]; // 0 a 255

                                // 1. LEDs Indicadores (Mapeados a PA5, PA6, PA7)
                                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, (sw_state & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
                                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, (sw_state & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
                                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, (sw_state & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);

                                // 2. Barra de LEDs Analógicos (Simulando un tacómetro o velocímetro)
                                // Pines: PB0, PB1, PB2, PB13, PB14, PB15
                                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,  (pot_val > 40)  ? GPIO_PIN_SET : GPIO_PIN_RESET);
                                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,  (pot_val > 80)  ? GPIO_PIN_SET : GPIO_PIN_RESET);
                                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,  (pot_val > 120) ? GPIO_PIN_SET : GPIO_PIN_RESET);
                                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, (pot_val > 160) ? GPIO_PIN_SET : GPIO_PIN_RESET);
                                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, (pot_val > 200) ? GPIO_PIN_SET : GPIO_PIN_RESET);
                                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, (pot_val > 240) ? GPIO_PIN_SET : GPIO_PIN_RESET);
                            }
                        }
                    }
                }
            }
        }
    }
}
