/*
 * motor_spi_uscia1.h
 * Created 6/10/15 by Nishant Pol
 * SPI Pinout to Motors
 * Uses USCIA1
 * P4.0: SCK
 * P4.3: MISO
 * P4.4: MOSI
 * PJ.2: Motor 1 CS (active low)
 * PJ.0: Motor 2 CS (active low)
 * P4.6: Motor 3 CS (active low)
 */

#ifndef MOTOR_SPI_USCIA1_H_
#define MOTOR_SPI_USCIA1_H_

#include <msp430.h>
#include "utils.h"

/* SPI Macros for CS */
#define MOTOR_SPI_BUF_SIZE 8
#define MOTOR1_SPI_CS_ASSERT		(PJOUT &= ~BIT2)
#define MOTOR2_SPI_CS_ASSERT		(PJOUT &= ~BIT0)
#define MOTOR3_SPI_CS_ASSERT		(P4OUT &= ~BIT6)
#define MOTOR1_SPI_CS_DEASSERT 		(PJOUT |= BIT2)
#define MOTOR2_SPI_CS_DEASSERT 		(PJOUT |= BIT0)
#define MOTOR3_SPI_CS_DEASSERT 		(P4OUT |= BIT6)
#define MOTOR_SPI_TXINT_ENABLE        (UCA1IE |= UCTXIE)
#define MOTOR_SPI_TXINT_DISABLE       (UCA1IE &= ~UCTXIE)
#define MOTOR_SPI_RXINT_ENABLE        (UCA1IE |= UCRXIE)
#define MOTOR_SPI_RXINT_DISABLE       (UCA1IE &= ~UCRXIE)

struct MOTOR_SPI_data_struct{
	uint8_t tx_bytes[MOTOR_SPI_BUF_SIZE];	//Bytes to send to slave
	uint8_t rx_bytes[MOTOR_SPI_BUF_SIZE];	//Bytes recieved from slave
	uint8_t tx_ptr;					//Index of byte to transmit next
	uint8_t rx_ptr;					//Index of next available empty space for recieved byte
	uint8_t num_bytes;				//Number of bytes to recieve/transmit
	uint8_t in_use_flag;			//Indicates if data has not been processed by main State machine
	uint8_t data_ready;				//Indicates that rx_bytes is valid
	uint8_t motor; 					//Chip Select number
};
extern volatile struct MOTOR_SPI_data_struct MOTOR_SPI_data;

void MOTOR_SPI_setup(uint8_t idle, uint8_t edge);
void init_MOTOR_SPI_transac(uint8_t *tx_bytes, uint8_t num_bytes, uint8_t motor);
uint8_t get_MOTOR_SPI_rx_data(uint8_t *rx_data);
void end_MOTOR_SPI_transac(void);
uint8_t is_MOTOR_spi_busy(void);
uint8_t is_MOTOR_spi_rx_ready(void);


#endif /* SPI_H_ */
