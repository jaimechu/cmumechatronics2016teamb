/*
 * ldc_spi_uscib0.h
 * Created 6/10/15 by Nishant Pol
 */

#ifndef LDC_SPI_USCIB0_H_
#define LDC_SPI_USCIB0_H_

#include <msp430.h>
#include "utils.h"

/* SPI Macros for CS */
#define LDC_SPI_BUF_SIZE 8
#define LDC0_SPI_CS_ASSERT			(P3OUT &= ~BIT6)
#define LDC1_SPI_CS_ASSERT			(P3OUT &= ~BIT5)
#define LDC0_SPI_CS_DEASSERT 		(P3OUT |= BIT6)
#define LDC1_SPI_CS_DEASSERT 		(P3OUT |= BIT5)
#define LDC_SPI_TXINT_ENABLE        (UCB0IE |= UCTXIE)
#define LDC_SPI_TXINT_DISABLE       (UCB0IE &= ~UCTXIE)
#define LDC_SPI_RXINT_ENABLE        (UCB0IE |= UCRXIE)
#define LDC_SPI_RXINT_DISABLE       (UCB0IE &= ~UCRXIE)

struct LDC_SPI_data_struct{
	uint8_t tx_bytes[LDC_SPI_BUF_SIZE];	//Bytes to send to slave
	uint8_t rx_bytes[LDC_SPI_BUF_SIZE];	//Bytes recieved from slave
	uint8_t tx_ptr;					//Index of byte to transmit next
	uint8_t rx_ptr;					//Index of next available empty space for recieved byte
	uint8_t num_bytes;				//Number of bytes to recieve/transmit
	uint8_t in_use_flag;			//Indicates if data has not been processed by main State machine
	uint8_t data_ready;				//Indicates that rx_bytes is valid
};
extern volatile struct LDC_SPI_data_struct LDC_SPI_data;

void LDC_SPI_setup(uint8_t idle, uint8_t edge);
void init_LDC_SPI_transac(uint8_t *tx_bytes, uint8_t num_bytes);
uint8_t get_LDC_SPI_rx_data(uint8_t *rx_data);
void end_LDC_SPI_transac(void);
uint8_t is_LDC_spi_busy(void);
uint8_t is_LDC_spi_rx_ready(void);


#endif /* SPI_H_ */
