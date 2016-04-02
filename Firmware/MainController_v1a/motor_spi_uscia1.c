/* motor_spi_uscia1.c
 * Created 11/25/15 by Nishant Pol
 * 18-578 Mechatronics
 * Mechatronics 18-578
 * MSP430F5521
 * Main Controller Code
 *
 * Revision History
 * 6/10/15: Nishant Pol
 * - Initial release for eInsights Live Pressure Sensor
 * 11/25/15: Nishant Pol
 * - Repurposed code for RoboCapstone Bus Prototype
 * 1/24/16: Nishant Pol
 * - Updated CS to reflect new pinout
 * 3/30/16: Nishant Pol and Jaime Chu
 * - Repurposed code for Mechatronics
 * - Modified code for MSP430F5521 (previously MSP430G2553)
 *
 * SPI Pinout to Motors
 * Uses USCIA1
 * P4.0: SCK
 * P4.3: MISO
 * P4.4: MOSI
 * PJ.2: Motor 1 CS (active low)
 * PJ.0: Motor 2 CS (active low)
 * P4.6: Motor 3 CS (active low)
 */
#include "motor_spi_uscia1.h"

/* SPI Datastructure
 * Stores data and config relevant to current transaction
 */

volatile struct MOTOR_SPI_data_struct MOTOR_SPI_data = {
	.tx_bytes = {0},
	.rx_bytes = {0},
	.tx_ptr = 0,
	.rx_ptr = 0,
	.num_bytes = 0,
	.in_use_flag = 0,
	.data_ready = 0
};


/* Setup SPI as Master on USCI_A1
 * Clock Source: SMCLK (25MHz)/
 * idle: When idle, clock is low (0) or high (1)
 * edge: write bus on idle-active (0) or active-idle (1) edge
 */

void Motor_SPI_setup(uint8_t idle, uint8_t edge){
	//Hold USCI in reset for setup
	UCA1CTL1 |= UCSWRST;

	UCA1CTL0 = (edge<<7) |	//Clock phase
			   (idle<<6) | 	//Clock polarity
			   UCMSB     |	//MSB first
			   UCMST     |	//Master
			   UCMODE_0;	//3-pin SPI

	UCA1CTL1 = UCSSEL_2  |	//Source from SMCLK
			   UCSWRST;		//Keep USCI in reset

	UCA1BR0 = 16;			//No divider, run at ~1MHz

	//Enable use of SPI pins MOSI, MISO, SCK
	P4SEL |= BIT0 | BIT3 | BIT4;

	//CS1 on PJ.2, set output high (disabled)
	PJDIR |= BIT2;
	MOTOR1_SPI_CS_DEASSERT;
	//CS2 on PJ.0, set output high (disabled)
	PJDIR |= BIT0;
	MOTOR2_SPI_CS_DEASSERT;
	//CS3 on P4.6, set output high (disabled)
	P4DIR |= BIT6;
	MOTOR3_SPI_CS_DEASSERT;

	UCA1CTL1 &= ~UCSWRST;	//Release USCI from Reset
	return;
}


/* SM loads SPI datastructure, start transaction
 *
 */
/*
void init_SPI_transac(uint8_t *tx_bytes, uint8_t num_bytes){
	//TODO: Check if datastructure already in use
	SPI_data.in_use_flag = 1;
	uint8_t i;
	for(i = 0; i < num_bytes; i++){			//Copy Tx data to SPI datastructure
		SPI_data.tx_bytes[i] = tx_bytes[i];
	}
	SPI_data.num_bytes = num_bytes;			//Copy Transaction size to SPI datastructure
	SPI_data.tx_ptr = 0;					//Reset pointers
	SPI_data.rx_ptr = 0;
	SPI_CS_ASSERT;
	IE2 |= UCB0TXIE | UCB0RXIE;				//Enable SPI interrupts


	return;
}
*/
/* SM gets recieved data, releases SPI datastructure
 *
 */
/*
uint8_t get_SPI_rx_data(uint8_t *rx_data){
	uint8_t i;
	for(i = 0; i < SPI_data.num_bytes; i++){	//Copy data
		rx_data[i] = SPI_data.rx_bytes[i];
	}
	IE2 &= ~(UCB0TXIE | UCB0RXIE);				//Disable SPI interrupts
	SPI_data.data_ready = 0;
	SPI_data.in_use_flag = 0;						//Release SPI datastructure
	return SPI_data.num_bytes;
}
*/

/* SM releases SPI datastructure without copying received data
 *
 */
/*
void end_SPI_transac(void){
	IE2 &= ~(UCB0TXIE | UCB0RXIE);				//Disable SPI interrupts
	SPI_data.data_ready = 0;
	SPI_data.in_use_flag = 0;						//Release SPI datastructure
	return;
}
*/

/* SM can poll SPI with this function to see if SPI is in use
 *
 */
/*
uint8_t is_spi_busy(void){
	return SPI_data.in_use_flag;
}

uint8_t is_spi_rx_ready(void){
	return SPI_data.data_ready;
}
*/
