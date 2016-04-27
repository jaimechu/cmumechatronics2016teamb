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
 * P4.5: MISO
 * P4.4: MOSI
 * P2.4: Motor 1 CS (active low)
 * P2.6: Motor 2 CS (active low)
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
	.data_ready = 0,
	.motor = 0
};


/* Setup SPI as Master on USCI_A1
 * Clock Source: SMCLK (25MHz)/
 * idle: When idle, clock is low (0) or high (1)
 * edge: write bus on idle-active (0) or active-idle (1) edge
 * LSB first
 */

void MOTOR_SPI_setup(uint8_t idle, uint8_t edge){
	//Hold USCI in reset for setup
	UCA1CTL1 |= UCSWRST;
	UCA1CTL0 = (edge<<7) |	//Clock phase
			   (idle<<6) | 	//Clock polarity
			   UCMST     |	//Master
			   UCMODE_0  |	//3-pin SPI
			   UCSYNC;

	UCA1CTL1 = UCSSEL_2  |	//Source from SMCLK
			   UCSWRST;		//Keep USCI in reset

	UCA1BR0 = 250;			//No divider, run at ~1MHz

	//Enable use of SPI pins MOSI, MISO, SCK
	P4SEL |= BIT0 | BIT4 | BIT5;

	//CS1 on P2.4, set output high (disabled)
	P2DIR |= BIT4;
	MOTOR1_SPI_CS_DEASSERT;
	//CS2 on P2.6, set output high (disabled)
	P2DIR |= BIT6;
	MOTOR2_SPI_CS_DEASSERT;
	//CS3 on P4.6, set output high (disabled)
	P4DIR |= BIT6;
	MOTOR3_SPI_CS_DEASSERT;

	UCA1CTL1 &= ~UCSWRST;	//Release USCI from Reset
	return;
}


void init_motor_SPI_transac(uint16_t CFG_REG, uint8_t motor){
	//TODO: Check if datastructure already in use
	MOTOR_SPI_data.in_use_flag = 1;
	MOTOR_SPI_data.tx_bytes[0] = (uint8_t)((CFG_REG >> 8) & 0xFF);
	MOTOR_SPI_data.tx_bytes[1] = (uint8_t)(CFG_REG & 0xFF);
	MOTOR_SPI_data.num_bytes = 2;
	MOTOR_SPI_data.tx_ptr = 0;					//Reset pointers
	MOTOR_SPI_data.rx_ptr = 0;

	if(motor == 1){
		MOTOR1_SPI_CS_ASSERT;
		MOTOR_SPI_data.motor = 1;
	} else if (motor == 2) {
		MOTOR2_SPI_CS_ASSERT;
		MOTOR_SPI_data.motor = 2;
	} else if (motor == 3) {
		MOTOR3_SPI_CS_ASSERT;
		MOTOR_SPI_data.motor = 3;
	}
	MOTOR_SPI_TXINT_ENABLE;
	MOTOR_SPI_RXINT_ENABLE;


	return;
}

/* SM gets recieved data, releases SPI datastructure
 *
 */

uint16_t get_motor_SPI_rx_data(void){
	uint16_t DIA_REG;
	DIA_REG = MOTOR_SPI_data.rx_bytes[0] | (MOTOR_SPI_data.rx_bytes[1] << 8);
	MOTOR_SPI_TXINT_DISABLE;
	MOTOR_SPI_RXINT_DISABLE;
	MOTOR_SPI_data.data_ready = 0;
	MOTOR_SPI_data.in_use_flag = 0;						//Release SPI datastructure
	return DIA_REG;
}


/* SM releases SPI datastructure without copying received data
 *
 */

void end_SPI_transac(void){
	MOTOR_SPI_TXINT_DISABLE;
	MOTOR_SPI_RXINT_DISABLE;
	MOTOR_SPI_data.data_ready = 0;
	MOTOR_SPI_data.in_use_flag = 0;						//Release SPI datastructure
	return;
}


/* SM can poll SPI with this function to see if SPI is in use
 *
 */
uint8_t is_motor_spi_busy(void){
	return MOTOR_SPI_data.in_use_flag;
}

uint8_t is_motor_spi_rx_ready(void){
	return MOTOR_SPI_data.data_ready;
}

