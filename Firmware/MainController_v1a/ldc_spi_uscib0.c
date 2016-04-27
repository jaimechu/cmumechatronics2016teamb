/* ldc_spi_uscib0.c
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
 * SPI Pinout to LDC Inductive sensors
 * Uses USCIB0
 * P3.2: SCK
 * P3.1: MISO
 * P3.0: MOSI
 * P3.6: LDC_CS0 (active low)
 * P3.5: LDC_CS1 (active low)
 * P7.0: LDC_CS2 (active low)
 */
#include "ldc_spi_uscib0.h"

/* SPI Datastructure
 * Stores data and config relevant to current transaction
 */
volatile struct LDC_SPI_data_struct LDC_SPI_data = {
	.tx_bytes = {0},
	.rx_bytes = {0},
	.tx_ptr = 0,
	.rx_ptr = 0,
	.num_bytes = 0,
	.in_use_flag = 0,
	.data_ready = 0,
	.brd = 0
};

/* Setup SPI as Master on USCI_B0
 * Clock Source: SMCLK (25MHz)/
 * idle: When idle, clock is low (0) or high (1)
 * edge: write bus on idle-active (0) or active-idle (1) edge
 */
void LDC_SPI_setup(uint8_t idle, uint8_t edge){
	//Hold USCI in reset for setup
	UCB0CTL1 |= UCSWRST;

	UCB0CTL0 = (edge<<7) |	//Clock phase
			   (idle<<6) | 	//Clock polarity
			   UCMSB     |	//MSB first
			   UCMST     |	//Master
			   UCMODE_0;	//3-pin SPI

	UCB0CTL1 = UCSSEL_2  |	//Source from SMCLK
			   UCSWRST;		//Keep USCI in reset

	UCB0BR0 = 250;			//run at 1MHz

	//Enable use of SPI pins MOSI, MISO, SCK
	P3SEL |= BIT0 + BIT1 + BIT2;

	//CS on P3.6, set output high (disabled)
	P3DIR |= BIT6;
	LDC0_SPI_CS_DEASSERT;

	//CS on P3.5, set output high (disabled)
	P3DIR |= BIT5;
	LDC1_SPI_CS_DEASSERT;

	//CS on P7.0, set output high (disabled)
	//P7DIR |= BIT0;
	//LDC2_SPI_CS_DEASSERT;

	UCB0CTL1 &= ~UCSWRST;	//Release USCI from Reset
	return;
}

/* SM loads SPI datastructure, start transaction
 * tx_bytes: array of bytes to send
 * num_bytes: number of bytes to send
 * brd: Board to access (0,1,2)
 */
void init_LDC_SPI_transac(uint8_t *tx_bytes, uint8_t num_bytes, uint8_t brd){
	LDC_SPI_data.in_use_flag = 1;
	uint8_t i;
	for(i = 0; i < num_bytes; i++){			//Copy Tx data to SPI datastructure
		LDC_SPI_data.tx_bytes[i] = tx_bytes[i];
	}
	LDC_SPI_data.num_bytes = num_bytes;			//Copy Transaction size to SPI datastructure
	LDC_SPI_data.tx_ptr = 0;					//Reset pointers
	LDC_SPI_data.rx_ptr = 0;
	if(brd == 0){
		LDC0_SPI_CS_ASSERT;
		LDC_SPI_data.brd = 0;
	} else if(brd == 1){
		LDC1_SPI_CS_ASSERT;
		LDC_SPI_data.brd = 1;
	/*} else if(brd == 2){
		LDC2_SPI_CS_ASSERT;
		LDC_SPI_data.brd = 2;*/
	} else {
		issue_warning(WARN_ILLEGAL_LDC_SPI_CS);
	}
	LDC_SPI_TXINT_ENABLE;	//Enable SPI Interrupts
	LDC_SPI_RXINT_ENABLE;
	return;
}

/* SM gets recieved data, releases SPI datastructure
 * rx_data: buffer to store recieved data
 */
uint8_t get_LDC_SPI_rx_data(uint8_t *rx_data){
	uint8_t i;
	for(i = 0; i < LDC_SPI_data.num_bytes; i++){	//Copy data
		rx_data[i] = LDC_SPI_data.rx_bytes[i];
	}
	LDC_SPI_TXINT_DISABLE;
	LDC_SPI_RXINT_DISABLE;
	LDC_SPI_data.data_ready = 0;
	LDC_SPI_data.in_use_flag = 0;						//Release SPI datastructure
	return LDC_SPI_data.num_bytes;
}

/* SM releases SPI datastructure without copying received data
 *
 */
void end_LDC_SPI_transac(void){
	LDC_SPI_TXINT_DISABLE;
	LDC_SPI_RXINT_DISABLE;
	LDC_SPI_data.data_ready = 0;
	LDC_SPI_data.in_use_flag = 0;						//Release SPI datastructure
	return;
}

/* SM can poll SPI with this function to see if SPI is in use
 *
 */
uint8_t is_LDC_spi_busy(void){
	return LDC_SPI_data.in_use_flag;
}

uint8_t is_LDC_spi_rx_ready(void){
	return LDC_SPI_data.data_ready;
}
