/* i2c_uscib1.c
 * Created 4/7/2016 by Nishant Pol and Jaime Chu
 * 18-578 Mechatronics
 * MSP430F5521
 * Main Controller Code
 *
 * I2C Pinout
 * Uses USCIB0
 * P4.1: SDA
 * P4.2: SCL
 * P1.4: I2C interrupt (pullup)
 *
 * I2C bus devices and addresses (7-bit)
 * 0x41: PCA9536 I/O expander
 * 0x50: LCD screen
 *
 */
#include "i2c_uscib1.h"

volatile struct I2C_data_struct I2C_data = {
	.tx_bytes = {0},
	.rx_bytes = {0},
	.tx_ptr = 0,
	.rx_ptr = 0,
	.num_bytes = 0,
	.in_use_flag = 0,
	.data_ready = 0,
	.address = 0
};

/* Setup i2c for peripheral devices
 * Master, 100kHz baud rate
 */
void i2c_uscib1_setup(void){
	//Toggle SCL to release bus
	P4DIR |= BIT2;
	while(!(P4IN &= BIT1)){
		P4OUT ^= BIT2;
	}
	P4DIR &= ~BIT2;
	//Assign I2C pins
	P4SEL |= BIT1 + BIT2;	//SDA and SCL mapped to default PMAP configuration
	UCB1CTL1 |= UCSWRST;	//Enable SW reset
	UCB1CTL0 |= UCMST + UCMODE_3 + UCSYNC;	//I2C Master, synchronous mode
	UCB1CTL1 = UCSSEL_2 + UCSWRST;            // Use SMCLK, keep SW reset
	UCB1BR0 = 0xf4;                             // fSCL = SMCLK/500 = ~50kHz
	UCB1BR1 = 0x02;
	UCB1CTL1 &= ~UCSWRST;
	UCB1IE |= UCNACKIE+UCALIE;	//Enable NACK and Arb lost interrupts
}

/* SM loads I2C datastructure, start transaction
 * tx_bytes: array of bytes to send
 * num_bytes: number of bytes to send
 * address: slave address
 */
void init_I2C_transac(uint8_t *tx_bytes, uint8_t num_bytes, uint8_t address){
	I2C_data.in_use_flag = 1;
	uint8_t i;
	for(i = 0; i < num_bytes; i++){			//Copy Tx data to I2C datastructure
		I2C_data.tx_bytes[i] = tx_bytes[i];
	}
	I2C_data.num_bytes = num_bytes;			//Copy Transaction size to I2C datastructure
	I2C_data.tx_ptr = 0;					//Reset pointers
	I2C_data.rx_ptr = 0;
	I2C_data.address = address;
	UCB1I2CSA = address;
	UCB1CTL1 |= UCTXSTT + UCTR;	//Start Master Transmit
	I2C_TXINT_ENABLE;	//Enable I2C Interrupts
	return;
}

/* SM releases I2C datastructure without copying received data
 *
 */
void end_I2C_transac(void){
	I2C_TXINT_DISABLE;
	I2C_RXINT_DISABLE;
	I2C_data.data_ready = 0;
	I2C_data.in_use_flag = 0;						//Release I2C datastructure
	return;
}

/* SM can poll I2C with this function to see if I2C is in use
 *
 */
uint8_t is_I2C_busy(void){
	return I2C_data.in_use_flag;
}

uint8_t is_I2C_rx_ready(void){
	return I2C_data.data_ready;
}

