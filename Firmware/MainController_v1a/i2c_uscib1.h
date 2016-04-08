/* i2c_uscib1.h
 * Created 4/7/2016 by Nishant Pol and Jaime Chu
 * 18-578 Mechatronics
 * MSP430F5521
 * Main Controller Code
 *
 *
 */

#ifndef I2C_USCIB1_H_
#define I2C_USCIB1_H_

#include <msp430.h>
#include "utils.h"
#include "err_warn_codes.h"

#define I2C_BUF_SIZE 16
struct I2C_data_struct{
	uint8_t tx_bytes[I2C_BUF_SIZE];	//Bytes to send to slave
	uint8_t rx_bytes[I2C_BUF_SIZE];	//Bytes recieved from slave
	uint8_t tx_ptr;					//Index of byte to transmit next
	uint8_t rx_ptr;					//Index of next available empty space for recieved byte
	uint8_t num_bytes;				//Number of bytes to recieve/transmit
	uint8_t in_use_flag;			//Indicates if data has not been processed by main State machine
	uint8_t data_ready;				//Indicates that rx_bytes is valid
	uint8_t address;					//Recipient address
};
extern volatile struct I2C_data_struct I2C_data;

#define I2C_TXINT_ENABLE        (UCB1IE |= UCTXIE)
#define I2C_TXINT_DISABLE       (UCB1IE &= ~UCTXIE)
#define I2C_RXINT_ENABLE        (UCB1IE |= UCRXIE)
#define I2C_RXINT_DISABLE       (UCB1IE &= ~UCRXIE)

#define I2C_ADDR_MOTOR_IO		0x41

void i2c_uscib1_setup(void);
void init_I2C_transac(uint8_t *tx_bytes, uint8_t num_bytes, uint8_t address);
void end_I2C_transac(void);
uint8_t is_I2C_busy(void);
uint8_t is_I2C_rx_ready(void);

#endif /* I2C_USCIB1_H_ */
