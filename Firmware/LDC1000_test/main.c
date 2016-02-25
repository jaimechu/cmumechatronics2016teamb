/* main.c
 * Created 11/25/15 by Nishant Pol
 * CMU Mechatronics Team B
 * CAN Bus Prototype
 *
 * Code Composer v6
 * MSP430G2553IPW20
 *
 * Revision History
 * 11/25/15: Nishant Pol
 * - Initial release for Robotics Capstone
 * 1/24/15: Nishant Pol
 * - Added LED blink loop for HW testing, then removed loop
 * - Added SPI commands to check communication to MCP2515
 * 2/9/16: Nishant Pol
 * - Added Debug UART Interface
 * 2/27/16: Nishant Pol
 * - Ported to Mechatronics project for LDC1000 tesing over SPI
 */
#include <msp430.h> 
#include "utils.h"
#include "clock.h"
#include "spi.h"
#include "uart.h"
#include <string.h>

uint16_t ldc_get_proximity(void);
void ldc_setup(void);
void ldc_write_reg(uint8_t reg_addr, uint8_t data);
void ldc_read_reg_multiple(uint8_t reg_addr, uint8_t *buf, uint8_t num_regs);
uint8_t ldc_read_reg(uint8_t reg_addr);

/** Debug task macros and globals **/
void debug_task(void);
void debug_rx_task(void);

inline void led_on(uint8_t led);
inline void led_off(uint8_t led);
inline uint8_t button_get(uint8_t *buf);
inline uint8_t debug_mcp2515_read_reg(uint8_t *debug_cmd_buf,uint8_t *response_buf);
inline uint8_t debug_mcp2515_write_reg(uint8_t *debug_cmd_buf,uint8_t *response_buf);
inline uint8_t ascii2hex_byte(uint8_t high_char, uint8_t low_char);
inline void hex2ascii_byte(uint8_t data, uint8_t *high_char, uint8_t *low_char);
inline uint8_t debug_can_rx(uint8_t *response_buf);


#define DEBUG_CMD_BUF_SIZE 32
uint8_t debug_cmd_buf[DEBUG_CMD_BUF_SIZE];
uint8_t debug_cmd_buf_ptr = 0;
/** END Debug task macros and globals **/


/*
 * main.c
 */
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
    P1DIR |= BIT0;
    P1OUT &= ~BIT0;
    setup_clock();
    USCIB0_SPI_setup(0,1);		//Setup SPI, idle low, out on falling edge
    setup_uart();
    __enable_interrupt();
    uint8_t resp = 0;
    ldc_setup();

	while(1){
		resp = ldc_read_reg(0x00);
		if(resp != 0x80){
			P1OUT |= BIT0;
		}
		ldc_get_proximity();
	}
}

/** Main application task functions **/
uint8_t ldc_read_reg(uint8_t reg_addr){
	uint8_t buf[2] = {0x80,0x00};
	buf[0] |= reg_addr;
	init_SPI_transac(buf, 2);
	while(!is_spi_rx_ready());
	get_SPI_rx_data(buf);
	return buf[1];
}

void ldc_read_reg_multiple(uint8_t reg_addr, uint8_t *buf, uint8_t num_regs){
	buf[0] = 0x80|reg_addr;
	init_SPI_transac(buf,num_regs+1);
	while(!is_spi_rx_ready());
	get_SPI_rx_data(buf);
	return;
}

void ldc_write_reg(uint8_t reg_addr, uint8_t data){
	uint8_t buf[2] = {0x00,0x00};
	buf[0] = reg_addr;
	buf[1] = data;
	init_SPI_transac(buf, 2);
	while(!is_spi_rx_ready());
	end_SPI_transac();
	return;
}

void ldc_setup(void){
	uint8_t resp = 0;
	//Check ID register
	resp = ldc_read_reg(0x00);
	if(resp != 0x80){
		P1OUT |= BIT0;
		while(1);
	}
	//Write Rp max/min values
	ldc_write_reg(0x01,0x00);	//RP_MAX
	ldc_write_reg(0x02,0x3f);	//RP_MIN
	//Watchdog frequency
	ldc_write_reg(0x03,179);
	//Configuration
	ldc_write_reg(0x04,BIT4|BIT1|BIT0);	//Amplitude=4V, Response time = 384
	//Clock configuration
	ldc_write_reg(0x05,BIT1|BIT0);	//Enable crystal
	//INTB configuration
	ldc_write_reg(0x0A,BIT2);	//INTB indicates data ready
	//Power configuration
	ldc_write_reg(0x0B,BIT0);	//Active mode
	return;
}

uint16_t ldc_get_proximity(void){
	uint8_t buf[8];
	ldc_read_reg_multiple(0x20,buf,6);
	return (buf[3]<<8)|buf[2];	//Proximity
}


/** END main application task functions **/


/* Create numerical byte from hex ascii characters
 * high_char: ascii code for high nibble
 * low_char: ascii code for low nibble
 * returns numerical byte value
 */
inline uint8_t ascii2hex_byte(uint8_t high_char, uint8_t low_char){
	uint8_t num = 0;
	if(('0'<= high_char) && (high_char <= '9')){
		num = (high_char-'0')<<4;
	} else if(('A' <= high_char) && (high_char <= 'F')){
		num = (high_char-'A'+10)<<4;
	} else if(('a' <= high_char) && (high_char <= 'f')){
		num = (high_char-'a'+10)<<4;
	}
	if(('0'<= low_char) && (low_char <= '9')){
		num |= (low_char-'0');
	} else if(('A' <= low_char) && (low_char <= 'F')){
		num |= (low_char-'A'+10);
	} else if(('a' <= low_char) && (low_char <= 'f')){
		num |= (low_char-'a'+10);
	}
	return num;
}

/* Create ascii character representation of numerical byte
 * data: number to be converted
 * high_char: pointer to high nibble character
 * low_char: pointer to low nibble character
 */
inline void hex2ascii_byte(uint8_t data, uint8_t *high_char, uint8_t *low_char){
	uint8_t upper = (data>>4)&0xf;
	uint8_t lower = (data)&0xf;
	if(upper <= 9){
		*high_char = upper+'0';
	} else if((10 <= upper) && (upper <= 15)){
		*high_char = upper-10+'A';
	}
	if(lower <= 9){
		*low_char = lower+'0';
	} else if((10 <= lower) && (lower <= 15)){
		*low_char = lower-10+'A';
	}
	return;
}


/** END Debug Task functions **/

/* SPI/UART Rx Interrupt Handler
 * SPI Rx: puts most recent character in SPI datastructure
 * UART Rx:
 */
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void){
	if((IFG2 & UCA0RXIFG) && (IE2 & UCA0RXIE)){	//UART Rxbuf full interrupt
		//Get byte and clear interrupt
		UART_data.rx_bytes[UART_data.rx_head] = UCA0RXBUF;
		UART_data.rx_head++;
		//Wraparound condition
		if(UART_data.rx_head >= UART_RX_BUF_SIZE){
			UART_data.rx_head = 0;
		}
		//if(UART_data.rx_head == UART_data.rx_tail)
			//TODO: Log error: buffer full
	} else
	if((IFG2 & UCB0RXIFG) && (IE2 & UCB0RXIE)){	//SPI Rxbuf full interrupt
		SPI_data.rx_bytes[SPI_data.rx_ptr] = UCB0RXBUF;	//Get latest byte from HW
		SPI_data.rx_ptr++;								//Flag reset with buffer read
		if(SPI_data.rx_ptr >= SPI_data.num_bytes){		//Done reading data
			SPI_CS_DEASSERT;							//Disable CS and disable interrupt
			IE2 &= ~UCB0RXIE;
			SPI_data.data_ready = 1;
		}
	} else {
		//TODO: Log Error, should never go here
		while(1);
	}
}

/* SPI/UART Tx Interrupt Handler
 * SPI Tx: Transmits next character in SPI datastructure
 * UART Tx:
 */
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void){
	if((IFG2 & UCA0TXIFG) && (IE2 & UCA0TXIE)){			//UART Txbuf ready interrupt
		//Load data and clear interrupt
		UCA0TXBUF = UART_data.tx_bytes[UART_data.tx_tail];
		UART_data.tx_tail++;
		//Wraparound condition
		if(UART_data.tx_tail >= UART_TX_BUF_SIZE){
			UART_data.tx_tail = 0;
		}
		//Disable Tx interrupt if last byte in buffer has been transmitted
		if(UART_data.tx_tail == UART_data.tx_head){
			disable_uart_txint();
		}
	} else
	if((IFG2 & UCB0TXIFG) && (IE2 & UCB0TXIE)){	//SPI Txbuf ready interrupt
		UCB0TXBUF = SPI_data.tx_bytes[SPI_data.tx_ptr];	//Load next byte into HW buffer
		SPI_data.tx_ptr++;								//Flag reset with buffer write
		if(SPI_data.tx_ptr >= SPI_data.num_bytes){		//Done transmitting data
			IE2 &= ~UCB0TXIE;							//Disable Tx interrupt
		}
	} else {
		//TODO: Log Error, should never go here
		while(1);
	}
}
