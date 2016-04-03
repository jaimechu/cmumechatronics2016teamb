/* ldc_1000.h
 * Created 11/25/15 by Nishant Pol and Jaime Chu
 * 18-578 Mechatronics
 * MSP430F5521
 * Main Controller Code
 *
 * LDC Registers
 *
 */

#ifndef LDC1000_H_
#define LDC1000_H_
#include <msp430.h>
#include "utils.h"

#define LDC_DEV_ID		0x00
#define LDC_RP_MAX		0x01
#define LDC_RP_MIN		0x02
#define LDC_WDT_F		0x03
#define LDC_CNFG		0x04
#define LDC_CLK_CFG		0x05
#define LDC_CMP_TH_H_LSB	0x06
#define LDC_CMP_TH_H_MSB	0x07
#define LDC_CMP_TH_L_LSB	0x08
#define LDC_CMP_TH_L_MSB	0x09
#define LDC_INTB_CNFG	0x0A
#define LDC_PWR_CNFG	0x0B
#define LDC_STATUS		0x20
#define OSC_DEAD		BIT7
#define DRDY			BIT6
#define WAKEUP			BIT5
#define COMPARATOR		BIT4

#define LDC_PROX_LSB	0x21
#define LDC_PROX_MSB	0x22
#define LDC_FRQ_LSB		0x23
#define LDC_FRQ_MID		0x24
#define LDC_FRQ_MSB		0x25

#define LDC_DEV_ID_VALUE	0x80

#endif /* LDC1000_H_ */
