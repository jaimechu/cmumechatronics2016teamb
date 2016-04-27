/*
 * err_warn_codes.h
 * Created 3/29/16 by Nishant Pol and Jaime Chu
 * Mechatronics 18-578
 * MSP430F5521
 * Main Controller Code
 *
 * Code Composer v6
 */

#ifndef ERR_WARN_CODES_H_
#define ERR_WARN_CODES_H_

/* General Errors */
#define ERR_NONE				0x0000
#define ERR_TOO_MANY_WARNS		0x0001
#define ERR_XT2_FAULT			0x0002
#define ERR_XT1_FAULT			0x0003
#define ERR_DCO_FAULT			0x0004
#define ERR_FLASH_VIOL			0x0005
#define ERROR_TEST				0x0006

/* Application specific Errors */

/* General Warnings */
#define WARN_NONE				0x0000
#define WARN_DBG_BUFF_OVERRUN	0x0001
#define WARN_DBG_TX_BUF_FULL1	0x0002
#define WARN_DBG_TX_BUF_FULL2	0x0003
#define WARN_DBG_RX_BUF_FULL	0x0004
#define WARN_USCIA0_INT_ILLEGAL_FLAG	0x0005
#define WARN_USCIA1_INT_ILLEGAL_FLAG	0x0006
#define WARN_USCIB0_INT_ILLEGAL_FLAG	0x0007
#define WARN_USCIB1_INT_ILLEGAL_FLAG	0x0008
#define WARN_ILLEGAL_MOTOR_SPI_CS2		0x0022
#define WARN_NMI				0x0009
#define WARN_RST_BOR			0x0010
#define WARN_RST_RSTNMI			0x0011
#define WARN_RST_DOBOR			0x0012
#define WARN_RST_LPM5WU			0x0013
#define WARN_RST_SECYV			0x0014
#define WARN_RST_SVSL			0x0015
#define WARN_RST_SVSH			0x0016
#define WARN_RST_SVMLOVP		0x0017
#define WARN_RST_SVMHOVP		0x0018
#define WARN_RST_DOPOR			0x0019
#define WARN_RST_WDTTO			0x001A
#define WARN_RST_WDTKEY			0x001B
#define WARN_RST_KEYV			0x001C
#define WARN_RSTFLLUL			0x001D
#define ERR_RST_PERF			0x001E
#define WARN_RST_PMM_KEY		0x001F
#define WARN_TEST				0x0020
#define WARN_ERROR_LOG_CLEARED 0x0021

/* Application specific Warnings */
#define WARN_ILLEGAL_ADC_SM_STATE	0x8110
#define WARN_ILLEGAL_LDC_SM_STATE	0x8111
#define WARN_ILLEGAL_MASS_SM_STATE 	0x8112
#define WARN_ILLEGAL_OPT_SM_STATE 	0x8113
#define WARN_ILLEGAL_MOT_SM_STATE   0x8114
#define WARN_ILLEGAL_I2C_SM_STATE 	0x8115
#define WARN_ILLEGAL_CHIM_SM_STATE  0x8116
#define WARN_ILLEGAL_COMPACT_SM_STATE  0x8117
#define WARN_ILLEGAL_BIN_SM_STATE  0x8118
#define WARN_ILLEGAL_BIN_REQUEST_SM_STATE 0x8119
#define WARN_ILLEGAL_BIN_REQUEST_SM_STATE2 0x811A
#define WARN_ILLEGAL_STEP_SM_STATE 0x811B
#define WARN_ILLEGAL_BIN_FULL_SM_STATE  0x811C
#define WARN_ILLEGAL_BIN_ACC_SM_STATE  0x811D

#define WARN_LOW_3V3				0x8200
#define WARN_HIGH_3V3				0x8201
#define WARN_LOW_5V0				0x8202
#define WARN_HIGH_5V0				0x8203
#define WARN_LOW_6VA				0x8204
#define WARN_HIGH_6VA				0x8205
#define WARN_LOW_12V				0x8206
#define WARN_HIGH_12V				0x8207
#define WARN_LOW_MCU_TEMP			0x820E
#define WARN_HIGH_MCU_TEMP			0x820F
#define ESTOP_ACTIVATED				0x8214
#define WARN_LDC_SPI_HANG0			0x8300
#define WARN_LDC_SPI_HANG1			0x8301
#define WARN_LDC_SPI_HANG2			0x8302
#define WARN_ILLEGAL_LDC_SPI_CS		0x8303
#define WARN_ILLEGAL_LDC_SPI_CS2	0x8304
#define WARN_LDC0_OSC_DEAD			0x8305
#define WARN_LDC1_OSC_DEAD			0x8306
#define WARN_LDC2_OSC_DEAD			0x8307
#define WARN_I2C_ARB_LOST			0x8400
#define WARN_I2C_NACK				0x8401
#define WARN_I2C_ILLEGAL_INTVECTOR	0x8402
#define WARN_NO_AVAIL_BUF			0x8410
#define WARN_OPT_TASK_STOP_ILLEGAL_BUF 		0x8411
#define WARN_OPT_TASK_STOP_ILLEGAL_BUF2 	0x8412
#define WARN_MASS_TASK_STOP_ILLEGAL_BUF 	0x8413
#define WARN_MASS_TASK_STOP_ILLEGAL_BUF2 	0x8414
#define WARN_MASS_TASK_STOP_ILLEGAL_BUF3 	0x8415
#define WARN_LDC_TASK_STOP_ILLEGAL_BUF 		0x8416
#define WARN_LDC_TASK_STOP_ILLEGAL_BUF2 	0x8417
#define WARN_ILLEGAL_COMPACT_POS	0x84A0

#define WARN_OPEN_CHIMNEY_OL_OFF    0x8500
#define WARN_OPEN_CHIMNEY_OL_ON		0x8501
#define WARN_OPEN_CHIMNEY_VS_UV 	0x8502
#define WARN_OPEN_CHIMNEY_VDD_OV	0x8503
#define WARN_OPEN_CHIMNEY_ILIM		0x8504
#define WARN_OPEN_CHIMNEY_TWARN		0x8505
#define WARN_OPEN_CHIMNEY_TSD		0x8506
#define WARN_OPEN_CHIMNEY_OC_LS1	0x8507
#define WARN_OPEN_CHIMNEY_OC_LS2	0x8508
#define WARN_OPEN_CHIMNEY_OC_HS1	0x8509
#define WARN_OPEN_CHIMNEY_OC_HS2	0x850A
#define WARN_OPEN_CHIMNEY_SGND_OFF	0x850B
#define WARN_OPEN_CHIMNEY_SBAT_OFF	0x850C

#define WARN_OPEN_COMPACT_OL_OFF    0x850D
#define WARN_OPEN_COMPACT_OL_ON		0x850E
#define WARN_OPEN_COMPACT_VS_UV 	0x850F
#define WARN_OPEN_COMPACT_VDD_OV	0x8510
#define WARN_OPEN_COMPACT_ILIM		0x8511
#define WARN_OPEN_COMPACT_TWARN		0x8512
#define WARN_OPEN_COMPACT_TSD		0x8513
#define WARN_OPEN_COMPACT_OC_LS1	0x8514
#define WARN_OPEN_COMPACT_OC_LS2	0x8515
#define WARN_OPEN_COMPACT_OC_HS1	0x8516
#define WARN_OPEN_COMPACT_OC_HS2	0x8517
#define WARN_OPEN_COMPACT_SGND_OFF	0x8518
#define WARN_OPEN_COMPACT_SBAT_OFF	0x8519

#define WARN_OPEN_BIN_OL_OFF    0x851A
#define WARN_OPEN_BIN_OL_ON		0x851B
#define WARN_OPEN_BIN_VS_UV 	0x851C
#define WARN_OPEN_BIN_VDD_OV	0x851D
#define WARN_OPEN_BIN_ILIM		0x851E
#define WARN_OPEN_BIN_TWARN		0x851F
#define WARN_OPEN_BIN_TSD		0x8520
#define WARN_OPEN_BIN_OC_LS1	0x8521
#define WARN_OPEN_BIN_OC_LS2	0x8522
#define WARN_OPEN_BIN_OC_HS1	0x8523
#define WARN_OPEN_BIN_OC_HS2	0x8524
#define WARN_OPEN_BIN_SGND_OFF	0x8525
#define WARN_OPEN_BIN_SBAT_OFF	0x8526

#endif /* ERR_WARN_CODES_H_ */
