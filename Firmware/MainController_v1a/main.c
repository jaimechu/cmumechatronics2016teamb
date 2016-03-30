/*
 * main.c
 * Created 3/29/16 by Nishant Pol and Jaime Chu
 * Mechatronics 18-578
 * MSP430F5521
 * Main Controller Code
 *
 * Code Composer v6
 * Version History
 * 3/17/16: Initial version, ported from bringup code
 * 3/29/16: Modified from RoboCapstone Mobility Controller Code
 */
#include <msp430.h>
#include "utils.h"
#include "clock_f5.h"
#include "dbg_uart_uscia0.h"


/** Debug task macros and globals **/
void debug_task(void);
void reset_isr(void);

#define DEBUG_CMD_BUF_SIZE 32
#define DEBUG_RESPONSE_BUF_SIZE 250
uint8_t debug_cmd_buf[DEBUG_CMD_BUF_SIZE];
uint8_t debug_cmd_buf_ptr = 0;

//Persistent command codes
#define PCMD_NONE	0x00
#define PCMD_RC_M1 0x01
#define PCMD_RC_M2 0x02
/** END Debug task macros and globals **/

/** Warning/Error code buffers and flags **/
volatile uint16_t warn_log[WARN_LOG_SIZE] = {0};
volatile uint8_t warn_log_ptr = 0;
volatile uint8_t warn_flag = 0;
volatile uint16_t err_log[ERR_LOG_SIZE] = {0};
volatile uint8_t err_log_ptr = 0;
volatile uint8_t err_flag = 0;
/** END Warning/Error code buffers and flags **/

/** Monitoring task globals **/
/* Struct to store monitoring data
 * Analog readings: Array contains current value | min value | max value | ready flag
 */
void monitor_task(void);
void monitor_setup(void);

//TODO: Update monitoring parameters
struct monitor_data_struct{
	uint16_t vsense_3V3[4];		//12-bit conversion of 3.3V rail voltage
	uint16_t vsense_5V0[4];		//12-bit conversion of 5V rail voltage
	uint16_t vsense_6VA[4];		//12-bit conversion of 6V analog rail voltage
	uint16_t vsense_12V[4];		//12-bit conversion of 12V rail voltage
	uint16_t mcu_temp[4];		//12-bit conversion of MSP430 temperature
	uint8_t estop_status;		//Set when ESTOP condition is active
};
struct monitor_data_struct monitor_data = {
		.vsense_3V3 = {0},
		.vsense_5V0 = {0},
		.vsense_6VA = {0},
		.vsense_12V = {0},
		.mcu_temp = {0},
		.estop_status = 0
};
//TODO: Update thresholds
#define VSENSE_3V3_MIN 0x0000
#define VSENSE_3V3_MAX 0xFFFF
#define VSENSE_5V0_MIN 0x0000
#define VSENSE_5V0_MAX	0xFFFF
#define VSENSE_6VA_MIN 0x0000
#define VSENSE_6VA_MAX 0xFFFF
#define VSENSE_12V_MIN	0x0000
#define VSENSE_12V_MAX	0xFFFF
#define MCU_TEMP_MIN	0x0000
#define MCU_TEMP_MAX	0xFFFF

//TODO: Change to ADC task
typedef enum  {MON_WAIT,
				START_ADC1,	//Drill-
				WAIT_ADC1,
				START_ADC2,	//Drill+
				WAIT_ADC2,
				START_ADC8,	//5Vsense
				WAIT_ADC8,
				START_ADC9,	//12Vsense
				WAIT_ADC9,
				START_ADC10,	//Temp
				WAIT_ADC10,
				START_ADC11,//3.3V divider
				WAIT_ADC11,
				SEND_MON_PCKT} mon_state_t;
volatile mon_state_t monCurrState = MON_WAIT;
#define MON_CNT_THRESH 100

/** END Monioring task globals **/

/** ADC task globals **/
void adc_setup(void);
void adc_task(void);
typedef enum  {INIT_SEQ1,
				WAIT_SEQ1,
				INIT_SEQ2,
				RUN_SEQ2,
				WAIT_SEQ2,
				UPDATE_OUT_BUF
				} adc_state_t;
volatile adc_state_t adc_current_state = INIT_SEQ1;
/* Output buffer and internal buffer data order
 * 0: A0 (5Vsense)
 * 1: A1 (12Vsense)
 * 2: A2 (6V Analog sense)
 * 3: A11 (3.3V sense)
 * 4: A10 (MCU temp)
 * 5: A3 (Hall Position Encoder 1)
 * 6: A4 (Hall Position Encoder 2)
 * 7: A5 (Hall Position Encoder 3)
 * 8: A6 (Hall Position Encoder 4)
 * 9: A7 (Hall Position Encoder 5)
 * 10: A12 (Hall Position Encoder 6)
 * 11-18: A8 (Optical bank, 8 channels)
 * 19-26: A13 (Hall Bank A, 8 channels)
 * 27-34: A14 (Hall Bank B, 8 channels)
 * 35-42: A15 (Hall Bank C, 8 channels)
 * Note: A9 is used as digital output
 */
#define ADC_BUF_LEN 43
uint16_t adc_internal_buffer[ADC_BUF_LEN] = {0};	//Data buffer for adc state only
uint16_t adc_output_buffer[ADC_BUF_LEN] = {0};	//Data buffer for other tasks to read
uint8_t adc_data_ready_flags[ADC_BUF_LEN] = {0};	//Flags set by adc task when new data is loaded, cleared by other tasks
uint8_t adc12_int_done_flag = 0;		//Indicates adc12 interrupt has finished
uint8_t adc_ext_mux_ptr = 0;			//Code for external muxes
uint8_t adc_seq2 = 0;					//Flag to indicate if adc sm is in seq1 or seq2

inline void set_hall_A_chnl(uint8_t mux_chnl);
inline void set_hall_B_chnl(uint8_t mux_chnl);
inline void set_hall_C_chnl(uint8_t mux_chnl);
inline void set_optical_chnl(uint8_t mux_chnl);

/** END ADC task globals **/

/** Main Loop **/

int main(void) {
	WDTCTL = WDTPW + WDTHOLD;   // Stop watchdog timer
	P1DIR |= BIT0;	//Debug LED1
	P2DIR |= BIT2;	//Debug LED2
	P7DIR |= BIT7;	//Debug LED3
	P5DIR |= BIT6;	//Debug WARN LED
	led_P1_0_off();
	led_P2_2_off();
	led_P7_7_on();
	led_P5_6_off();
	reset_isr();
	setup_clock();
	setup_dbg_uart();
	adc_setup();
	//monitor_setup();
    // Enable Interrupts
    __bis_SR_register(GIE);
    while(1)
    {
        debug_task();
        adc_task();
        //monitor_task();
    }
}

/** END Main Loop **/

/** Monitor Task Functions **/

/* Setup monitoring task */
/*
//TODO: Port to ADC task
void monitor_setup(void){
	ADC12CTL0 = ADC12SHT1_12 +	//1024 ADCLK cycles for sampling
				ADC12SHT0_12 +
				ADC12REF2_5V +	//2.5V reference
				ADC12REFON +	//Reference on
				ADC12ON;		//ADC on
	ADC12CTL1 = ADC12DIV_7 +	//clock divider: 8
				ADC12SSEL_2 +	//clock source: MCLK
				ADC12SHP;		//Use sampling timer
	return;
}

//TODO: Port to ADC task
void monitor_task(void){
	static uint8_t run_mon_cnt = 0;	//Set when Check routine must run
	static uint16_t drill_vzcr = 0;	//Negative reference for drill current
	uint8_t pckt_size = 0;				//Size of transmitted packet
	uint8_t buf[16];					//Buffer for transmitted/recieved packet
	uint16_t conversion;
	switch(monCurrState){
	case MON_WAIT:
		//State action
		run_mon_cnt++;
		//State transition
		if(run_mon_cnt > MON_CNT_THRESH){
			run_mon_cnt = 0;
			monCurrState = START_ADC1;	//T_MON0
		} else {
			monCurrState = MON_WAIT;	//T_MON1
		}
		break;
	case START_ADC1:
		//State action
		ADC12CTL0 &= ~ADC12ENC;			//Disable conversions to change channel
		ADC12MCTL0 = ADC12INCH_1;		//A1
		ADC12CTL0 |= ADC12SC+ADC12ENC;	//Start conversion
		//State transition
		monCurrState = WAIT_ADC1;		//T_MON2
		break;
	case WAIT_ADC1:
		//State action: nothing
		//State transition
		if(ADC12IFG & ADC12IFG0){
			monCurrState = START_ADC2;	//T_MON4
		} else {
			monCurrState = WAIT_ADC1;	//T_MON3
		}
		break;
	case START_ADC2:
		//State action
		drill_vzcr = ADC12MEM0;			//Get drill current reference
		ADC12CTL0 &= ~ADC12ENC;			//Disable conversions to change channel
		ADC12MCTL0 = ADC12INCH_2;		//A2
		ADC12CTL0 |= ADC12SC+ADC12ENC;	//Start conversion
		//State transition
		monCurrState = WAIT_ADC2;		//T_MON5
		break;
	case WAIT_ADC2:
		//State action: nothing
		//State transition
		if(ADC12IFG & ADC12IFG0){
			monCurrState = START_ADC8;	//T_MON7
		} else {
			monCurrState = WAIT_ADC2;	//T_MON6
		}
		break;
	case START_ADC8:
		//State action
		conversion = ADC12MEM0 - drill_vzcr;	//Get drill current
		monitor_data.drill_current[0] = conversion;
		if((conversion < monitor_data.drill_current[1]) || !monitor_data.drill_current[3]){
			monitor_data.drill_current[1] = conversion;
		}
		if((conversion > monitor_data.drill_current[2]) || !monitor_data.drill_current[3]){
			monitor_data.drill_current[2] = conversion;
		}
		monitor_data.drill_current[3] = 1;
		ADC12CTL0 &= ~ADC12ENC;			//Disable conversions to change channel
		ADC12MCTL0 = ADC12INCH_8;		//A8
		ADC12CTL0 |= ADC12SC+ADC12ENC;	//Start conversion
		//State transition
		monCurrState = WAIT_ADC8;		//T_MON8
		break;
	case WAIT_ADC8:
		//State action: nothing
		//State transition
		if(ADC12IFG & ADC12IFG0){
			monCurrState = START_ADC9;	//T_MON10
		} else {
			monCurrState = WAIT_ADC8;	//T_MON9
		}
		break;
	case START_ADC9:
		//State action
		conversion = ADC12MEM0;	//Get 5V sense voltage
		monitor_data.vsense_5V0[0] = conversion;
		if((conversion < monitor_data.vsense_5V0[1]) || !monitor_data.vsense_5V0[3]){
			monitor_data.vsense_5V0[1] = conversion;
		}
		if((conversion > monitor_data.vsense_5V0[2]) || !monitor_data.vsense_5V0[3]){
			monitor_data.vsense_5V0[2] = conversion;
		}
		monitor_data.vsense_5V0[3] = 1;
		ADC12CTL0 &= ~ADC12ENC;			//Disable conversions to change channel
		ADC12MCTL0 = ADC12INCH_9;		//A9
		ADC12CTL0 |= ADC12SC+ADC12ENC;	//Start conversion
		//State transition
		monCurrState = WAIT_ADC9;		//T_MON11
		break;
	case WAIT_ADC9:
		//State action: nothing
		//State transition
		if(ADC12IFG & ADC12IFG0){
			monCurrState = START_ADC10;	//T_MON13
		} else {
			monCurrState = WAIT_ADC9;	//T_MON12
		}
		break;
	case START_ADC10:
		//State action
		conversion = ADC12MEM0;	//Get 12V sense voltage
		monitor_data.vsense_12V[0] = conversion;
		if((conversion < monitor_data.vsense_12V[1]) || !monitor_data.vsense_12V[3]){
			monitor_data.vsense_12V[1] = conversion;
		}
		if((conversion > monitor_data.vsense_12V[2]) || !monitor_data.vsense_12V[3]){
			monitor_data.vsense_12V[2] = conversion;
		}
		monitor_data.vsense_12V[3] = 1;
		ADC12CTL0 &= ~ADC12ENC;			//Disable conversions to change channel
		while(REFCTL0 & REFGENBUSY);
		REFCTL0 = REFMSTR + REFVSEL_0 + REFON;
		ADC12CTL0 &= ~ADC12REF2_5V;		//1.5V reference
		ADC12MCTL0 = ADC12INCH_10 + ADC12SREF_1;		//A10, REF+
		ADC12CTL0 |= ADC12SC+ADC12ENC;	//Start conversion
		//State transition
		monCurrState = WAIT_ADC10;		//T_MON14
		break;
	case WAIT_ADC10:
		//State action: nothing
		//State transition
		if(ADC12IFG & ADC12IFG0){
			monCurrState = START_ADC11;	//T_MON16
		} else {
			monCurrState = WAIT_ADC10;	//T_MON15
		}
		break;
	case START_ADC11:
		//State action
		conversion = ADC12MEM0;	//Get Temp voltage
		monitor_data.mcu_temp[0] = conversion;
		if((conversion < monitor_data.mcu_temp[1]) || !monitor_data.mcu_temp[3]){
			monitor_data.mcu_temp[1] = conversion;
		}
		if((conversion > monitor_data.mcu_temp[2]) || !monitor_data.mcu_temp[3]){
			monitor_data.mcu_temp[2] = conversion;
		}
		monitor_data.mcu_temp[3] = 1;
		ADC12CTL0 &= ~ADC12ENC;			//Disable conversions to change channel
		while(REFCTL0 & REFGENBUSY);
		REFCTL0 = REFMSTR + REFVSEL_2 + REFON;
		ADC12CTL0 |= ADC12REF2_5V;		//2.5V reference
		ADC12MCTL0 = ADC12INCH_11 + ADC12SREF_1;		//A11, VREF+
		ADC12CTL0 |= ADC12SC+ADC12ENC;	//Start conversion
		//State transition
		monCurrState = WAIT_ADC11;		//T_MON17
		break;
	case WAIT_ADC11:
		//State action: nothing
		//State transition
		if(ADC12IFG & ADC12IFG0){
			monCurrState = SEND_MON_PCKT;//T_MON19
		} else {
			monCurrState = WAIT_ADC11;	//T_MON18
		}
		break;
	case SEND_MON_PCKT:
		//State action
		conversion = ADC12MEM0;	//Get 3.3V sense voltage
		monitor_data.vsense_3V3[0] = conversion;
		if((conversion < monitor_data.vsense_3V3[1]) || !monitor_data.vsense_3V3[3]){
			monitor_data.vsense_3V3[1] = conversion;
		}
		if((conversion > monitor_data.vsense_3V3[2]) || !monitor_data.vsense_3V3[3]){
			monitor_data.vsense_3V3[2] = conversion;
		}
		monitor_data.vsense_3V3[3] = 1;
		//Check parameters.  All values valid since to reach this state, all conversions must be completed.
		if(monitor_data.vsense_3V3[0] < VSENSE_3V3_MIN) issue_warning(WARN_LOW_3V3);
		if(monitor_data.vsense_3V3[0] > VSENSE_3V3_MAX) issue_warning(WARN_HIGH_3V3);
		if(monitor_data.vsense_5V0[0] < VSENSE_5V0_MIN) issue_warning(WARN_LOW_5V0);
		if(monitor_data.vsense_5V0[0] > VSENSE_5V0_MAX) issue_warning(WARN_HIGH_5V0);
		if(monitor_data.vsense_12V[0] < VSENSE_12V_MIN) issue_warning(WARN_LOW_12V);
		if(monitor_data.vsense_12V[0] > VSENSE_12V_MAX) issue_warning(WARN_HIGH_12V);
		if(monitor_data.mcu_temp[0] < MCU_TEMP_MIN) issue_warning(WARN_LOW_MCU_TEMP);
		if(monitor_data.mcu_temp[0] > MCU_TEMP_MAX) issue_warning(WARN_HIGH_MCU_TEMP);
		if(monitor_data.drill_current[0] < DR_CURRENT_MIN) issue_warning(WARN_LOW_DRILL_CURRENT);
		if(monitor_data.drill_current[0] > DR_CURRENT_MAX) issue_warning(WARN_HIGH_DRILL_CURRENT);
		if(monitor_data.estop_status) issue_warning(ESTOP_ACTIVATED);
		//These parameters are gathered by other tasks, so need to know if ran at least once.
		if(rc_check_ran_once){
			if(monitor_data.rc_mbatt[0] < RC_MBATT_MIN) issue_warning(WARN_LOW_RC_MBATT);
			if(monitor_data.rc_mbatt[0] > RC_MBATT_MAX) issue_warning(WARN_HIGH_RC_MBATT);
			if(monitor_data.rc_lbatt[0] < RC_LBATT_MIN) issue_warning(WARN_LOW_RC_LBATT);
			if(monitor_data.rc_lbatt[0] > RC_LBATT_MAX) issue_warning(WARN_HIGH_RC_LBATT);
			if(monitor_data.m1_current[0] < M_CURRENT_MIN) issue_warning(WARN_LOW_M1_CURRENT);
			if(monitor_data.m1_current[0] > M_CURRENT_MAX) issue_warning(WARN_HIGH_M1_CURRENT);
			if(monitor_data.m2_current[0] < M_CURRENT_MIN) issue_warning(WARN_LOW_M2_CURRENT);
			if(monitor_data.m2_current[0] > M_CURRENT_MAX) issue_warning(WARN_HIGH_M2_CURRENT);
			if(monitor_data.rc_temp[0] < RC_TEMP_MIN) issue_warning(WARN_LOW_RC_TEMP);
			if(monitor_data.rc_temp[0] > RC_TEMP_MAX) issue_warning(WARN_HIGH_RC_TEMP);
			if(monitor_data.rc_status & RC_STAT_M1_OVERCURRENT) issue_warning(WARN_RC_M1_OVERCURRENT);
			if(monitor_data.rc_status & RC_STAT_M2_OVERCURRENT) issue_warning(WARN_RC_M2_OVERCURRENT);
			if(monitor_data.rc_status & RC_STAT_ESTOP) issue_warning(WARN_RC_ESTOP);
			if(monitor_data.rc_status & RC_STAT_TEMP_ERR) issue_warning(WARN_RC_TEMP_ERR);
			if(monitor_data.rc_status & RC_STAT_TEMP2_ERR) issue_warning(WARN_RC_TEMP2_ERR);
			if(monitor_data.rc_status & RC_STAT_MBATT_H_ERR) issue_warning(WARN_RC_MBATT_H_ERR);
			if(monitor_data.rc_status & RC_STAT_LBATT_H_ERR) issue_warning(WARN_RC_LBATT_H_ERR);
			if(monitor_data.rc_status & RC_STAT_LBATT_L_ERR) issue_warning(WARN_RC_LBATT_L_ERR);
			if(monitor_data.rc_status & RC_STAT_M1_FAULT) issue_warning(WARN_RC_M1_FAULT);
			if(monitor_data.rc_status & RC_STAT_M2_FAULT) issue_warning(WARN_RC_M2_FAULT);
			if(monitor_data.rc_status & RC_STAT_MBATT_H_WARN) issue_warning(WARN_RC_MBATT_H_WARN);
			if(monitor_data.rc_status & RC_STAT_MBATT_L_WARN) issue_warning(WARN_RC_MBATT_L_WARN);
			if(monitor_data.rc_status & RC_STAT_TEMP_WARN) issue_warning(WARN_RC_TEMP_WARN);
			if(monitor_data.rc_status & RC_STAT_TEMP2_WARN) issue_warning(WARN_RC_TEMP2_WARN);
		}
		//State transition
		monCurrState = MON_WAIT;		//T_MON20
		break;
	default:
		issue_warning(WARN_ILLEGAL_MON_SM_STATE);
		monCurrState = MON_WAIT;
		break;
	}
}
*/
/** END Monitor Task Functions **/

/** ADC Task Functions **/
void adc_setup(void){
	//Setup mux pins
	//HALL_A_SEL1: P5.1
	//HALL_A_SEL2: P5.4
	//HALL_A_SEL3: P5.5
	//HALL_B_SEL1: P8.0
	//HALL_B_SEL2: P8.1
	//HALL_B_SEL3: P8.2
	//HALL_C_SEL1: P2.1
	//HALL_C_SEL2: P2.3
	//HALL_C_SEL3: P2.4
	//OPT_SEL1: P2.5
	//OPT_SEL2: P2.6
	//OPT_SEL3: P2.7
	P2DIR |= BIT1+BIT3+BIT4+BIT5+BIT6+BIT7;
	P5DIR |= BIT1+BIT4+BIT5;
	P8DIR |= BIT0+BIT1+BIT2;
	//Setup ADC pins to use ADC
	P6SEL = 0xFF;
	P5SEL |= BIT0;
	P7SEL |= BIT0 + BIT1 + BIT2 + BIT3;
	//Setup ADC12
	ADC12CTL0 = ADC12SHT1_0 +	//1024 ADCLK cycles for sampling
				ADC12SHT0_0 +
				ADC12REF2_5V +	//2.5V reference
				ADC12REFON +	//Reference on
				ADC12MSC + 		//Trigger sequential conversions automatically
				ADC12ON;		//ADC on
	ADC12CTL1 = ADC12DIV_7 +	//clock divider: 8
				ADC12SSEL_2 +	//clock source: MCLK
				ADC12SHP +		//Use sampling timer
				ADC12CONSEQ_1;	//Channel sequence, no repeat
	REFCTL0 = REFMSTR + REFVSEL_2 + REFON;	//2.5V reference
	return;
}

void adc_task(void){
	uint8_t i;
	switch(adc_current_state){
	case INIT_SEQ1:									//S8.1
		//State action
		adc_seq2 = 0;
		ADC12CTL0 &= ~ADC12ENC;						//Disable conversions to change channel
		ADC12MCTL0 = ADC12SREF_0 + ADC12INCH_0;		//A0 5Vsense, start of sequence
		ADC12MCTL1 = ADC12SREF_0 + ADC12INCH_1;		//A1 12Vsense
		ADC12MCTL2 = ADC12SREF_0 + ADC12INCH_2;		//A2 6VAnalog sense
		ADC12MCTL3 = ADC12SREF_0 + ADC12INCH_3;		//A3 Hall pos 1
		ADC12MCTL4 = ADC12SREF_0 + ADC12INCH_4;		//A4 Hall pos 2
		ADC12MCTL5 = ADC12SREF_0 + ADC12INCH_5;		//A5 Hall pos 3
		ADC12MCTL6 = ADC12SREF_0 + ADC12INCH_6;		//A6 Hall pos 4
		ADC12MCTL7 = ADC12SREF_0 + ADC12INCH_7;		//A7 Hall pos 5
		ADC12MCTL8 = ADC12SREF_0 + ADC12INCH_10;	//A10 Temp
		ADC12MCTL9 = ADC12SREF_0 + ADC12INCH_11;	//A11 3.3V sense
		ADC12MCTL10 = ADC12EOS + ADC12SREF_0 + ADC12INCH_12;	//A12 Hall pos 6, end of sequence
		ADC12IE = ADC12IE10;						//Enable ADC12 interrupt
		ADC12CTL0 |= ADC12SC+ADC12ENC;				//Start conversion
		//State transition
		adc_current_state = WAIT_SEQ1;				//T8.1
		break;
	case WAIT_SEQ1:									//S8.2
		//State action: no action
		//State transition
		if(adc12_int_done_flag){
			adc_current_state = INIT_SEQ2;			//T8.3
		} else {
			adc_current_state = WAIT_SEQ1;			//T8.2
		}
		break;
	case INIT_SEQ2:		//S8.4
		//State action
		adc_seq2 = 1;
		adc12_int_done_flag = 0;
		adc_ext_mux_ptr = 0;
		set_hall_A_chnl(adc_ext_mux_ptr);	//Reset external muxes
		set_hall_B_chnl(adc_ext_mux_ptr);
		set_hall_C_chnl(adc_ext_mux_ptr);
		set_optical_chnl(adc_ext_mux_ptr);
		ADC12CTL0 &= ~ADC12ENC;						//Disable conversions to change channel
		ADC12MCTL0 = ADC12SREF_0 + ADC12INCH_8;		//A8 optical, start of sequence
		ADC12MCTL1 = ADC12SREF_0 + ADC12INCH_13;		//A13 Hall Bank A
		ADC12MCTL2 = ADC12SREF_0 + ADC12INCH_14;		//A14 Hall Bank B
		ADC12MCTL3 = ADC12EOS+ ADC12SREF_0 + ADC12INCH_15;		//A15 Hall Bank C, end of sequence
		ADC12MCTL10 &= ~ADC12EOS;
		ADC12IE = ADC12IE3;						//Enable ADC12 interrupt
		ADC12CTL0 |= ADC12SC+ADC12ENC;				//Start conversion
		adc12_int_done_flag = 0;
		//State transition
		adc_current_state = WAIT_SEQ2;		//T8.5
		break;
	case WAIT_SEQ2:
		//State action: no action
		//State transition
		if(adc12_int_done_flag){
			adc_current_state = UPDATE_OUT_BUF;	//T8.7
		} else {
			adc_current_state = WAIT_SEQ2;		//T8.6
		}
		break;
	case UPDATE_OUT_BUF:
		//State action
		//copy input buffer to output buffer
		for(i = 0; i < ADC_BUF_LEN; i++){
			adc_output_buffer[i] = adc_internal_buffer[i];
			adc_data_ready_flags[i] = 1;
		}
		//State transition
		adc_current_state = INIT_SEQ1;		//T8.8
		break;
	default:
		issue_warning(WARN_ILLEGAL_ADC_SM_STATE);
		break;
	}
}

inline void set_hall_A_chnl(uint8_t mux_chnl){
	//HALL_A_SEL1: P5.1
	//HALL_A_SEL2: P5.4
	//HALL_A_SEL3: P5.5
	P5OUT = (P5OUT&(~BIT1))|((mux_chnl&BIT0)<<1);
	P5OUT = (P5OUT&(~BIT4))|((mux_chnl&BIT1)<<3);
	P5OUT = (P5OUT&(~BIT5))|((mux_chnl&BIT2)<<3);
	return;
}

inline void set_hall_B_chnl(uint8_t mux_chnl){
	//HALL_B_SEL1: P8.0
	//HALL_B_SEL2: P8.1
	//HALL_B_SEL3: P8.2
	P8OUT = mux_chnl&0x7;
	return;
}

inline void set_hall_C_chnl(uint8_t mux_chnl){
	//HALL_C_SEL1: P2.1
	//HALL_C_SEL2: P2.3
	//HALL_C_SEL3: P2.4
	P2OUT = (P2OUT&(~BIT1))|((mux_chnl&BIT0)<<1);
	P2OUT = (P2OUT&(~BIT3))|((mux_chnl&BIT1)<<2);
	P2OUT = (P2OUT&(~BIT4))|((mux_chnl&BIT2)<<2);
	return;
}

inline void set_optical_chnl(uint8_t mux_chnl){
	//OPT_SEL1: P2.5
	//OPT_SEL2: P2.6
	//OPT_SEL3: P2.7
	P2OUT = (P2OUT&0x1F)|((mux_chnl&0x7)<<5);
	return;
}


/** END ADC Task Functions **/

/** Debug Task functions **/

/* Process debug command functions */
void debug_task(void){
	static uint8_t persistent_cmd = 0;	//Code of command to continue running while other tasks run
	//static uint8_t pcmd_data0 = 0;		//Storage for persistent commands
	//static uint8_t pcmd_data1 = 0;
	uint8_t debug_cmd_ready = 0;
	uint8_t response_buf[DEBUG_RESPONSE_BUF_SIZE];
	uint8_t response_size = 0;
	//Check if serial data is ready to be formed into a command
	if(is_dbg_uart_rx_data_ready() && !persistent_cmd){
		uint8_t rx_byte = dbg_uart_get_byte();

		//Enter (CR) indicates command is complete
		if(rx_byte == 13){			//Enter pressed (CR)
			dbg_uart_send_byte(13);		//CR
			dbg_uart_send_byte(10);		//Line feed
			debug_cmd_ready = 1;	//Reset command buffer
		} else {//Fill command buffer
			dbg_uart_send_byte(rx_byte);	//Echo back character
			debug_cmd_buf[debug_cmd_buf_ptr] = rx_byte;
			if(debug_cmd_buf_ptr < DEBUG_CMD_BUF_SIZE){
				debug_cmd_buf_ptr++;
			} else {
				issue_warning(WARN_DBG_BUFF_OVERRUN);
			}
		}
	}
	//If command needs to continue running as other tasks run, do actions here
	if(persistent_cmd){
		switch(persistent_cmd){
		default:
			persistent_cmd = 0;
			break;
		}
		//When done with command, return terminal to user
		if(!persistent_cmd){
			dbg_uart_send_byte(13);		//CR
			dbg_uart_send_byte(10);		//Line feed
			dbg_uart_send_byte('>');	//Terminal prompt
			debug_cmd_ready = 0;
			debug_cmd_buf_ptr = 0;
		}
	}
	//Process command
	if(!persistent_cmd && debug_cmd_ready){
		if(debug_cmd_buf_ptr == 0){
			//No command, do nothing
		} else if((strncmp(debug_cmd_buf,"led1 on",6)==0) && (debug_cmd_buf_ptr == 7)){
			//>led1 on
			led_P1_0_on();
		} else if((strncmp(debug_cmd_buf,"led1 off",7)==0) && (debug_cmd_buf_ptr == 8)){
			//>led1 off
			led_P1_0_off();
		} else if((strncmp(debug_cmd_buf,"led2 on",6)==0) && (debug_cmd_buf_ptr == 7)){
			//>led2 on
			led_P2_2_on();
		} else if((strncmp(debug_cmd_buf,"led2 off",7)==0) && (debug_cmd_buf_ptr == 8)){
			//>led2 off
			led_P2_2_off();
		} else if((strncmp(debug_cmd_buf,"led3 on",6)==0) && (debug_cmd_buf_ptr == 7)){
			//>led3 on
			led_P7_7_on();
		} else if((strncmp(debug_cmd_buf,"led3 off",7)==0) && (debug_cmd_buf_ptr == 8)){
			//>led3 off
			led_P7_7_off();
		} else if((strncmp(debug_cmd_buf,"led4 on",6)==0) && (debug_cmd_buf_ptr == 7)){
			//>led4 on
			led_P5_6_on();
		} else if((strncmp(debug_cmd_buf,"led4 off",7)==0) && (debug_cmd_buf_ptr == 8)){
			//>led4 off
			led_P5_6_off();
		} else if((strncmp(debug_cmd_buf,"P1 get",6)==0) && (debug_cmd_buf_ptr == 6)){
			//>P1 get
			response_size = P1_get(response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"P2 get",6)==0) && (debug_cmd_buf_ptr == 6)){
			//>P2 get
			response_size = P2_get(response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"P3 get",6)==0) && (debug_cmd_buf_ptr == 6)){
			//>P3 get
			response_size = P3_get(response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"P4 get",6)==0) && (debug_cmd_buf_ptr == 6)){
			//>P4 get
			response_size = P4_get(response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"P5 get",6)==0) && (debug_cmd_buf_ptr == 6)){
			//>P5 get
			response_size = P5_get(response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"P6 get",6)==0) && (debug_cmd_buf_ptr == 6)){
			//>P6 get
			response_size = P6_get(response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"P7 get",6)==0) && (debug_cmd_buf_ptr == 6)){
			//>P7 get
			response_size = P7_get(response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"P8 get",6)==0) && (debug_cmd_buf_ptr == 6)){
			//>P8 get
			response_size = P8_get(response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"testwarn",8)==0) && (debug_cmd_buf_ptr == 8)){
			//>testwarn
			issue_warning(WARN_TEST);
		} else if((strncmp(debug_cmd_buf,"testerr",7)== 0) && (debug_cmd_buf_ptr == 7)){
			//>testerr
			issue_error(ERROR_TEST);
		} else if((strncmp(debug_cmd_buf,"warndump",8)==0) && (debug_cmd_buf_ptr == 8)){
			//>warndump
			response_size = warning_dump(response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"errdump",7)==0) && (debug_cmd_buf_ptr == 7)){
			//>errdump
			response_size = error_dump(response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"warnclear",9)==0) && (debug_cmd_buf_ptr == 9)){
			//>warnclear
			clear_warnings();
		} else if((strncmp(debug_cmd_buf,"errclear",8)==0) && (debug_cmd_buf_ptr == 8)){
			//>errclear
			clear_errors();
		} else if((strncmp(debug_cmd_buf,"mon estop",9)==0) && (debug_cmd_buf_ptr == 9)){
			//>mon estop
			if(monitor_data.estop_status){
				response_buf[0] = 'o';
				response_buf[1] = 'n';
				response_size = 2;
			} else {
				response_buf[0] = 'o';
				response_buf[1] = 'f';
				response_buf[2] = 'f';
				response_size = 3;
			}
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"mon mcu temp",12)==0) && (debug_cmd_buf_ptr == 12)){
			//>mon mcu temp
			response_size = print_mon_analog_value(monitor_data.mcu_temp, response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"mon 12V",7)==0) && (debug_cmd_buf_ptr == 7)){
			//>mon 12V
			response_size = print_mon_analog_value(monitor_data.vsense_12V, response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"mon 3V3",7)==0) && (debug_cmd_buf_ptr == 7)){
			//>mon 3V3
			response_size = print_mon_analog_value(monitor_data.vsense_3V3, response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"mon 5V0",7)==0) && (debug_cmd_buf_ptr == 7)){
			//>mon 5V0
			response_size = print_mon_analog_value(monitor_data.vsense_5V0, response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"mon 6VA",7)==0) && (debug_cmd_buf_ptr == 7)){
			//>mon 6VA
			response_size = print_mon_analog_value(monitor_data.vsense_6VA, response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else {
			dbg_uart_send_string("Invalid Command",15);
		}
		debug_cmd_ready = 0;
		if(!persistent_cmd){
			dbg_uart_send_byte(13);		//CR
			dbg_uart_send_byte(10);		//Line feed
			dbg_uart_send_byte('>');	//Terminal prompt
			debug_cmd_buf_ptr = 0;
		}
	}
}

/** END Debug Task functions **/

/** Interrupts **/

/* ADC12 Interrupt Handler
 * Grab conversion from ADC, put into internal buffer
 * If using external mux (optical and hall), change pins to mux input
 */
#pragma vector=ADC12_VECTOR
__interrupt void ADC12_ISR(void){
	/* Output buffer and internal buffer data order
	 * 0: A0 (5Vsense)
	 * 1: A1 (12Vsense)
	 * 2: A2 (6V Analog sense)
	 * 3: A11 (3.3V sense)
	 * 4: A10 (MCU temp)
	 * 5: A3 (Hall Position Encoder 1)
	 * 6: A4 (Hall Position Encoder 2)
	 * 7: A5 (Hall Position Encoder 3)
	 * 8: A6 (Hall Position Encoder 4)
	 * 9: A7 (Hall Position Encoder 5)
	 * 10: A12 (Hall Position Encoder 6)
	 * 11-18: A8 (Optical bank, 8 channels)
	 * 19-26: A13 (Hall Bank A, 8 channels)
	 * 27-34: A14 (Hall Bank B, 8 channels)
	 * 35-42: A15 (Hall Bank C, 8 channels)
	 * Note: A9 is used as digital output
	 */
	if(adc_seq2){
		adc_internal_buffer[11+adc_ext_mux_ptr] = ADC12MEM[0];	//Gather conversions
		adc_internal_buffer[19+adc_ext_mux_ptr] = ADC12MEM[1];
		adc_internal_buffer[27+adc_ext_mux_ptr] = ADC12MEM[2];
		adc_internal_buffer[35+adc_ext_mux_ptr] = ADC12MEM[3];
		adc_ext_mux_ptr++;
		if(adc_ext_mux_ptr > 7){
			adc12_int_done_flag = 1;			//Done with conversions, return to sm
		} else {
			ADC12CTL0 &= ~ADC12SC;	//Clear SC bit
			ADC12CTL0 |= ADC12SC;	//Start conversion for next sequence
			set_hall_A_chnl(adc_ext_mux_ptr);
			set_hall_B_chnl(adc_ext_mux_ptr);
			set_hall_C_chnl(adc_ext_mux_ptr);
			set_optical_chnl(adc_ext_mux_ptr);
		}
	} else { //Store conversions into internal buffer
		adc_internal_buffer[0] = ADC12MEM[0];
		adc_internal_buffer[1] = ADC12MEM[1];
		adc_internal_buffer[2] = ADC12MEM[2];
		adc_internal_buffer[3] = ADC12MEM[3];
		adc_internal_buffer[4] = ADC12MEM[4];
		adc_internal_buffer[5] = ADC12MEM[5];
		adc_internal_buffer[6] = ADC12MEM[6];
		adc_internal_buffer[7] = ADC12MEM[7];
		adc_internal_buffer[8] = ADC12MEM[8];
		adc_internal_buffer[9] = ADC12MEM[9];
		adc_internal_buffer[10] = ADC12MEM[10];
		adc12_int_done_flag = 1;
	}

}

/* Debug UART USCIA0 Interrupt Handler
 * UART Rx: Recieves incoming bytes from debug channel, puts into Debug UART datastructure
 * UART Tx: Sends bytes from Debug UART datastructure to debug channel
 */
#pragma vector=USCI_A0_VECTOR
__interrupt void USCIA0_ISR(void){
	if((UCA0IE & UCRXIE) && (UCA0IFG & UCRXIFG)){	//UART Rxbuf full interrupt
		//Get byte and clear interrupt
		DBG_UART_data.rx_bytes[DBG_UART_data.rx_head] = UCA0RXBUF;
		DBG_UART_data.rx_head++;
		//Wraparound condition
		if(DBG_UART_data.rx_head >= DBG_UART_RX_BUF_SIZE){
			DBG_UART_data.rx_head = 0;
		}
		if(DBG_UART_data.rx_head == DBG_UART_data.rx_tail)
			issue_warning(WARN_DBG_RX_BUF_FULL);
	} else if((UCA0IE & UCTXIE) && (UCA0IFG & UCTXIFG)){	//UART Txbuf ready interrupt
		//Load data and clear interrupt
		UCA0TXBUF = DBG_UART_data.tx_bytes[DBG_UART_data.tx_tail];
		DBG_UART_data.tx_tail++;
		//Wraparound condition
		if(DBG_UART_data.tx_tail >= DBG_UART_TX_BUF_SIZE){
			DBG_UART_data.tx_tail = 0;
		}
		//Disable Tx interrupt if last byte in buffer has been transmitted
		if(DBG_UART_data.tx_tail == DBG_UART_data.tx_head){
			disable_dbg_uart_txint();
		}
	} else {
		issue_warning(WARN_USCIA0_INT_ILLEGAL_FLAG);
		while(1);
	}
}
/* System Unmaskable Interrupt Handler
 * NMIIFG:
 * OFIFG: Oscillator Fault
 */
#pragma vector=UNMI_VECTOR
__interrupt void unmi_isr(void){
	switch(__even_in_range(SYSUNIV, 0x08)){
		case 0x00: break;	//No interrupt pending
		case 0x02: // NMIIFG
			issue_warning(WARN_NMI);
			break;
		case 0x04: 			// OFIFG
			if(UCSCTL7 & XT2OFFG){		//XT2 Oscillator fault
				issue_error(ERR_XT2_FAULT);
			}
			if(UCSCTL7 & XT1LFOFFG){	//XT1 Oscillator failt, low frequency mode
				issue_error(ERR_XT1_FAULT);
			}
			if(UCSCTL7 & DCOFFG){		//DCO fault
				issue_error(ERR_DCO_FAULT);
			}
			break;
		case 0x06: // ACCVIFG
			issue_error(ERR_FLASH_VIOL);
			break;
		case 0x08: // BUSIFG
			// If needed, obtain the flash error location here.
			//ErrorLocation = MidGetErrAdr();
			switch(__even_in_range(SYSBERRIV, 0x08)){
				case 0x00: break; // no bus error
				case 0x02: break; // USB bus error
				case 0x04: break; // reserved
				case 0x06: // MID error
					//<place your MID error handler code here>
					break;
				case 0x08: break;
				default: break;
			}
			break;
		default: break;
	}
}

/* Reset Interrupt Handler
 */
void reset_isr(void){
	switch(SYSRSTIV){
		case SYSRSTIV_NONE:
			break;
		case SYSRSTIV_BOR:
			issue_warning(WARN_RST_BOR);
			break;
		case SYSRSTIV_RSTNMI:
			issue_warning(WARN_RST_RSTNMI);
			break;
		case SYSRSTIV_DOBOR:
			issue_warning(WARN_RST_DOBOR);
			break;
		case SYSRSTIV_LPM5WU:
			issue_warning(WARN_RST_LPM5WU);
			break;
		case SYSRSTIV_SECYV:
			//issue_warning(WARN_RST_SECYV); //Software reset
			break;
		case SYSRSTIV_SVSL:
			issue_warning(WARN_RST_SVSL);
			break;
		case SYSRSTIV_SVSH:
			issue_warning(WARN_RST_SVSH);
			break;
		case SYSRSTIV_SVML_OVP:
			issue_warning(WARN_RST_SVMLOVP);
			break;
		case SYSRSTIV_SVMH_OVP:
			issue_warning(WARN_RST_SVMHOVP);
			break;
		case SYSRSTIV_DOPOR:
			issue_warning(WARN_RST_DOPOR);
			break;
		case SYSRSTIV_WDTTO:
			issue_warning(WARN_RST_WDTTO);
			break;
		case SYSRSTIV_WDTKEY:
			issue_warning(WARN_RST_WDTKEY);
			break;
		case SYSRSTIV_KEYV:
			issue_warning(WARN_RST_KEYV);
			break;
		case SYSRSTIV_FLLUL:
			issue_warning(WARN_RSTFLLUL);
			break;
		case SYSRSTIV_PERF:
			issue_error(ERR_RST_PERF);
			break;
		case SYSRSTIV_PMMKEY:
			issue_warning(WARN_RST_PMM_KEY);
			break;
		default:
			break;
	}
}
/** END Interrupts **/