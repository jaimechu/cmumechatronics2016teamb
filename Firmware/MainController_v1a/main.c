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
#include "ldc_spi_uscib0.h"
#include "ldc1000.h"
#include "i2c_uscib1.h"
#include "motor.h"
#include "motor_spi_uscia1.h"

#define PLASTIC 1
#define GLASS 2
#define METAL 3
#define OTHER 4
#define TRANSPARENT 5
#define OPAQUE 6
#define HEAVY 7
#define LIGHT 8
#define NOT_METAL 9


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
 * 11-18: A8 (Optical bank, 8 channels) "OPT"
 * 19-26: A13 (Hall Bank A, 8 channels) "MASS"
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

/** END ADC task globals **/

/** LDC task globals **/
void ldc_task(void);
typedef enum {IDLE,
			  POLL_BRD0,
			  WAIT_BRD0,
			  POLL_BRD1,
			  WAIT_BRD1,
			  POLL_BRD2,
			  WAIT_BRD2,
			  COMPUTE,
			  HANG_ERR,
			  REPOLL_BRD0
} ldc_state_t;
volatile ldc_state_t ldc_current_state = IDLE;
#define LDC_WAIT_THRESH 10000//250
#define LDC_WAIT_THRESH_MIN 150

#define LDC_METAL_H 0x0630
#define LDC_METAL_NO_COMPT 0x1000
#define LDC_WOOD_H 0x100

#define NUM_LDC_SENSORS 2
//TODO: Change buffer size

uint16_t ldc_data_buf[NUM_LDC_SENSORS] = {0};
uint8_t ldc_data_buf_ptr = 0;

uint8_t ldc_run = 0;
uint8_t ldc_stop = 0;
uint16_t ldc_final_val = 0;

uint8_t ldc_sensor_data_check_inside();
uint8_t ldc_sensor_data_check_outside (uint8_t curr_buf);

uint16_t ldc_get_proximity(uint8_t brd);
void ldc_setup(uint8_t brd);
void ldc_write_reg(uint8_t reg_addr, uint8_t data, uint8_t brd);
inline uint8_t ldc_read_reg(uint8_t reg_addr, uint8_t brd);
inline void ldc_read_reg_init(uint8_t reg_addr, uint8_t brd);
inline uint8_t ldc_read_reg_get_data(void);
inline void ldc_read_reg_multiple(uint8_t reg_addr, uint8_t *buf, uint8_t num_regs, uint8_t brd);
inline void ldc_read_reg_multiple_init(uint8_t reg_addr, uint8_t *buf, uint8_t num_regs, uint8_t brd);
inline uint8_t ldc_read_reg_multiple_get_data(uint8_t *buf);


/** END LDC task globals **/

/** Mass task globals **/
void mass_task(void);
uint8_t mass_sensor_data_check_inside();
uint8_t mass_sensor_data_check_outside(uint8_t curr_buf);
typedef enum {MASS_IDLE,
	          MASS_WAIT,
			  MASS_GET,
			  MASS_COMPUTE
} mass_state_t;
volatile mass_state_t mass_current_state = MASS_IDLE;
uint8_t mass_run = 0;
uint8_t mass_end = 0;
#define NUM_MASS_SENSORS 4
#define MASS_BUF_SIZE 100
#define MASS_WAIT_CNTR 5000
//TODO: Change buffer size
uint16_t mass_data_buf[NUM_MASS_SENSORS][MASS_BUF_SIZE];
uint16_t mass_buf_ptr = 0;
uint16_t mass_hist_buf[32] = {0};
uint16_t hist_delta_bin = 0;
/** END Mass task globals **/

/** Optical task globals **/
void opt_task(void);
uint8_t opt_sensor_data_check_inside();
uint8_t opt_sensor_data_check_outside (uint8_t curr_buf);

typedef enum {OPT_IDLE,
	          OPT_WAIT,
			  OPT_GET,
			  OPT_COMPUTE
} opt_state_t;
volatile opt_state_t opt_current_state = OPT_IDLE;
uint8_t opt_run = 0;
uint8_t opt_end = 0;
#define NUM_OPT_SENSORS 8
#define OPT_BUF_SIZE 100
#define OPT_WAIT_CNTR 5000
#define OPT_CROSS_THRESH 1000

//#define OPTDUMP
#ifdef OPTDUMP
uint16_t opt_data_buf[NUM_OPT_SENSORS][OPT_BUF_SIZE];
#endif
uint16_t opt_data_time_buf[NUM_OPT_SENSORS] = {0};
uint16_t opt_data_low_buf[NUM_OPT_SENSORS] = {0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF};
uint16_t opt_buf_ptr = 0;
uint16_t opt_low_val = 0;
//Comment this out for just dumps of values
#define OPTVAL
#ifdef OPTVAL
uint16_t opt_max_time = 0;
#endif



/** END Optical task globals **/

/** Motor Enable task globals **/
//TODO: Fix the chimney and bin cmd
#define CHIMNEY_CMD 0x0004
#define COMPACT_CMD 0x0C04
#define BIN_CMD 0x0000

// TODO: Uncomment when UI is ready
#define NO_UI

//TODO: Uncomment for actual run
#define RAW_DATA_PRINT

uint8_t chimney_motor_enable = 0;
uint8_t bin_motor_enable = 0;
uint8_t compact_motor_enable = 0;
uint8_t sort_motor_enable = 0;

void motor_enable_setup(void);
void i2c_task(void);
void motor_spi_task(void);
void check_chimney_resp(uint16_t resp);
void check_compact_resp(uint16_t resp);
void check_bin_resp(uint16_t resp);
uint8_t is_inside_interval(uint16_t test_val, uint16_t low_thresh, uint16_t high_thresh); // Helper function



typedef enum  {I2C_SEND_MOTOR,
			   I2C_WAIT1,
			   I2C_HOME_CURSOR,
			   I2C_WAIT2,
			   I2C_SEND_UI,
			   I2C_WAIT3,
			   I2C_CLOSE} i2c_state_t;
volatile i2c_state_t i2cCurrState = I2C_SEND_MOTOR;

typedef enum  {MOTOR_INIT,
			   MOTOR1_READ,
			   MOTOR2_READ,
			   MOTOR3_READ,
			   MOTOR_WAIT1,
			   MOTOR_WAIT2,
			   MOTOR_WAIT3,
			   MOTOR_WAIT4} motor_state_t;
volatile motor_state_t motorCurrState = MOTOR_INIT;
/** END Motor Enable task globals **/

/* Buffer globals */
struct sensor_data_struct{
	//TODO: Bin task should reset this struct
	uint8_t obj_en; //Chimney sets to indicate struct has been assigned to obj - bin reads
	uint8_t mass_result; // Set by mass_task() - read by mass-task()
	uint8_t opt_result; // Set by opt_task() - read by mass-task()
	uint8_t LDC_result; // Set by LDC_result - read by mass_task()
	uint16_t mass_start;  // Set by chimney_task
	uint16_t mass_end;
	uint16_t opt_start;
	uint16_t opt_end;
	uint16_t LDC_start;
	uint16_t LDC_end;
	uint8_t final_result;
	uint8_t compact_ok;
};

extern volatile struct sensor_data_struct sensor_data1 = {
		.obj_en = 0,
		.mass_result = 0,
		.opt_result = 0, // Set by opt_task() - read by mass-task()
		.LDC_result = 0, // Set by LDC_result - read by mass_task()
		.mass_start = 0,  // Set by chimney_task
		.mass_end = 0,
		.opt_start = 0,
		.opt_end = 0,
		.LDC_start = 0,
		.LDC_end = 0,
		.final_result = 0,
		.compact_ok	= 0
};
extern volatile struct sensor_data_struct sensor_data2 = {
		.obj_en = 0,
		.mass_result = 0,
		.opt_result = 0, // Set by opt_task() - read by mass-task()
		.LDC_result = 0, // Set by LDC_result - read by mass_task()
		.mass_start = 0,  // Set by chimney_task
		.mass_end = 0,
		.opt_start = 0,
		.opt_end = 0,
		.LDC_start = 0,
		.LDC_end = 0,
		.final_result = 0,
		.compact_ok = 0
};
extern volatile struct sensor_data_struct sensor_data3 = {
		.obj_en = 0,
		.mass_result = 0,
		.opt_result = 0, // Set by opt_task() - read by mass-task()
		.LDC_result = 0, // Set by LDC_result - read by mass_task()
		.mass_start = 0,  // Set by chimney_task
		.mass_end = 0,
		.opt_start = 0,
		.opt_end = 0,
		.LDC_start = 0,
		.LDC_end = 0,
		.final_result = 0,
		.compact_ok = 0
};

typedef enum  {SYS_OFF,
			   SYS_ON,
			   SYS_MAINT} sys_state_t;
volatile sys_state_t sysState = SYS_OFF;

void sys_task();

struct total_count_struct{
	uint8_t plastic;
	uint8_t glass;
	uint8_t metal;
	uint8_t other;
	uint8_t plastic_full;
	uint8_t glass_full;
	uint8_t metal_full;
	uint8_t other_full;
};
volatile struct total_count_struct total_count = {
		.plastic = 0,
		.glass = 0,
		.metal = 0,
		.other = 0,
		.plastic_full =0,
		.glass_full =0,
		.metal_full = 0,
		.other_full = 0
};
const uint8_t ui_buf[80] = "PL:000  GL:000  xxxxxxxXxxxxxxxXxxxxxxxXMT:000  OT:000  xxxxxxxXxxxxxxxXxxxxxxxX";
#define UI_PLASTIC_INDEX 3
#define UI_GLASS_INDEX 11
#define UI_METAL_INDEX 43
#define UI_OTHER_INDEX 51
#define I2C_ADDR_UI_IO 0x28
/* END Buffer globals */

/* Chimney Functional task globals */
#define DEG0_POS_L 0x4A00
#define DEG0_POS_H 0x4B00
#define DEG90_POS_L 0x0A00
#define DEG90_POS_H 0x0B00
#define DEG180_POS_L 0xCA00
#define DEG180_POS_H 0xCB00
#define DEG270_POS_L 0x8900
#define DEG270_POS_H 0x8A00
#define FULL_ROT_LOW_THRESH 0x0400
#define FULL_ROT_HIGH_THRESH 0x0600
#define CHIM_SPEED 0x439

#define MASS_OFFSET_START  0x7360// Add this to the door value to get the start of mass
#define MASS_OFFSET_END 0xB360
#define OPT_OFFSET_START 0x2F50
#define OPT_OFFSET_END 0x6F50
#define LDC_OFFSET_START 0x100// 0x1D00
#define LDC_OFFSET_END 0x500 // 0x38B0

uint16_t last_insert_pos = 0;

void chimney_task(void);
uint16_t get_curr_chimney_pos(void);
uint8_t check_chim_paddle_pos(void);
uint8_t check_chim_next_pos(void);
uint8_t check_chim_full_rot(void);
uint8_t is_entry_door_open(void);

typedef enum  {CHIM_IDLE_OFF,
			   CHIM_CHECK_PADDLE,
			   CHIM_TURN_PADDLE,
			   CHIM_IDLE_ON,
			   CHIM_CHECK_POS,
			   CHIM_RUN2DOOR,
			   CHIM_STOP,
			   CHIM_RUNALL,
			   CHIM_INIT_BUF} chimney_state_t;
volatile chimney_state_t chimneyCurrState = CHIM_IDLE_OFF;
volatile uint8_t last_paddle = 0;
/* END Chimney Functional task globals */

/* Compact Functional task globals */
#define COMPACT_SPEED 0x9C3 // Max Speed
#define COMPACT_OPEN_H 0x300
#define COMPACT_OPEN_L 0x600
#define COMPACT_CLOSE_H 0x900
#define COMPACT_CLOSE_L 0xA00

#define COMPACT_ILLEGAL 0
#define COMPACT_OPEN  1
#define COMPACT_CLOSE 2
#define COMPACT_NOT_ILLEGAL 3

uint16_t get_curr_compact_pos(void);
uint8_t check_compact_pos(void);
void compact_setup(void);
uint8_t check_compact_timeout(void);
void reset_compact_timer(void);
void compact_task(void);

uint8_t compact_ok = 0;
uint8_t compact_done = 0;

typedef enum  {COMPACT_IDLE_OFF,
			   COMPACT_IDLE_ON,
			   COMPACT_CHECK_POS,
			   OPEN_COMPACT,
			   COMPACT_AT_OPEN,
			   COMPACT_WAIT,
			   CLOSE_COMPACT,
			   COMPACT_AT_CLOSE,
			   COMPACT_ERR} compact_state_t;
volatile compact_state_t compactCurrState = COMPACT_IDLE_OFF;

/* END Compact Functional task globals */

/* Bin Functional task globals */

//Bin position thresholds
#define GLASS_DUMP_L 0x4A00
#define GLASS_DUMP_H 0x4B00
#define PLASTIC_DUMP_L 0x0A00
#define PLASTIC_DUMP_H 0x0B00
#define METAL_DUMP_L 0xCA00
#define METAL_DUMP_H 0xCB00
#define OTHER_DUMP_L 0x8900
#define OTHER_DUMP_H 0x8A00

#define GLASS_DUMP_AVG (GLASS_DUMP_H + GLASS_DUMP_L)/2
#define PLASTIC_DUMP_AVG (PLASTIC_DUMP_H + PLASTIC_DUMP_L)/2
#define METAL_DUMP_AVG (METAL_DUMP_H + METAL_DUMP_L)/2
#define OTHER_DUMP_AVG (OTHER_DUMP_H + OTHER_DUMP_L)/2



#define GLASS_DOOR_L 0x4A00
#define GLASS_DOOR_H 0x4B00
#define PLASTIC_DOOR_L 0x0A00
#define PLASTIC_DOOR_H 0x0B00
#define METAL_DOOR_L 0xCA00
#define METAL_DOOR_H 0xCB00
#define OTHER_DOOR_L 0x8900
#define OTHER_DOOR_H 0x8A00

#define GLASS_THRESH 0x4A00
#define PLASTIC_THRESH 0x4B00
#define METAL_THRESH 0xCB00
#define OTHER_THRESH 0x8900

#define GLASS_HALF_THRESH_L 0x0000
#define GLASS_HALF_THRESH_H 0xFFF0
#define PLASTIC_HALF_THRESH_L 0x0000
#define PLASTIC_HALF_THRESH_H 0xFFF0
#define METAL_HALF_THRESH_L 0x0000
#define METAL_HALF_THRESH_H 0xFFF0
#define OTHER_HALF_THRESH_L 0x0000
#define OTHER_HALF_THRESH_H 0xFFF0

#define BIN_SPEED_SLOW 0x439
#define BIN_SPEED_FAST 0x900
//TODO: calibrate this val
#define BIN_SPEED_THRESH 0x500


typedef enum  {BIN_PLASTIC,
			   BIN_GLASS,
			   BIN_METAL,
			   BIN_OTHER } bin_t;
bin_t binCurr = BIN_PLASTIC;


typedef enum  {BIN_IDLE_OFF,
			   BIN_FIND_NEAREST,
			   BIN_CHECK_POS_MAINT,
			   BIN_RUN_TO_DOOR,
			   BIN_WAIT_OPEN,
			   BIN_WAIT_USER,
			   BIN_INC_INT,
			   BIN_IDLE_ON,
			   BIN_CHECK_POS,
			   BIN_RUN_TO_DUMP,
			   BIN_AT_POS} bin_state_t;
volatile bin_state_t binCurrState = BIN_IDLE_OFF;

typedef enum  {ACC_IDLE,
		       ACC_RUN1,
			   ACC_RUN2} acc_state_t;
volatile acc_state_t accCurrState = ACC_IDLE;

uint8_t bin_request = 0; //TODO: Issue request in mass_task
bin_t bin_int_request = BIN_GLASS;
uint8_t bin_ready = 0;
uint16_t start_bin_pos = 0;
uint16_t total_distance = 0;
uint8_t bin_run = 0;


void bin_acc_task(void);
void bin_task(void);
uint16_t get_curr_bin_pos(void);
uint8_t check_bin_pos(bin_t bin_requested);
bin_t check_bin_nearest_pos(void);
uint8_t check_bin_request_pos(bin_t bin_requested);
uint8_t is_bin_door_open(void);
uint16_t get_target_distance(uint16_t pos, bin_t bin_requested);
/* END Bin Functional task globals */

/* Stepper Functional task globals */
#define STEP_DIR_OPEN 1
#define STEP_DIR_CLOSE 0

#define STEP_CLOSE_POS 0x0000
#define STEP_TIME 0x1000

uint8_t step_switch = 0;
uint16_t step_position = 0;
uint8_t step_close = 0;
uint8_t step_request = 0;

typedef enum  {STEP_IDLE_OFF,
			   STEP_FIND_HOME,
			   STEP_WAIT_HOME,
			   STEP_STOP,
			   STEP_WAIT_REQUEST,
			   STEP_MOVE,
			   STEP_PAUSE,
	           STEP_RUN} step_state_t;
volatile step_state_t stepCurrState = STEP_IDLE_OFF;

void stepper_setup(void);
void step_task(void);
uint8_t is_step_switch_pressed(void);
void stepper_enable(uint8_t direction);
void stepper_disable(void);
/* END Stepper Functional task globals*/

/* LED Functional task globals */
void led_setup(void);
void led_task(void);
/* END LED task globals */

/* Bin Full Functional task globals */
//TODO: FIX THIS VAL
#define BIN_FULL_THRESH_H 0x500
#define BIN_FULL_THRESH_L 0x001

typedef enum  {BIN_FULL_IDLE,
		       BIN_FULL_START,
			   BIN_FULL_WAIT,
			   BIN_FULL_END } binFull_state_t;
volatile binFull_state_t binFullCurrState = BIN_FULL_IDLE;

uint8_t us_request = 0;
uint16_t distance = 0; // US distance
uint8_t us_timer_done = 0;

uint8_t check_us_timeroverflow(void);
void bin_full_setup(void);
void bin_full_task(void);
/*END Bin full task globals */


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
	i2c_uscib1_setup();
	motor_pwm_setup();
	MOTOR_SPI_setup(0,0); // Idle low, output on rising
	stepper_setup();
	compact_setup();
	bin_full_setup();
	//LDC_SPI_setup(0,1); //LDC
    // Enable Interrupts
    __bis_SR_register(GIE);
    //ldc_setup(0); 		//LDC
    //ldc_setup(1); 		//LDC
    motor_enable_setup();
    led_setup();

    /*
    uint16_t i;
    uint8_t buf[4] = {65,65,65,65};
    while(1){
    	init_I2C_transac(buf,1,0x28);
    	while(!is_I2C_rx_ready());
    	end_I2C_transac();
    	for(i=0; i<50000; i++);
    	buf[0]++;
    }
*/
    while(1)
    {
        sys_task();
    	debug_task();
        adc_task();
        //ldc_task();
        mass_task();
        opt_task();
        i2c_task();
        motor_spi_task();
        compact_task();
        step_task();
        chimney_task();
        //bin_task();
        bin_full_task();
        led_task();
    }


/*
    uint8_t resp = 0;
    volatile uint16_t prox_data = 0;
    volatile uint8_t buf[16] = {0};
    while(1){
    	resp = ldc_read_reg(0x00,1);
    	if(resp != 0x80){
    		while(1);
    	}
    	ldc_read_reg_multiple(0x00,buf,6,1);
    	ldc_read_reg_multiple(0x0A,buf,2,1);
    	ldc_read_reg_multiple(0x20,buf,6,1);
    	if(buf[1]&OSC_DEAD){
    		uint8_t i = 0;
    		for(i=0; i < 1; i++);
    	}
    	prox_data = ldc_get_proximity(1);

    }
*/
}

/** END Main Loop **/

/** System Task Function **/
void sys_task(void){
	if(P1IN & BIT1){
		sysState = SYS_ON;
	} else if (P1IN & BIT2){
		sysState = SYS_MAINT;
	} else {
		sysState = SYS_OFF;
	}
	return;
}
/** END System Task Function **/

/** ADC Task Functions **/
void adc_setup(void){
	//Setup mux pins
	//HALL_A_SEL1: P5.1
	//HALL_A_SEL2: P5.4
	//HALL_A_SEL3: P5.5
	//HALL_B_SEL1: P8.0
	//HALL_B_SEL2: P8.1
	//HALL_B_SEL3: P8.2
	P5DIR |= BIT1+BIT4+BIT5;
	P8DIR |= BIT0+BIT1+BIT2;
	//Setup ADC pins to use ADC
	//P5SEL |= BIT0; //Unused
	P6SEL |= BIT0+BIT1+BIT2+BIT3+BIT4+BIT5;
	P7SEL |= BIT1 + BIT2 + BIT3;	//P7.0 removed from sequence (reassigned to LDC_CS2)
	//Setup ADC12
	ADC12CTL0 = ADC12SHT1_6 +	//128? maybe(check this) ADCLK cycles for sampling (775Hz sequence rate)
				ADC12SHT0_6 +
				//ADC12REF2_5V +	//2.5V reference
				ADC12REFON +	//Reference on
				ADC12MSC + 		//Trigger sequential conversions automatically
				ADC12ON;		//ADC on
	ADC12CTL1 = ADC12DIV_7 +	//clock divider: 8
				ADC12SSEL_2 +	//clock source: MCLK
				ADC12SHP +		//Use sampling timer
				ADC12CONSEQ_1;	//Channel sequence, no repeat
	REFCTL0 = REFMSTR + REFVSEL_0 + REFON;	//1.5V reference
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
		ADC12MCTL6 = ADC12SREF_1 + ADC12INCH_10;	//A10 Temp
		//TODO: Change ADC12REF to 2.5 volts for 3.3V sense
		ADC12MCTL7 = ADC12EOS + ADC12SREF_0 + ADC12INCH_11;	//A11 3.3V sense, end of sequence
		ADC12IE = ADC12IE7;						//Enable ADC12 interrupt
		ADC12CTL0 |= ADC12SC+ADC12ENC;				//Start conversion
		//State transition
		adc_current_state = WAIT_SEQ1;				//T8.1
		break;
	case WAIT_SEQ1:									//S8.2
		//State action: no action
		//State transition
		if(adc12_int_done_flag){
			adc12_int_done_flag = 0;
			adc_current_state = INIT_SEQ2;			//T8.3
		} else {
			adc_current_state = WAIT_SEQ1;			//T8.2
		}
		break;
	case INIT_SEQ2:		//S8.4
		//State action
		adc_seq2 = 1;
		//adc12_int_done_flag = 0;
		adc_ext_mux_ptr = 0;
		set_hall_A_chnl(adc_ext_mux_ptr);	//Reset external muxes
		set_hall_B_chnl(adc_ext_mux_ptr);
		ADC12CTL0 &= ~ADC12ENC;						//Disable conversions to change channel
		ADC12MCTL0 = ADC12SREF_1 + ADC12INCH_13;		//A13 Hall Bank A , changed the SREF, end of sequence
		ADC12MCTL1 = ADC12EOS + ADC12SREF_0 + ADC12INCH_14;		//A14 Hall Bank B
		ADC12MCTL7 &= ~ADC12EOS;
		ADC12IE = ADC12IE1;						//Enable ADC12 interrupt
		ADC12CTL0 |= ADC12SC+ADC12ENC;				//Start conversion
		//adc12_int_done_flag = 0;
		//State transition
		adc_current_state = WAIT_SEQ2;		//T8.5
		break;
	case WAIT_SEQ2:
		//State action: no action
		//State transition
		if(adc12_int_done_flag){
			adc12_int_done_flag = 0;
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
		adc_current_state = INIT_SEQ1;
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
/** END ADC Task Functions **/

/** LDC Task Functions **/
uint8_t ldc_sensor_data_check_inside(){
	uint16_t currChimPos = get_curr_chimney_pos();
	if(sensor_data1.obj_en && is_inside_interval(currChimPos, sensor_data1.LDC_start, sensor_data1.LDC_end)){
		return 1;
	} else if(sensor_data2.obj_en && is_inside_interval(currChimPos, sensor_data2.LDC_start, sensor_data2.LDC_end)){
		return 2;
	} else if(sensor_data3.obj_en && is_inside_interval(currChimPos, sensor_data3.LDC_start, sensor_data3.LDC_end)){
		return 3;
	}
	return 0;
}

uint8_t ldc_sensor_data_check_outside(uint8_t curr_buf){
	uint16_t currChimPos = get_curr_chimney_pos();
	if(curr_buf == 1) {
		if (!is_inside_interval(currChimPos, sensor_data1.LDC_start, sensor_data1.LDC_end)){
			return 1;
		}
		return 0;
	} else if (curr_buf == 2){
		if(!is_inside_interval(currChimPos, sensor_data2.LDC_start, sensor_data2.LDC_end)){
			return 1;
		}
		return 0;
	} else if (curr_buf == 3) {
		if(!is_inside_interval(currChimPos, sensor_data3.LDC_start, sensor_data3.LDC_end)){
			return 1;
		}
		return 0;
	}
	issue_warning(WARN_LDC_TASK_STOP_ILLEGAL_BUF);
	return 0;
}

void ldc_task(void){
	static uint16_t wait_cntr = 0;
	volatile static uint16_t base_ldc_val = 0;
	volatile static uint16_t ldc_min_val = 0;
	volatile static uint16_t ldc_max_val = 0;
	volatile static uint16_t first_run = 1;
	uint16_t min_diff = 0;
	uint16_t max_diff = 0;
	uint8_t ldc_result = 0;
	uint8_t response_size = 0;
	uint8_t buf[8];
	static uint8_t curr_buf = 0;
	switch(ldc_current_state){
	case IDLE:										//STATE 4.1
		//State action:
		ldc_stop = 0;
		//State transition
		if(ldc_run == 1) { 							// For debug
			ldc_current_state = POLL_BRD0;
		} else if (curr_buf = ldc_sensor_data_check_inside()) {
			ldc_current_state = POLL_BRD0;
		} else {
			ldc_current_state = IDLE;
		}
		break;
	case POLL_BRD0:									//STATE 4.2
		//State action
		ldc_run = 0;
		ldc_read_reg_multiple_init(LDC_STATUS,buf,3,0);	//Send request for board 0 data
		wait_cntr = 0;
		//State transition
		ldc_current_state = WAIT_BRD0;				//T4.3
		break;
	case REPOLL_BRD0:								//STATE 4.9
		//State action
		ldc_read_reg_multiple_get_data(buf);		//Get Board 1 data
		if(buf[1]&DRDY){
			ldc_data_buf[1] = (buf[3]<<8)|buf[2];
		}
		if(buf[1]&OSC_DEAD){
			issue_warning(WARN_LDC2_OSC_DEAD);
		}
		ldc_read_reg_multiple_init(LDC_STATUS,buf,3,0);	//Send request for board 0 data
		wait_cntr = 0;
		//State transiton
		ldc_current_state = WAIT_BRD0;				//T4.17
		break;
	case WAIT_BRD0:									//STATE 4.3
		//State action
		wait_cntr++;

		//State transition
		//if(is_LDC_spi_rx_ready() && wait_cntr > LDC_WAIT_THRESH_MIN){					//T4.4
		if(is_LDC_spi_rx_ready()){					//T4.4
			ldc_current_state = POLL_BRD1;
		} else if(wait_cntr > LDC_WAIT_THRESH){		//T4.13
			ldc_current_state = HANG_ERR;
			issue_warning(WARN_LDC_SPI_HANG0);
		} else {									//T4.10
			ldc_current_state = WAIT_BRD0;
		}
		break;
	case POLL_BRD1:									//STATE 4.4
		//State action
		ldc_read_reg_multiple_get_data(buf);		//Get Board 0 data
		if(buf[1]&DRDY){
			ldc_data_buf[0] = (buf[3]<<8)|buf[2];
			// Find min and max values for both boards (0 AND 1)
			if(first_run) {
				base_ldc_val = ldc_data_buf[0];
				ldc_min_val = ldc_data_buf[0];
				ldc_max_val = ldc_data_buf[0];
				first_run = 0;
			} else {
				if(ldc_data_buf[0] > ldc_max_val){
					ldc_max_val = ldc_data_buf[0];
				} else if (ldc_data_buf[0] < ldc_min_val) {
					ldc_min_val = ldc_data_buf[0];
				}
			}
		}
		if(buf[1]&OSC_DEAD){
			issue_warning(WARN_LDC0_OSC_DEAD);
		}
		ldc_read_reg_multiple_init(LDC_STATUS,buf,3,1); //Send request for board 1 data
		wait_cntr = 0;
		//State transition
		ldc_current_state = WAIT_BRD1;				//T4.5

		break;
	case WAIT_BRD1:									//STATE 4.5
		//State action
		wait_cntr++;
		//State transition
		//if(is_LDC_spi_rx_ready()  && wait_cntr > LDC_WAIT_THRESH_MIN){					//T4.6


		if(ldc_stop && is_LDC_spi_rx_ready()){		//T4.8  (FOR DEBUG)
			ldc_current_state = COMPUTE;
		} else if (ldc_sensor_data_check_outside(curr_buf)  && is_LDC_spi_rx_ready()) {
			ldc_current_state = COMPUTE;
		} else if(!ldc_stop && is_LDC_spi_rx_ready()){//T4.16
			ldc_current_state = REPOLL_BRD0;
		} else if(wait_cntr > LDC_WAIT_THRESH){		//T4.15
			ldc_current_state = HANG_ERR;
			issue_warning(WARN_LDC_SPI_HANG1);
		} else {									//T4.12
			ldc_current_state = WAIT_BRD1;
		}
		break;
	case COMPUTE:									//STATE 4.8
		//State action
		ldc_read_reg_multiple_get_data(buf);		//Get Board 1 data
		first_run = 1; 								//Reset first run value for next run
		if(buf[1]&DRDY){
			ldc_data_buf[1] = (buf[3]<<8)|buf[2];
		}
		if(buf[1]&OSC_DEAD){
			issue_warning(WARN_LDC1_OSC_DEAD);
		}


		//Add to sensor data
		//Update buffer
		max_diff = ldc_max_val - base_ldc_val;
		min_diff = base_ldc_val - ldc_min_val;
#ifdef RAW_DATA_PRINT
		dbg_uart_send_byte('l');
		dbg_uart_send_byte(9);//TAB
		hex2ascii_int(ldc_max_val, &buf[0], &buf[1], &buf[2], &buf[3]);	//LDC_MAX_VAL
		buf[4] = 9;//TAB
		response_size = 5;
		dbg_uart_send_string(buf,response_size);
		hex2ascii_int(ldc_min_val, &buf[0], &buf[1], &buf[2], &buf[3]);	//LDC_MIN_VAL
		buf[4] = 9;//TAB
		response_size = 5;
		dbg_uart_send_string(buf,response_size);
		hex2ascii_int(base_ldc_val, &buf[0], &buf[1], &buf[2], &buf[3]);	//LDC_BASE_VAL
		response_size = 4;
		dbg_uart_send_string(buf,response_size);
		dbg_uart_send_byte(10);
		dbg_uart_send_byte(13);
#endif
		if((max_diff > min_diff) && (min_diff > LDC_METAL_H)){
			ldc_result = METAL;
			/*
		} else if ((min_diff < max_diff) && (min_diff < LDC_WOOD_H)){
			ldc_result = OTHER;
			*/
		} else {
			ldc_result = NOT_METAL;
		}

		ldc_final_val = ldc_min_val;

		if(curr_buf == 1){
			sensor_data1.LDC_result = ldc_result;
		} else if(curr_buf == 2){
			sensor_data2.LDC_result = ldc_result;
		} else if (curr_buf == 3){
			sensor_data3.LDC_result = ldc_result;
		} else {
			issue_warning(WARN_MASS_TASK_STOP_ILLEGAL_BUF2);
		}

		//State transition
		ldc_current_state = IDLE;					//T4.9
		break;
	case HANG_ERR:									//STATE 4.10
		//State action
		end_LDC_SPI_transac();
		//State transition
		ldc_current_state = POLL_BRD0;				//T4.18
		break;
	default:
		issue_warning(WARN_ILLEGAL_LDC_SM_STATE);
		ldc_current_state = IDLE;
		break;
	}
}

/* Read LDC1000 register
 * reg_addr: 8-bit register
 * brd: 0,1,2 to access CS0,1,2.
 */
inline uint8_t ldc_read_reg(uint8_t reg_addr, uint8_t brd){
	uint8_t buf[2] = {0x80,0x00};
	buf[0] |= reg_addr;
	init_LDC_SPI_transac(buf, 2,brd);
	while(!is_LDC_spi_rx_ready());
	get_LDC_SPI_rx_data(buf);
	return buf[1];
}

/* Read LDC1000 register, start transaction, non-blocking
 * reg_addr: 8-bit register
 * brd: 0,1,2 to access CS0,1,2.
 */
inline void ldc_read_reg_init(uint8_t reg_addr, uint8_t brd){
	uint8_t buf[2] = {0x00,0x00};
	buf[0] |= reg_addr;
	init_LDC_SPI_transac(buf, 2,brd);
	return;
}

/* Read LDC1000 register, get data from transaction, non-blocking
 * ldc_read_reg_init() must be called previously
 * reg_addr: 8-bit register
 * brd: 0,1,2 to access CS0,1,2.
 */
inline uint8_t ldc_read_reg_get_data(void){
	uint8_t buf[2] = {0x80,0x00};
	get_LDC_SPI_rx_data(buf);
	return buf[1];
}

/* Read LDC1000 multiple registers
 * buf: buffer with data to send (reg address is inserted automatically)
 * reg_addr: 8-bit register
 * brd: 0,1,2 to access CS0,1,2.
 */
inline void ldc_read_reg_multiple(uint8_t reg_addr, uint8_t *buf, uint8_t num_regs, uint8_t brd){
	buf[0] = 0x80|reg_addr;
	init_LDC_SPI_transac(buf,num_regs+1,brd);
	while(!is_LDC_spi_rx_ready());
	get_LDC_SPI_rx_data(buf);
	return;
}

/* Read LDC1000 multiple registers, start transaction, non-blocking
 * buf: buffer with data to send (reg address is inserted automatically)
 * reg_addr: 8-bit register
 * brd: 0,1,2 to access CS0,1,2.
 */
inline void ldc_read_reg_multiple_init(uint8_t reg_addr, uint8_t *buf, uint8_t num_regs, uint8_t brd){
	buf[0] = 0x80|reg_addr;
	init_LDC_SPI_transac(buf,num_regs+1,brd);
	return;
}

/* Read LDC1000 multiple registers, get data from transaction, non-blocking
 * buf: buffer to store data
 * returns number of bytes recieved
 */
inline uint8_t ldc_read_reg_multiple_get_data(uint8_t *buf){
	return get_LDC_SPI_rx_data(buf);
}

void ldc_write_reg(uint8_t reg_addr, uint8_t data, uint8_t brd){
	uint8_t buf[2] = {0x00,0x00};
	buf[0] = reg_addr;
	buf[1] = data;
	init_LDC_SPI_transac(buf, 2, brd);
	while(!is_LDC_spi_rx_ready());
	end_LDC_SPI_transac();
	//for debug
	volatile uint8_t temp = 0;
	temp = ldc_read_reg(0x05, brd);
	return;
}

void ldc_setup(uint8_t brd){
	volatile uint8_t resp = 0;
	//Check ID register
	resp = ldc_read_reg(0x00, brd);
	if(resp != 0x80){
		P1OUT |= BIT0;
		while(1);
	}
	volatile uint8_t readback = 0;
	//Setup in standby mode
	ldc_write_reg(0x0B,0, brd);
	//Write Rp max/min values
	ldc_write_reg(0x01,0x00, brd);	//RP_MAX
	ldc_write_reg(0x02,0x3f, brd);	//RP_MIN
	//Watchdog frequency
	ldc_write_reg(0x03,25, brd);//179
	//Configuration
	ldc_write_reg(0x04,BIT4|BIT2|BIT1|BIT0, brd);	//Amplitude=4V, Response time = 6144, 4kHz sampling rate - 0x17
	//Clock configuration
	ldc_write_reg(0x05,BIT1, brd);	//Enable crystal
	//INTB configuration
	ldc_write_reg(0x0A,BIT2, brd);	//INTB indicates data ready

	//check
	readback = ldc_read_reg(0x05, brd);
	//Power configuration
	ldc_write_reg(0x0B,BIT0, brd);	//Active mode

	readback = ldc_read_reg(0x05, brd);
	return;
}

uint16_t ldc_get_proximity(uint8_t brd){
	uint8_t buf[8];
	ldc_read_reg_multiple(0x20,buf,6, brd);
	return (buf[3]<<8)|buf[2];	//Proximity
}

/** END LDC Task Functions **/

/** Mass Task Functions **/
void mass_task(void){
	uint8_t i = 0;
	uint8_t hist_index = 0;
	static uint16_t wait_cntr = 0;
	uint16_t mass_hist_val1 = 0;
	uint16_t mass_hist_val2 = 0;
	uint16_t mass_hist_bin1 = 0;
	uint16_t mass_hist_bin2 = 0;
	static uint8_t curr_buf = 0;
	uint8_t mass_result = 0;
	uint8_t response_size = 0;
	uint8_t buf[8];

	switch(mass_current_state){
	case MASS_IDLE:							//STATE 2.1
		//State action
		mass_buf_ptr = 0;
		mass_end = 0;
		wait_cntr = 0;
		//State transistion
		if(mass_run == 1){ //For debug
			mass_current_state = MASS_WAIT;
		} else if(curr_buf = mass_sensor_data_check_inside()){ //For debug
			mass_current_state = MASS_WAIT;
		} else {
			mass_current_state = MASS_IDLE;
		}
		break;
	case MASS_WAIT: 						//STATE 2.2
		//State action
		mass_run = 0;
		wait_cntr++;
		//State transistion
		if(adc_data_ready_flags[19] == 1 && wait_cntr >= MASS_WAIT_CNTR){
			adc_data_ready_flags[19] = 0;
			mass_current_state = MASS_GET;
			wait_cntr = 0;
		} else {
			mass_current_state = MASS_WAIT;
		}
		break;
	case MASS_GET:							//STATE 2.3
		//State action
		for(i=0;i<NUM_MASS_SENSORS;i++){
			mass_data_buf[i][mass_buf_ptr]= adc_output_buffer[19+i];
			hist_index = adc_output_buffer[19+i] >> 5;
			mass_hist_buf[hist_index]++;
		}
		mass_buf_ptr++;
		if(mass_buf_ptr >= MASS_BUF_SIZE){
			mass_buf_ptr = 0;
		}

		//State transistion
		if(mass_end == 1){ //For debug
			mass_current_state = MASS_COMPUTE;
		} else if(mass_sensor_data_check_outside(curr_buf)){
			mass_current_state = MASS_COMPUTE;
		} else {
			mass_current_state = MASS_WAIT;

		}
		break;
	case MASS_COMPUTE:						//STATE 2.4
		//State action
		mass_hist_bin1 = 0;					// Index of Max Value in Histogram
		mass_hist_val1 = mass_hist_buf[0]; // Current Max Value in Histogram
		for(i=1; i<32; i++){
			if(mass_hist_buf[i] > mass_hist_val1){
				mass_hist_bin1 = i;
				mass_hist_val1 = mass_hist_buf[i];
			}
		}
		mass_hist_bin2 = 0;					// Index of Max2 Value in Histogram
		mass_hist_val2 = 0;
		for(i=0; i<32; i++){
			if(mass_hist_buf[i] > mass_hist_val2 && (i+1 != mass_hist_bin1 && i-1 != mass_hist_bin1 && i != mass_hist_bin1)){
				mass_hist_bin2 = i;
				mass_hist_val2 = mass_hist_buf[i];
			}
			//Clear buffer
			mass_hist_buf[i] = 0;
		}

		//Find the delta value
		if(mass_hist_bin1 == 0 || mass_hist_bin2 == 0){
			hist_delta_bin = 0;
		} else if(mass_hist_bin1 > mass_hist_bin2){
			hist_delta_bin = mass_hist_bin1 - mass_hist_bin2;
		} else {
			hist_delta_bin = mass_hist_bin2 - mass_hist_bin1;
		}
#ifdef RAW_DATA_PRINT
		dbg_uart_send_byte('m');
		dbg_uart_send_byte(9);//TAB
		hex2ascii_int(hist_delta_bin, &buf[0], &buf[1], &buf[2], &buf[3]);
		response_size = 4;
		dbg_uart_send_string(buf,response_size);
		dbg_uart_send_byte(10);
		dbg_uart_send_byte(13);
#endif

		//Add to sensor data
		//Update buffer

		if(hist_delta_bin == 0){
			//dbg_uart_send_string("Other",5);
			mass_result = OTHER;
		} else if(hist_delta_bin >= 16) {
			mass_result = HEAVY;
			//dbg_uart_send_string("Glass/Heavy Other",17);
		} else{
			mass_result = LIGHT;
			//dbg_uart_send_string("Plastic/Other",13);
		}


		if(curr_buf == 1){
			sensor_data1.mass_result = mass_result;
		} else if(curr_buf == 2){
			sensor_data2.mass_result = mass_result;
		} else if (curr_buf == 3){
			sensor_data3.mass_result = mass_result;
		} else {
			issue_warning(WARN_MASS_TASK_STOP_ILLEGAL_BUF2);
		}

		//TODO: Final computation of final_result
		if(ldc_final_val < LDC_METAL_H) {
			dbg_uart_send_string("Metal",5);
			dbg_uart_send_byte(13);		//CR
			dbg_uart_send_byte(10);		//Line feed
			total_count.metal++;
			bin_int_request = BIN_METAL;
		} else if(opt_max_time == 0 && hist_delta_bin == 0){
			dbg_uart_send_string("Other",5);
			dbg_uart_send_byte(13);		//CR
			dbg_uart_send_byte(10);		//Line feed
			total_count.other++;
			bin_int_request = BIN_OTHER;
		} else if(opt_max_time < 20 && hist_delta_bin >= 16) {
			dbg_uart_send_string("Glass",5);
			dbg_uart_send_byte(13);		//CR
			dbg_uart_send_byte(10);		//Line feed
			total_count.glass++;
			bin_int_request = BIN_GLASS;
		} else if (hist_delta_bin >= 16) {
			dbg_uart_send_string("Glass",5);
			dbg_uart_send_byte(13);		//CR
			dbg_uart_send_byte(10);		//Line feed
			total_count.glass++;
			bin_int_request = BIN_GLASS;
		} else if(opt_max_time < 20) {
			dbg_uart_send_string("Plastic",7);
			dbg_uart_send_byte(13);		//CR
			dbg_uart_send_byte(10);		//Line feed
			total_count.plastic++;
			bin_int_request = BIN_PLASTIC;
		} else{
			dbg_uart_send_string("Other",5);
			dbg_uart_send_byte(13);		//CR
			dbg_uart_send_byte(10);		//Line feed
			total_count.other++;
			bin_int_request = BIN_OTHER;
		}

		// Send flags
		bin_request = 1;
		bin_ready = 0;
		step_request = 1;

		// Release buffers
		if(curr_buf == 1){
			sensor_data1.obj_en = 0;
			dbg_uart_send_string("BR1",3);
		} else if(curr_buf == 2){
			sensor_data2.obj_en = 0;
			dbg_uart_send_string("BR2",3);
		} else if (curr_buf == 3){
			sensor_data3.obj_en = 0;
			dbg_uart_send_string("BR3",3);
		} else {
			issue_warning(WARN_MASS_TASK_STOP_ILLEGAL_BUF3);
		}

		//State transistion
		mass_current_state = MASS_IDLE;
		break;
	default:
		issue_warning(WARN_ILLEGAL_MASS_SM_STATE);
		mass_current_state = MASS_IDLE;
		break;
	}
}

uint8_t mass_sensor_data_check_inside(){
	uint16_t currChimPos = get_curr_chimney_pos();
	if(sensor_data1.obj_en && is_inside_interval(currChimPos, sensor_data1.mass_start, sensor_data1.mass_end)){
		return 1;
	} else if(sensor_data2.obj_en && is_inside_interval(currChimPos, sensor_data2.mass_start, sensor_data2.mass_end)){
		return 2;
	} else if(sensor_data3.obj_en && is_inside_interval(currChimPos, sensor_data3.mass_start, sensor_data3.mass_end)){
		return 3;
	}
	return 0;
}

uint8_t mass_sensor_data_check_outside(uint8_t curr_buf){
	uint16_t currChimPos = get_curr_chimney_pos();
	if(curr_buf == 1) {
		if (!is_inside_interval(currChimPos, sensor_data1.mass_start, sensor_data1.mass_end)){
			return 1;
		}
		return 0;
	} else if (curr_buf == 2){
		if(!is_inside_interval(currChimPos, sensor_data2.mass_start, sensor_data2.mass_end)){
			return 1;
		}
		return 0;
	} else if (curr_buf == 3) {
		if(!is_inside_interval(currChimPos, sensor_data3.mass_start, sensor_data3.mass_end)){
			return 1;
		}
		return 0;
	}
	issue_warning(WARN_MASS_TASK_STOP_ILLEGAL_BUF);
	return 0;
}

/** END Mass Task Function **/


/** Optical Task Functions **/
void opt_task(void){
	uint8_t i = 0;
	static uint16_t wait_cntr = 0;
	static uint8_t curr_buf = 0;
	uint8_t opt_result = 0;
	uint8_t buf[8];
	uint8_t response_size = 0;

	switch(opt_current_state){
	case OPT_IDLE:							//STATE 3.1
		//State action
		opt_buf_ptr = 0;
		opt_end = 0;
		wait_cntr = 0;

		//State transistion
		if(opt_run == 1){	//For debug
			opt_current_state = OPT_WAIT;
		} else if(curr_buf = opt_sensor_data_check_inside()){	//For debug
			opt_current_state = OPT_WAIT;
		} else {
			opt_current_state = OPT_IDLE;
		}
		break;
	case OPT_WAIT: 						//STATE 3.2
		//State action
		opt_run = 0;
		wait_cntr++;
		//State transistion
		if(adc_data_ready_flags[27] == 1 && wait_cntr >= OPT_WAIT_CNTR){
			adc_data_ready_flags[27] = 0;
			opt_current_state = OPT_GET;
			wait_cntr = 0;
		} else {
			opt_current_state = OPT_WAIT;
		}
		break;
	case OPT_GET:							//STATE 3.3
		//State action
#ifdef OPTDUMP
		for(i=0;i<NUM_OPT_SENSORS;i++){
			opt_data_buf[i][opt_buf_ptr]= adc_output_buffer[27+i];
		}
		opt_buf_ptr++;
		if(opt_buf_ptr >= OPT_BUF_SIZE){
			opt_buf_ptr = 0;
		}

#endif

#ifdef OPTVAL
		for(i=0;i<NUM_OPT_SENSORS; i++){
			if(adc_output_buffer[27+i] <= OPT_CROSS_THRESH){
				opt_data_time_buf[i]++;
			}
			if(adc_output_buffer[27+i] < opt_data_low_buf[i]){
				opt_data_low_buf[i] = adc_output_buffer[27+i];
			}

		}
#endif
		//State transistion
		if(opt_end == 1){ // For debug
			opt_current_state = OPT_COMPUTE;
		} else if (opt_sensor_data_check_outside(curr_buf)) {
			opt_current_state = OPT_COMPUTE;
		} else {
			opt_current_state = OPT_WAIT;
		}
		break;
	case OPT_COMPUTE:						//STATE 3.4
#ifdef OPTVAL
		opt_low_val = opt_data_low_buf[1];
		opt_max_time = opt_data_time_buf[1];
		//State action
		for(i=2;i<NUM_OPT_SENSORS;i++){
			if(opt_data_low_buf[i] < opt_low_val){
				opt_low_val = opt_data_low_buf[i];
			}
			if(opt_data_time_buf[i] > opt_max_time){
				opt_max_time = opt_data_time_buf[i];
			}
			//Clear buffer
			opt_data_time_buf[i] = 0;
			opt_data_low_buf[i] = 0xFFFF;
		}
		//Clear buffer for [0]th value
		opt_data_time_buf[0] = 0;
		opt_data_low_buf[0] = 0xFFFF;

		//Update buffer
		if(opt_max_time == 0){
			//dbg_uart_send_string("Other",5);
			opt_result = OTHER;
		} else if(opt_max_time < 20) {
			//dbg_uart_send_string("Plastic/Glass",13);
			opt_result = TRANSPARENT;
		} else{
			//dbg_uart_send_string("Other",5);
			opt_result = OPAQUE;
		}

		if(curr_buf == 1){
			sensor_data1.opt_result = opt_result;
		} else if(curr_buf == 2){
			sensor_data2.opt_result = opt_result;
		} else if (curr_buf == 3){
			sensor_data3.opt_result = opt_result;
		} else {
			issue_warning(WARN_OPT_TASK_STOP_ILLEGAL_BUF2);
		}
#endif
#ifdef RAW_DATA_PRINT
		dbg_uart_send_byte('o');
		dbg_uart_send_byte(9);//TAB
		hex2ascii_int(opt_low_val, &buf[0], &buf[1], &buf[2], &buf[3]);
		response_size = 4;
		dbg_uart_send_string(buf,response_size);
		dbg_uart_send_byte(9);//TAB
		hex2ascii_int(opt_max_time, &buf[0], &buf[1], &buf[2], &buf[3]);
		response_size = 4;
		dbg_uart_send_string(buf,response_size);
		dbg_uart_send_byte(10);
		dbg_uart_send_byte(13);
#endif

		//State transistion
		opt_current_state = OPT_IDLE;
		break;
	default:
		issue_warning(WARN_ILLEGAL_OPT_SM_STATE);
		opt_current_state = OPT_IDLE;
		break;
	}
}

uint8_t opt_sensor_data_check_inside(){
	uint16_t currChimPos = get_curr_chimney_pos();
	if(sensor_data1.obj_en && is_inside_interval(currChimPos, sensor_data1.opt_start, sensor_data1.opt_end)){
		return 1;
	} else if(sensor_data2.obj_en && is_inside_interval(currChimPos, sensor_data2.opt_start, sensor_data2.opt_end)){
		return 2;
	} else if(sensor_data3.obj_en && is_inside_interval(currChimPos, sensor_data3.opt_start, sensor_data3.opt_end)){
		return 3;
	}
	return 0;
}

uint8_t opt_sensor_data_check_outside(uint8_t curr_buf){
	uint16_t currChimPos = get_curr_chimney_pos();
	if(curr_buf == 1) {
		if (!is_inside_interval(currChimPos, sensor_data1.opt_start, sensor_data1.opt_end)){
			return 1;
		}
		return 0;
	} else if (curr_buf == 2){
		if(!is_inside_interval(currChimPos, sensor_data2.opt_start, sensor_data2.opt_end)){
			return 1;
		}
		return 0;
	} else if (curr_buf == 3) {
		if(!is_inside_interval(currChimPos, sensor_data3.opt_start, sensor_data3.opt_end)){
			return 1;
		}
		return 0;
	}
	issue_warning(WARN_OPT_TASK_STOP_ILLEGAL_BUF);
	return 0;
}

/** END Optical Task Function **/

/** I2C/Motor Enable Task Function **/

void motor_enable_setup(void){
	uint8_t buf[4];
	end_I2C_transac();	//Force stop condition to clear any pending transactions
	//Configure all I/O of IO expander as outputs
	buf[0] = 3;
	buf[1] = 0x00;		//All as outputs
	init_I2C_transac(buf,2,I2C_ADDR_MOTOR_IO);
	while(!is_I2C_rx_ready());
	end_I2C_transac();
	//Set all IO expander outputs to 0 to disable
	buf[0] = 1;
	buf[1] = 0x00;
	//TODO: See if stepper motor needs to be changed to active low enable
	init_I2C_transac(buf,2,I2C_ADDR_MOTOR_IO);
	while(!is_I2C_rx_ready());
	end_I2C_transac();
	return;
}

void i2c_task(void){
	uint8_t buf[80];
	uint8_t i = 0;
	switch(i2cCurrState){
	case I2C_SEND_MOTOR:
		//State action
		buf[0] = 1;
		buf[1] = (sort_motor_enable<<3)|(bin_motor_enable<<2)|(compact_motor_enable<<1)|chimney_motor_enable;
		init_I2C_transac(buf,2,I2C_ADDR_MOTOR_IO);
#ifdef NO_UI
		//State transistion
		i2cCurrState = I2C_WAIT1;
		break;
	case I2C_WAIT1:
		//State action
		// Do nothing
		//State transistion
		if(is_I2C_rx_ready()){
			i2cCurrState = I2C_HOME_CURSOR;
		} else {
			i2cCurrState = I2C_WAIT1;
		}
		break;
	case I2C_HOME_CURSOR:
		end_I2C_transac();
		buf[0] = 0xFE;
		buf[1] = 0x45;
		buf[2] = 0x00; // Set to col1, row1
		init_I2C_transac(buf,3,I2C_ADDR_UI_IO);
		i2cCurrState = I2C_WAIT2;
		break;
	case I2C_WAIT2:
		if(is_I2C_rx_ready()){
			i2cCurrState = I2C_SEND_UI;
		} else {
			i2cCurrState = I2C_WAIT2;
		}
		break;
	case I2C_SEND_UI:
		//State action
		end_I2C_transac();
		//Send string to UI
		for(i=0; i<80;i++){
			buf[i] = ui_buf[i];
		}
		if(!total_count.plastic_full) {
			dec2ascii_byte(total_count.plastic, &buf[UI_PLASTIC_INDEX], &buf[UI_PLASTIC_INDEX+1], &buf[UI_PLASTIC_INDEX+2]);
		} else {
			buf[UI_PLASTIC_INDEX] = 'F';
			buf[UI_PLASTIC_INDEX+1] = 'U';
			buf[UI_PLASTIC_INDEX+2] = 'L';
			buf[UI_PLASTIC_INDEX+3] = 'L';
		}
		if(!total_count.glass_full) {
			dec2ascii_byte(total_count.glass, &buf[UI_GLASS_INDEX], &buf[UI_GLASS_INDEX+1], &buf[UI_GLASS_INDEX+2]);
		} else {
			buf[UI_GLASS_INDEX] = 'F';
			buf[UI_GLASS_INDEX+1] = 'U';
			buf[UI_GLASS_INDEX+2] = 'L';
			buf[UI_GLASS_INDEX+3] = 'L';
		}
		if(!total_count.metal_full) {
			dec2ascii_byte(total_count.metal, &buf[UI_METAL_INDEX], &buf[UI_METAL_INDEX+1], &buf[UI_METAL_INDEX+2]);
		} else {
			buf[UI_METAL_INDEX] = 'F';
			buf[UI_METAL_INDEX+1] = 'U';
			buf[UI_METAL_INDEX+2] = 'L';
			buf[UI_METAL_INDEX+3] = 'L';
		}
		if(!total_count.other_full) {
			dec2ascii_byte(total_count.other, &buf[UI_OTHER_INDEX], &buf[UI_OTHER_INDEX+1], &buf[UI_OTHER_INDEX+2]);
		} else {
			buf[UI_OTHER_INDEX] = 'F';
			buf[UI_OTHER_INDEX+1] = 'U';
			buf[UI_OTHER_INDEX+2] = 'L';
			buf[UI_OTHER_INDEX+3] = 'L';
		}
		init_I2C_transac(buf,80,I2C_ADDR_UI_IO);
#endif
		//State transistion
		i2cCurrState = I2C_WAIT3;
		break;
	case I2C_WAIT3:
		//State action
		//Do nothing
		//State transistion
		if(is_I2C_rx_ready()){
			i2cCurrState = I2C_CLOSE;
		} else {
			i2cCurrState = I2C_WAIT3;
		}
		break;
	case I2C_CLOSE:
		//State action
		end_I2C_transac();
		//State transistion
		i2cCurrState = I2C_SEND_MOTOR;
		break;
	default:
		issue_warning(WARN_ILLEGAL_I2C_SM_STATE);
		i2cCurrState = I2C_SEND_MOTOR;
		break;
	}

	return;
}

void motor_spi_task(void){
	uint16_t resp = 0;

	switch(motorCurrState){
	case MOTOR_INIT:							//STATE 10.1
		//State action
		init_motor_SPI_transac(CHIMNEY_CMD, 1);
		//State transistion
		motorCurrState = MOTOR_WAIT1;
		break;
	case MOTOR_WAIT1:
		//State action
		//Do nothing
		//State transistion
		if(is_motor_spi_rx_ready()){
			motorCurrState = MOTOR1_READ;
		} else {
			motorCurrState = MOTOR_WAIT1;
		}
		break;
	case MOTOR1_READ: 						//STATE 10.2
		//State action
		resp = get_motor_SPI_rx_data();
		check_chimney_resp(resp);
		init_motor_SPI_transac(COMPACT_CMD, 2);
		//State transistion
		motorCurrState = MOTOR_WAIT2;
		break;
	case MOTOR_WAIT2:
		//State action
		//Do nothing
		//State transistion
		if(is_motor_spi_rx_ready()){
			motorCurrState = MOTOR2_READ;
		}else {
			motorCurrState = MOTOR_WAIT2;
		}

		break;
	case MOTOR2_READ:							//STATE 10.3
		//State action
		resp = get_motor_SPI_rx_data();
		check_compact_resp(resp);
		init_motor_SPI_transac(BIN_CMD, 3);
		//State transistion
		motorCurrState = MOTOR_WAIT3;
		break;
	case MOTOR_WAIT3:
		//State action
		//Do nothing
		//State transistion
		if(is_motor_spi_rx_ready()){
			motorCurrState = MOTOR3_READ;
		}else {
			motorCurrState = MOTOR_WAIT3;
		}
		break;
	case MOTOR3_READ:						//STATE 10.4
		//State action
		resp = get_motor_SPI_rx_data();
		check_bin_resp(resp);
		init_motor_SPI_transac(CHIMNEY_CMD, 1);
		//State transistion
		motorCurrState = MOTOR_WAIT4;
		break;
	case MOTOR_WAIT4:
		//State action
		//Do nothing
		//State transistion
		if(is_motor_spi_rx_ready()){
			motorCurrState = MOTOR1_READ;
		}else {
			motorCurrState = MOTOR_WAIT4;
		}
		break;
	default:
		issue_warning(WARN_ILLEGAL_MOT_SM_STATE);
		motorCurrState = MOTOR_INIT;
		break;
	}
	return;
}

void check_chimney_resp(uint16_t resp){
	if(resp & BIT0) issue_warning(WARN_OPEN_CHIMNEY_OL_OFF);
	if(resp & BIT1) issue_warning(WARN_OPEN_CHIMNEY_OL_ON);
	if(resp & BIT2) issue_warning(WARN_OPEN_CHIMNEY_VS_UV);
	if(resp & BIT3) issue_warning(WARN_OPEN_CHIMNEY_VDD_OV);
	if(resp & BIT4) issue_warning(WARN_OPEN_CHIMNEY_ILIM);
	if(resp & BIT5) issue_warning(WARN_OPEN_CHIMNEY_TWARN);
	if(resp & BIT6) issue_warning(WARN_OPEN_CHIMNEY_TSD);
	if(resp & BIT8) issue_warning(WARN_OPEN_CHIMNEY_OC_LS1);
	if(resp & BIT9) issue_warning(WARN_OPEN_CHIMNEY_OC_LS2);
	if(resp & BITA) issue_warning(WARN_OPEN_CHIMNEY_OC_HS1);
	if(resp & BITB) issue_warning(WARN_OPEN_CHIMNEY_OC_HS2);
	if(resp & BITE) issue_warning(WARN_OPEN_CHIMNEY_SGND_OFF);
	if(resp & BITF) issue_warning(WARN_OPEN_CHIMNEY_SBAT_OFF);
}


void check_compact_resp(uint16_t resp){
	if(resp & BIT0) issue_warning(WARN_OPEN_COMPACT_OL_OFF);
	if(resp & BIT1) issue_warning(WARN_OPEN_COMPACT_OL_ON);
	if(resp & BIT2) issue_warning(WARN_OPEN_COMPACT_VS_UV);
	if(resp & BIT3) issue_warning(WARN_OPEN_COMPACT_VDD_OV);
	//if(resp & BIT4) issue_warning(WARN_OPEN_COMPACT_ILIM);
	if(resp & BIT5) issue_warning(WARN_OPEN_COMPACT_TWARN);
	if(resp & BIT6) issue_warning(WARN_OPEN_COMPACT_TSD);
	if(resp & BIT8) issue_warning(WARN_OPEN_COMPACT_OC_LS1);
	if(resp & BIT9) issue_warning(WARN_OPEN_COMPACT_OC_LS2);
	if(resp & BITA) issue_warning(WARN_OPEN_COMPACT_OC_HS1);
	if(resp & BITB) issue_warning(WARN_OPEN_COMPACT_OC_HS2);
	if(resp & BITE) issue_warning(WARN_OPEN_COMPACT_SGND_OFF);
	if(resp & BITF) issue_warning(WARN_OPEN_COMPACT_SBAT_OFF);
}

void check_bin_resp(uint16_t resp){
	if(resp & BIT0) issue_warning(WARN_OPEN_BIN_OL_OFF);
	if(resp & BIT1) issue_warning(WARN_OPEN_BIN_OL_ON);
	if(resp & BIT2) issue_warning(WARN_OPEN_BIN_VS_UV);
	if(resp & BIT3) issue_warning(WARN_OPEN_BIN_VDD_OV);
	if(resp & BIT4) issue_warning(WARN_OPEN_BIN_ILIM);
	if(resp & BIT5) issue_warning(WARN_OPEN_BIN_TWARN);
	if(resp & BIT6) issue_warning(WARN_OPEN_BIN_TSD);
	if(resp & BIT8) issue_warning(WARN_OPEN_BIN_OC_LS1);
	if(resp & BIT9) issue_warning(WARN_OPEN_BIN_OC_LS2);
	if(resp & BITA) issue_warning(WARN_OPEN_BIN_OC_HS1);
	if(resp & BITB) issue_warning(WARN_OPEN_BIN_OC_HS2);
	if(resp & BITE) issue_warning(WARN_OPEN_BIN_SGND_OFF);
	if(resp & BITF) issue_warning(WARN_OPEN_BIN_SBAT_OFF);
}


uint8_t is_inside_interval(uint16_t test_val, uint16_t low_thresh, uint16_t high_thresh){
	if(low_thresh < high_thresh) {
		if(test_val > low_thresh && test_val <= high_thresh){
			return 1;
		}
	} else {
		if (test_val > low_thresh || test_val <= high_thresh){
			return 1;
		}
	}
	return 0;
}


/** END Motor Enable Task Function **/

/** Chimney Task Function **/
void chimney_task(void){
	uint16_t currChimPos = get_curr_chimney_pos();
	volatile uint8_t temp;
	switch(chimneyCurrState){
	case CHIM_IDLE_OFF:
		//State action
		//Do nothing
		//State transistion
		if(sysState == SYS_ON){
			chimneyCurrState = CHIM_CHECK_PADDLE;
		} else {
			chimneyCurrState = CHIM_IDLE_OFF;
		}
		break;
	case CHIM_CHECK_PADDLE:
		//State action
		set_chimney_speed(0);
		chimney_motor_enable = 0;
		//State transistion
		if(check_chim_paddle_pos()) {
			chimneyCurrState = CHIM_IDLE_ON;
		} else {
			chimneyCurrState = CHIM_TURN_PADDLE;
		}
		break;
	case CHIM_TURN_PADDLE:
		//State action
		set_chimney_speed(CHIM_SPEED);
		chimney_motor_enable = 1;
		//State transistion
		if(check_chim_paddle_pos()){
			chimneyCurrState = CHIM_IDLE_ON;
		} else {
			chimneyCurrState = CHIM_TURN_PADDLE;
		}
		break;
	case CHIM_IDLE_ON:
		//State action
		set_chimney_speed(0);
		chimney_motor_enable = 0;
		//State transistion
		if(is_entry_door_open()) {
			chimneyCurrState = CHIM_CHECK_POS;
		} else if (sysState == SYS_OFF || sysState == SYS_MAINT) {
			chimneyCurrState = CHIM_IDLE_OFF;
		} else {
			chimneyCurrState = CHIM_IDLE_ON;
		}
		break;
	case CHIM_CHECK_POS:
		//State action
		// Do nothing
		//State transistion
		temp = check_chim_next_pos();
		if(temp){
			chimneyCurrState = CHIM_STOP;
		} else {
			chimneyCurrState = CHIM_RUN2DOOR;
		}
		break;
	case CHIM_RUN2DOOR:
		//State action
		set_chimney_speed(CHIM_SPEED);
		chimney_motor_enable = 1;
		//State transistion
		temp = check_chim_next_pos();
		if(temp) {
			chimneyCurrState = CHIM_STOP;
		} else {
			chimneyCurrState = CHIM_RUN2DOOR;
		}
		break;
	case CHIM_STOP:
		//State action
		set_chimney_speed(0);
		chimney_motor_enable = 0;
		last_insert_pos = get_curr_chimney_pos();
		//State transistion
		if(!is_entry_door_open()){
			chimneyCurrState = CHIM_INIT_BUF;
		} else {
			chimneyCurrState = CHIM_STOP;
		}
		break;
	case CHIM_INIT_BUF:
		//State action
		if(!sensor_data1.obj_en){
			sensor_data1.obj_en = 1;
			sensor_data1.mass_start = currChimPos + MASS_OFFSET_START;
			sensor_data1.mass_end = currChimPos + MASS_OFFSET_END ;
			sensor_data1.opt_start = currChimPos + OPT_OFFSET_START ;
			sensor_data1.opt_end = currChimPos + OPT_OFFSET_END ;
			sensor_data1.LDC_start = currChimPos + LDC_OFFSET_START;
			sensor_data1.LDC_end = currChimPos + LDC_OFFSET_END;
			dbg_uart_send_string("BI1",3);
		} else if(!sensor_data2.obj_en){
			sensor_data2.obj_en = 1;
			sensor_data2.mass_start = currChimPos + MASS_OFFSET_START;
			sensor_data2.mass_end = currChimPos + MASS_OFFSET_END ;
			sensor_data2.opt_start = currChimPos + OPT_OFFSET_START ;
			sensor_data2.opt_end = currChimPos + OPT_OFFSET_END ;
			sensor_data2.LDC_start = currChimPos + LDC_OFFSET_START;
			sensor_data2.LDC_end = currChimPos + LDC_OFFSET_END;
			dbg_uart_send_string("BI2",3);
		} else if(!sensor_data3.obj_en){
			sensor_data3.obj_en = 1;
			sensor_data3.mass_start = currChimPos + MASS_OFFSET_START;
			sensor_data3.mass_end = currChimPos + MASS_OFFSET_END ;
			sensor_data3.opt_start = currChimPos + OPT_OFFSET_START ;
			sensor_data3.opt_end = currChimPos + OPT_OFFSET_END ;
			sensor_data3.LDC_start = currChimPos + LDC_OFFSET_START;
			sensor_data3.LDC_end = currChimPos + LDC_OFFSET_END;
			dbg_uart_send_string("BI3",3);
		} else {
			issue_warning(WARN_NO_AVAIL_BUF);
			dbg_uart_send_string("BN",2);
		}
		//State transistion
		chimneyCurrState = CHIM_RUNALL;
		break;
	case CHIM_RUNALL:
		//State action
		set_chimney_speed(CHIM_SPEED);
		chimney_motor_enable = 1;
		//State transistion
		if(check_chim_full_rot()){
			chimneyCurrState = CHIM_IDLE_ON;
			last_paddle = 0;
		} else if (is_entry_door_open()) {
			chimneyCurrState = CHIM_CHECK_POS;
		} else {
			chimneyCurrState = CHIM_RUNALL;
		}
		break;
	default:
		issue_warning(WARN_ILLEGAL_CHIM_SM_STATE);
		chimneyCurrState = CHIM_IDLE_OFF;
		break;
	}
	return;
}

uint16_t get_curr_chimney_pos(void){
	volatile uint16_t currChimPos = (adc_output_buffer[3] <<4 );
	return currChimPos; // Hall pos 1
}

/* Check is chimney motor encoder val is at 90 degrees with the door */
uint8_t check_chim_paddle_pos(void){
	//uint16_t currChimPos2 = get_curr_chimney_pos();
	volatile uint16_t currChimPos2 = get_curr_chimney_pos();
	volatile uint8_t ret_val = 0;
	if(currChimPos2 < DEG0_POS_H && currChimPos2 >= DEG0_POS_L) ret_val = 1;
	if(currChimPos2 < DEG90_POS_H && currChimPos2 >= DEG90_POS_L) ret_val = 2;
	if(currChimPos2 < DEG180_POS_H && currChimPos2 >= DEG180_POS_L) ret_val = 3;
	if(currChimPos2 < DEG270_POS_H && currChimPos2 >= DEG270_POS_L) ret_val = 4;
	return ret_val;
}

uint8_t check_chim_next_pos(void){

	volatile uint16_t currChimPos = get_curr_chimney_pos();
	if(currChimPos < DEG0_POS_H && currChimPos >= DEG0_POS_L && last_paddle != 1) {
		last_paddle = 1;
		return 1;
	}
	if(currChimPos < DEG90_POS_H && currChimPos >= DEG90_POS_L && last_paddle != 2) {
		last_paddle = 2;
		return 2;
	}
	if(currChimPos < DEG180_POS_H && currChimPos >= DEG180_POS_L && last_paddle !=3) {
		last_paddle = 3;
		return 3;
	}
	if(currChimPos < DEG270_POS_H && currChimPos >= DEG270_POS_L && last_paddle != 4) {
		last_paddle = 4;
		return 4;
	}
	return 0;
}

uint8_t check_chim_full_rot(void){
	volatile uint16_t currChimPos = get_curr_chimney_pos();
	if(last_insert_pos - FULL_ROT_LOW_THRESH > last_insert_pos - FULL_ROT_HIGH_THRESH) {
			if(currChimPos < last_insert_pos - FULL_ROT_LOW_THRESH &&  currChimPos >= last_insert_pos - FULL_ROT_HIGH_THRESH) {
				return 1;
			}
		} else {
			if(currChimPos < last_insert_pos - FULL_ROT_LOW_THRESH ||  currChimPos >= last_insert_pos - FULL_ROT_HIGH_THRESH) {
				return 1;
			}
		}
	return 0;
}

uint8_t is_entry_door_open(void){
	return !(P6IN & BIT7);
}

/** END Chimney Task Function **/

/** Bins Function **/

uint8_t is_bin_door_open(void){
	return !(P6IN & BIT6);
}

uint16_t get_curr_bin_pos(void){
	volatile uint16_t currBinPos = (adc_output_buffer[5] <<4 );
	return currBinPos; // Hall pos 3
}

uint16_t get_target_distance(uint16_t pos, bin_t bin_requested){
	uint8_t distance = 0;
	switch(bin_requested){
	case BIN_GLASS:
		if(GLASS_DUMP_AVG > pos){
			distance = GLASS_DUMP_AVG - pos;
		} else {
			distance = pos - GLASS_DUMP_AVG;
		}
		break;
	case BIN_PLASTIC:
		if(PLASTIC_DUMP_AVG > pos){
			distance = PLASTIC_DUMP_AVG - pos;
		} else {
			distance = pos - PLASTIC_DUMP_AVG;
		}
		break;
	case BIN_METAL:
		if(METAL_DUMP_AVG > pos){
			distance = METAL_DUMP_AVG - pos;
		} else {
			distance = pos - METAL_DUMP_AVG;
		}
		break;
	case BIN_OTHER:
		if(OTHER_DUMP_AVG > pos){
			distance = OTHER_DUMP_AVG - pos;
		} else {
			distance = pos - OTHER_DUMP_AVG;
		}
		break;
	default:
		issue_warning(WARN_ILLEGAL_BIN_REQUEST_SM_STATE);
	}
	return distance;
}
//Check if bin is at requested location
uint8_t check_bin_request_pos(bin_t bin_requested){
	volatile uint16_t currBinPos = get_curr_bin_pos();
	uint8_t ret_val = 0;
	switch(bin_requested){
	case BIN_GLASS:
		if(currBinPos < GLASS_DUMP_H && currBinPos >= GLASS_DUMP_L){
			ret_val = 1;
		}
		break;
	case BIN_PLASTIC:
		if(currBinPos < PLASTIC_DUMP_H && currBinPos >= PLASTIC_DUMP_L){
			ret_val = 1;
		}
		break;
	case BIN_METAL:
		if(currBinPos < METAL_DUMP_H && currBinPos >= METAL_DUMP_L){
			ret_val = 1;
		}
		break;
	case BIN_OTHER:
		if(currBinPos < OTHER_DUMP_H && currBinPos >= OTHER_DUMP_L){
			ret_val = 1;
		}
		break;
	default:
		issue_warning(WARN_ILLEGAL_BIN_REQUEST_SM_STATE);
	}
	return ret_val;
}

/*Check bin for maintenance mode */
bin_t check_bin_nearest_pos(void) {
	//TODO: Fix wraparound issues with encoders
	volatile uint16_t currBinPos = get_curr_bin_pos();
	if(currBinPos <= GLASS_THRESH  && currBinPos > PLASTIC_THRESH){
		return BIN_GLASS;
	}
	else if(currBinPos <= PLASTIC_THRESH  && currBinPos > METAL_THRESH){
		return BIN_PLASTIC;
	}
	else if(currBinPos <= METAL_THRESH && currBinPos > OTHER_THRESH ){
		return BIN_GLASS;
	}
	else {
		return BIN_OTHER;
	}
}

/* Check is bin motor encoder val is at 90 degrees with the door - Maint mode*/
uint8_t check_bin_pos(bin_t bin_requested){
	volatile uint16_t currBinPos = get_curr_bin_pos();
	uint8_t ret_val = 0;
	switch(bin_requested){
	case BIN_GLASS:
		if(currBinPos < GLASS_DOOR_H && currBinPos >= GLASS_DOOR_L){
			ret_val = 1;
		}
		break;
	case BIN_PLASTIC:
		if(currBinPos < PLASTIC_DOOR_H && currBinPos >= PLASTIC_DOOR_L){
			ret_val = 1;
		}
		break;
	case BIN_METAL:
		if(currBinPos < METAL_DOOR_H && currBinPos >= METAL_DOOR_L){
			ret_val = 1;
		}
		break;
	case BIN_OTHER:
		if(currBinPos < OTHER_DOOR_H && currBinPos >= OTHER_DOOR_L){
			ret_val = 1;
		}
		break;
	default:
		issue_warning(WARN_ILLEGAL_BIN_REQUEST_SM_STATE);
	}
	return ret_val;
}

uint8_t get_bin_motor_dir(bin_t bin_requested){
	volatile uint16_t currBinPos = get_curr_bin_pos();
	uint8_t ret_val = 0;
	switch(bin_requested){
	case BIN_GLASS:
		if(currBinPos < GLASS_HALF_THRESH_H && currBinPos >= GLASS_HALF_THRESH_L){
			ret_val = 1;
		} else {
			ret_val = 0;
		}
		break;
	case BIN_PLASTIC:
		if(currBinPos < PLASTIC_HALF_THRESH_H && currBinPos >= PLASTIC_HALF_THRESH_L){
			ret_val = 1;
		} else {
			ret_val = 0;
		}
		break;
	case BIN_METAL:
		if(currBinPos < METAL_HALF_THRESH_H && currBinPos >= METAL_HALF_THRESH_L){
			ret_val = 1;
		} else {
			ret_val = 0;
		}
		break;
	case BIN_OTHER:
		if(currBinPos < OTHER_HALF_THRESH_H && currBinPos >= OTHER_HALF_THRESH_L){
			ret_val = 1;
		} else {
			ret_val = 0;
		}
		break;
	default:
		issue_warning(WARN_ILLEGAL_BIN_REQUEST_SM_STATE2);
	}
	return ret_val;
}

void bin_task(){
	static bin_t bin_int = BIN_GLASS;
	volatile uint16_t currBinPos = 0;
	switch(binCurrState){
	case BIN_IDLE_OFF:
		//State actions
		//Do nothing
		//State transistions
		if(sysState == SYS_ON){
				binCurrState = BIN_IDLE_ON;
		} else if (sysState == SYS_MAINT){
			binCurrState = BIN_FIND_NEAREST;
		} else {
			binCurrState = BIN_IDLE_OFF;
		}
		break;
	case BIN_FIND_NEAREST:
		//State actions
		bin_int = check_bin_nearest_pos();
		set_bin_direction(0);
		//State transitions
		binCurrState = BIN_CHECK_POS_MAINT;
		break;
	case BIN_CHECK_POS_MAINT:
		//State actions
		set_bin_speed(0);
		bin_motor_enable = 0;
		//State transistions
		if(check_bin_pos(bin_int)){
			binCurrState = BIN_WAIT_OPEN;
		} else {
			binCurrState = BIN_RUN_TO_DOOR;
		}
		break;
	case BIN_RUN_TO_DOOR:
		//State actions
		set_bin_speed(BIN_SPEED_SLOW);
		bin_motor_enable = 1;

		//State transistions
		if(check_bin_pos(bin_int)){
			binCurrState = BIN_WAIT_OPEN;
		} else {
			binCurrState = BIN_RUN_TO_DOOR;
		}
		break;
	case BIN_WAIT_OPEN:
		//State actions
		set_bin_speed(0);
		bin_motor_enable = 0;
		//State transitions
		if(sysState == SYS_OFF) {
			binCurrState = BIN_IDLE_OFF;
		} else if(is_bin_door_open()){
			binCurrState = BIN_WAIT_USER;
		} else {
			binCurrState = BIN_WAIT_OPEN;
		}
		break;
	case BIN_WAIT_USER:
		//State actions
		set_bin_speed(0);
		bin_motor_enable = 0;
		//State transistions
		if(!is_bin_door_open()){
			binCurrState = BIN_INC_INT;
		} else {
			binCurrState = BIN_WAIT_USER;
		}
		break;
	case BIN_INC_INT:
		//State actions
		switch(bin_int){
		case BIN_GLASS:
			bin_int = BIN_PLASTIC;
			break;
		case BIN_PLASTIC:
			bin_int = BIN_METAL;
			break;
		case BIN_METAL:
			bin_int = BIN_OTHER;
			break;
		case BIN_OTHER:
			bin_int = BIN_GLASS;
			break;
		default:
			issue_warning(WARN_ILLEGAL_BIN_REQUEST_SM_STATE2);
		}
		//State transistions
		if(sysState == SYS_OFF){
			binCurrState = BIN_IDLE_OFF;
		} else {
			binCurrState = BIN_CHECK_POS_MAINT;
		}
		break;
	case BIN_IDLE_ON:
		//State actions
		set_bin_speed(0);
		bin_motor_enable = 0;
		//State transistions
		if(bin_request){
			binCurrState = BIN_CHECK_POS;
		} else {
			binCurrState = BIN_IDLE_ON;
		}
		break;
	case BIN_CHECK_POS:
		//State actions
		//set_bin_speed(0);
		//bin_motor_enable = 0;
		bin_request = 0;
		//State transistions
		if(check_bin_request_pos(bin_int_request)){
			binCurrState = BIN_AT_POS;
		} else {
			binCurrState = BIN_RUN_TO_DUMP;
		}
		break;
	case BIN_RUN_TO_DUMP:
		//State actions
		//set_bin_speed(BIN_SPEED);
		set_bin_direction(get_bin_motor_dir(bin_int_request));
		//bin_motor_enable = 1;
		bin_run = 1;
		start_bin_pos = get_curr_bin_pos();
		total_distance = get_target_distance(start_bin_pos, bin_int_request);
		//State transistions
		if(check_bin_request_pos(bin_int_request)){
			binCurrState = BIN_AT_POS;

		} else {
			binCurrState = BIN_RUN_TO_DUMP;
		}
		break;
	case BIN_AT_POS:
		//State actions
		//set_bin_speed(0);
		//bin_motor_enable = 0;
		//bin_ready = 1;
		//State transistions
		binCurrState = BIN_IDLE_ON;
		break;
	default:
		issue_warning(WARN_ILLEGAL_BIN_SM_STATE);
		binCurrState = BIN_IDLE_OFF;
		break;
	}
	return;
}

void bin_acc_task(){
	static volatile uint16_t currBinPos;
	static volatile bin_t binTarget;
	static volatile uint16_t distance_left;
	static volatile uint16_t distance_traveled;
	switch(accCurrState){
	case ACC_IDLE:
		//State action
		set_bin_speed(0);
		bin_motor_enable = 0;
		binTarget = bin_int_request;
		distance_left = total_distance;
		distance_traveled = 0;
		//State transition
		if(bin_run) {
			accCurrState = ACC_RUN1;
			bin_run = 0;
		} else {
			accCurrState = ACC_IDLE;
		}
		break;
	case ACC_RUN1:
		//State action
		currBinPos = get_curr_bin_pos();
		distance_left = get_target_distance(currBinPos, binTarget);
		distance_traveled = total_distance - distance_left;
		set_bin_speed(BIN_SPEED_SLOW);
		bin_motor_enable = 1;
		//State transition
		if(distance_traveled > BIN_SPEED_THRESH && distance_left > BIN_SPEED_THRESH){
			accCurrState = ACC_RUN2;
		} else if (check_bin_request_pos(binTarget)){
			accCurrState = ACC_IDLE;
			bin_ready = 1;
		} else {
			accCurrState = ACC_RUN1;
		}
		break;
	case ACC_RUN2:
		//State action
		currBinPos = get_curr_bin_pos();
		distance_left = get_target_distance(currBinPos, bin_int_request);
		distance_traveled = total_distance - distance_left;
		set_bin_speed(BIN_SPEED_FAST);
		bin_motor_enable = 1;

		//State transition
		if(distance_left < BIN_SPEED_THRESH) {
			accCurrState = ACC_RUN1;
		} else {
			accCurrState = ACC_RUN2;
		}
		break;
	default:
		issue_warning(WARN_ILLEGAL_BIN_ACC_SM_STATE);
		accCurrState = ACC_IDLE;
		break;
	}
	return;
}
/*
 * TA0.4/P1.5 US_ECHO
 * P2.0 US_TRIG
 */

uint8_t check_us_timeroverflow(void){
	if(TA0CCTL4 & COV) {
		TA0CCTL4 &= ~COV;
		return 1;
	}
	return 0;
}

void bin_full_setup(void){
	P2DIR |= BIT0; // Init trigger
	P2OUT &= ~BIT0;
	P1SEL |= BIT5; // Timer
	TA0CTL =  TASSEL_2 | // SMCLK
			   ID_0 | //div/1
			   TACLR |
			   MC_2; // Mode continuously counts up
	TA0CCTL4 = CM1| // Capture on rising edge
			   CAP| // Capture mode
			   CCIE; // Interrupt
	return;
}

void bin_full_task(void){
	static volatile bin_t currBin = BIN_OTHER;

	switch(binFullCurrState){
	case BIN_FULL_IDLE:
		//State action
		//Do nothing
		//State transition
		if(us_request){
			binFullCurrState = BIN_FULL_START;
		} else {
			binFullCurrState = BIN_FULL_IDLE;
		}
		break;
	case BIN_FULL_START:
		//State action
		//Set trigger High
		P2OUT |= BIT0;
		//Clear COV and reset timer
		TA0CCTL4 &= ~(COV|CCIFG);
		TA0CTL |= TACLR;
		//State transistion
		binFullCurrState = BIN_FULL_WAIT;
		P2OUT &= ~BIT0;
		break;
	case BIN_FULL_WAIT:
		//State action
		//Do nothings
		//State transition
		if(check_us_timeroverflow() || us_timer_done) {
			us_timer_done = 0;
			binFullCurrState = BIN_FULL_END;
		} else {
			binFullCurrState = BIN_FULL_WAIT;
		}
		break;
	case BIN_FULL_END:
		//State action
		currBin = check_bin_nearest_pos();
		//State transition
		if(distance < BIN_FULL_THRESH_L) {
			if (currBin == BIN_GLASS){
				total_count.glass_full = 1;
			} else if(currBin == BIN_PLASTIC){
				total_count.plastic_full = 1;
			} else if(currBin == BIN_METAL){
				total_count.metal_full = 1;
			} else if(currBin == BIN_OTHER){
				total_count.other_full = 1;
			}
		} else if(distance > BIN_FULL_THRESH_H){
			if (currBin == BIN_GLASS){
				total_count.glass_full = 0;
			} else if(currBin == BIN_PLASTIC){
				total_count.plastic_full = 0;
			} else if(currBin == BIN_METAL){
				total_count.metal_full = 0;
			} else if(currBin == BIN_OTHER){
				total_count.other_full = 0;
			}
		}
		binFullCurrState = BIN_FULL_IDLE;
		break;
	default:
		issue_warning(WARN_ILLEGAL_BIN_FULL_SM_STATE);
		binFullCurrState = BIN_FULL_IDLE;
		break;
	}
	return;
}
/** END Bins Task Function **/

/** Compacting Task Function **/

uint16_t get_curr_compact_pos(void){
	volatile uint16_t currCompactPos = (adc_output_buffer[4] <<4 );
	return currCompactPos; // Hall pos 3
}

/* Check is compact open or closed */
uint8_t check_compact_pos(void){
	//TODO: Fix illegal and not illegal if checks
	volatile uint16_t currCompactPos = get_curr_compact_pos();
	uint8_t ret_val = 0;
	if(currCompactPos < COMPACT_OPEN_H && currCompactPos >= COMPACT_OPEN_L){
		ret_val = COMPACT_OPEN;
	} else if (currCompactPos < COMPACT_CLOSE_H && currCompactPos >= COMPACT_CLOSE_L) {
		ret_val = COMPACT_CLOSE;
	} else if (currCompactPos < COMPACT_CLOSE_L && currCompactPos >= COMPACT_OPEN_H) {
		ret_val = COMPACT_NOT_ILLEGAL;
	} else {
		ret_val = COMPACT_ILLEGAL;
	}

	return ret_val;
}

uint8_t check_compact_timeout(void){
	if(TA2CCTL0 & CCIFG){	//Check interrupt flag
		TA2CCTL0 &= ~CCIFG;	//Clear flag
		return 1;
	}
	return 0;
}

void reset_compact_timer(void){
	TA2CTL |= TACLR;
	TA2CCTL0 &= ~CCIFG;
	return;
}

void compact_setup(void){
	UCSCTL5 |= DIVA_5;
	TA2CTL = TASSEL_1 + ID_3 + MC_1; //SMCLK div 8 Up mode
	TA2EX0 = TAIDEX_7; // Divide by 8
	TA2CCR0 = 12207;	//1 sec
	return;
}

void compact_task(){
	volatile uint16_t currCompactArea = 0;
	switch(compactCurrState){
	case COMPACT_IDLE_OFF:
		//State action
		//Do nothing
		//State transistions
		if(sysState == SYS_ON){
			compactCurrState = COMPACT_IDLE_ON;
		} else {
			compactCurrState = COMPACT_IDLE_OFF;
		}
		break;
	case COMPACT_IDLE_ON:
		//State action
		set_compact_speed(0);
		compact_motor_enable = 0;
		//State transistions
		compactCurrState = COMPACT_CHECK_POS;
		break;
	case COMPACT_CHECK_POS:
		//State action

		currCompactArea = check_compact_pos();
		compact_motor_enable = 0;
		//State transistions
		if(currCompactArea == COMPACT_OPEN){
			compactCurrState = COMPACT_AT_OPEN;
		} else if (currCompactArea == COMPACT_CLOSE || currCompactArea == COMPACT_NOT_ILLEGAL){
			compactCurrState = OPEN_COMPACT;
		} else {
			issue_warning(WARN_ILLEGAL_COMPACT_POS);
			compactCurrState = COMPACT_ERR;
		}
		break;
	case OPEN_COMPACT:
		//State action
		set_compact_direction(MOT_CCW);
		set_compact_speed(COMPACT_SPEED);
		compact_motor_enable = 1;
		//State transistions
		if(currCompactArea == COMPACT_OPEN){
			compactCurrState = COMPACT_AT_OPEN;
		} else {
			compactCurrState = OPEN_COMPACT;
		}
		break;
	case COMPACT_AT_OPEN:
		//State action
		set_compact_speed(0);
		compact_motor_enable = 0;
		//State transistions
		if(sysState == SYS_OFF) {
			compactCurrState = COMPACT_IDLE_OFF;
		} else if(compact_ok){
			compactCurrState = COMPACT_WAIT;
		} else {
			compactCurrState = COMPACT_AT_OPEN;
		}
		break;
	case COMPACT_WAIT:
		//State action
		set_compact_speed(0);
		compact_motor_enable = 0;
		//State transistions
		if(compact_ok && step_close) {
			reset_compact_timer();
			compactCurrState = CLOSE_COMPACT;
		} else {
			compactCurrState = COMPACT_WAIT;
		}
		break;
	case CLOSE_COMPACT:
		//State action
		set_compact_direction(MOT_CW);
		set_compact_speed(COMPACT_SPEED);
		compact_motor_enable = 1;
		//State transistions
		if(currCompactArea == COMPACT_CLOSE || check_compact_timeout()){
			compactCurrState = COMPACT_AT_CLOSE;
		} else if (currCompactArea == COMPACT_OPEN || currCompactArea == COMPACT_NOT_ILLEGAL) {
			compactCurrState = CLOSE_COMPACT;
		} else {
			issue_warning(WARN_ILLEGAL_COMPACT_POS);
			compactCurrState = COMPACT_ERR;
		}
		break;
	case COMPACT_AT_CLOSE:
		//State action
		set_compact_speed(0);
		compact_motor_enable = 0;
		compact_done = 1;
		//State transistions
		compactCurrState = COMPACT_CHECK_POS;
		break;
	case COMPACT_ERR:
		//State action
		set_compact_speed(0);
		compact_motor_enable = 0;
		//WAIT HERE
		//No State transitions
		break;
	default:
		issue_warning(WARN_ILLEGAL_COMPACT_SM_STATE);
		compactCurrState = COMPACT_IDLE_OFF;
	}
	return;
}
/** END Compacting Task Function **/

/** Stepper Task Function **/

/* Setup stepper motor driver */
void stepper_setup(void){
	//Assign stepper pins
	P3DIR |= BIT7;		//Step output
	P4DIR |= BIT3;		//Direction output
	//Setup TA1.0 to run, pulse pin in interrupt
	//Up mode, Set/Reset 50% duty
	TA1CCR0 = STEP_TIME;
	TA1CTL = TASSEL_2 + MC_0 + TACLR + ID_3;	//SMCLK, off, clear TAR, div8
	TA1EX0 = TAIDEX_7;
	TA1CCTL0 |= CCIE;
	return;
}

inline void start_pwm(void){
	TA1CTL |= MC_1+TACLR;	//Up mode
	return;
}

inline void stop_pwm(void){
	TA0CTL &= ~MC_3; //Off, clear ouput pin
	P3OUT &= ~BIT7;
}

uint8_t is_step_switch_pressed(void){
	return (P2IN & BIT7);
}

/* STEPPER_STEP - USEL1
 * DIR - USEL2
 * EN - SERVO_OUTPUT ENABLE
 */
void stepper_enable(uint8_t direction){
	if(direction){
		P4OUT |= BIT3;
	} else {
		P4OUT &= ~BIT3;
	}
	//Enable
	sort_motor_enable = 1;
	return;
}
void stepper_disable(void){
	sort_motor_enable = 0;
	return;
}

void step_task(void){
	switch(stepCurrState){
	case STEP_IDLE_OFF:
		//State action
		stop_pwm();
		stepper_disable();
		//State transition
		if(sysState == SYS_ON){
			stepCurrState = STEP_FIND_HOME;
		} else {
			stepCurrState = STEP_IDLE_OFF;
		}
		break;
	case STEP_FIND_HOME:
		//State action
		stepper_enable(STEP_DIR_OPEN);
		start_pwm();
		//State transition
		if(!(sysState == SYS_ON)) {
			stepCurrState = STEP_IDLE_OFF;
		} else {
			stepCurrState = STEP_WAIT_HOME;
		}
		break;
	case STEP_WAIT_HOME:
		//State action
		step_position = 0;
		us_request = 1;
		//State transition
		if(!(sysState == SYS_ON)) {
			stepCurrState = STEP_IDLE_OFF;
		} else if(is_step_switch_pressed()) {
			stepCurrState = STEP_STOP;
		} else {
			stepCurrState = STEP_WAIT_HOME;
		}
		break;
	case STEP_STOP:
		//State action
		stop_pwm();
		stepper_disable();
		step_close = 0;
		//State transition
		if(!(sysState == SYS_ON)) {
			stepCurrState = STEP_IDLE_OFF;
		} else {
			stepCurrState = STEP_WAIT_REQUEST;
		}
		break;
	case STEP_WAIT_REQUEST:
		//State action
		//No action
		//State transition
		if(!(sysState == SYS_ON)) {
			stepCurrState = STEP_IDLE_OFF;
		} else if (step_request){
			stepCurrState = STEP_MOVE;
		} else {
			stepCurrState = STEP_WAIT_REQUEST;
		}
		break;
	case STEP_MOVE:
		//State action
		//State transition
		if(!(sysState == SYS_ON)) {
			stepCurrState = STEP_IDLE_OFF;
		} else {
			stepCurrState = STEP_RUN;
		}
		break;
	case STEP_RUN:
		//State action
		//No action
		//State transition
		if(!(sysState == SYS_ON)) {
			stepCurrState = STEP_IDLE_OFF;
		} else if (step_position >= STEP_CLOSE_POS){
			stepCurrState = STEP_PAUSE;
		} else {
			stepCurrState = STEP_RUN;
		}
		break;
	case STEP_PAUSE:
		stop_pwm();
		stepper_disable();
		step_close = 1;
		if(!(sysState == SYS_ON)) {
			stepCurrState = STEP_IDLE_OFF;
		} else if (compact_done && bin_ready){
			stepCurrState = STEP_FIND_HOME;
			compact_done = 0;
		} else {
			stepCurrState = STEP_PAUSE;
		}
		break;
	default:
		issue_warning(WARN_ILLEGAL_STEP_SM_STATE);
		stepCurrState = STEP_IDLE_OFF;
	}
	return;
}
/** END Stepper Task Function **/


/** LED Task Functions **/
void led_setup(void){
	P1DIR |= BIT4;
	P2DIR |= BIT1;
	P1OUT &= ~BIT4;
	P2OUT &= ~BIT1;
}

/* P1.4 - LED_MAINT SWITCH
 * P2.1 - LED_ON SWITCH
 */
void led_task(void){
	if( P1IN & BIT1){ // MAINT
		P1OUT |= BIT4;
	} else {
		P1OUT &= ~BIT4;
	}
	if(P1IN & BIT2) { // ON
		P2OUT |= BIT1;
	} else {
		P2OUT &= ~BIT1;
	}
}
/** END LED Task Function **/
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
		} else if((strncmp(debug_cmd_buf,"opt",3)==0) && (debug_cmd_buf_ptr == 4)){
			//>opt<number>
			uint16_t value = 0;
			switch(debug_cmd_buf[4]){
			case '0':
				value = adc_output_buffer[11];
				break;
			case '1':
				value = adc_output_buffer[12];
				break;
			case '2':
				value = adc_output_buffer[13];
				break;
			case '3':
				value = adc_output_buffer[14];
				break;
			case '4':
				value = adc_output_buffer[15];
				break;
			case '5':
				value = adc_output_buffer[16];
				break;
			case '6':
				value = adc_output_buffer[17];
				break;
			case '7':
				value = adc_output_buffer[18];
				break;
			}
			response_buf[0] = '0';
			response_buf[1] = 'x';
			hex2ascii_int(value, &response_buf[2], &response_buf[3], &response_buf[4], &response_buf[5]);
			response_size = 6;
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"opt",3)==0) && (debug_cmd_buf_ptr == 3)){
			//>opt
			uint16_t sum = 0;
			sum += adc_output_buffer[11];
			sum += adc_output_buffer[12];
			sum += adc_output_buffer[13];
			sum += adc_output_buffer[14];
			//sum += adc_output_buffer[15];
			//sum += adc_output_buffer[16];
			//sum += adc_output_buffer[17];
			//sum += adc_output_buffer[18];
			response_buf[0] = '0';
			response_buf[1] = 'x';
			hex2ascii_int(sum, &response_buf[2], &response_buf[3], &response_buf[4], &response_buf[5]);
			response_size = 6;
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"optall",6)==0) && (debug_cmd_buf_ptr == 6)){
			//>optall
			uint8_t i = 0;
			for(i= 11; i <= 14; i++){
				response_buf[0] = '0';
				response_buf[1] = 'x';
				hex2ascii_int(adc_output_buffer[i], &response_buf[2], &response_buf[3], &response_buf[4], &response_buf[5]);
				response_buf[6] = 10;
				response_buf[7] = 13;
				response_size =8;
				dbg_uart_send_string(response_buf,response_size);
			}
		/*} else if((strncmp(debug_cmd_buf,"l",1)==0) && (debug_cmd_buf_ptr == 1)){
			//>ldcall
			// WARNING: DO NOT USE UNTIL SM CHECKS FOR SPI DATASTRUCTURE BUSY
			uint8_t i = 0;
			for(i = 0; i <= 2 ; i++){
				uint16_t prox_data = ldc_get_proximity(i);
				response_buf[0] = '0';
				response_buf[1] = 'x';
				hex2ascii_int(prox_data, &response_buf[2], &response_buf[3], &response_buf[4], &response_buf[5]);
				response_size = 6;
				dbg_uart_send_string(response_buf,response_size);
			}*/
		} else if((strncmp(debug_cmd_buf,"ls",2)==0) && (debug_cmd_buf_ptr == 2)){
			//>ldcrun
			ldc_run = 1;
		} else if((strncmp(debug_cmd_buf,"le",2)==0) && (debug_cmd_buf_ptr == 2)){
			//>ldcstop
			ldc_stop = 1;
		} else if((strncmp(debug_cmd_buf,"lg",2)==0) && (debug_cmd_buf_ptr == 2)){
			//>ldcget
			uint8_t i = 0;
			for(i = 0; i <= 2 ; i++){
				uint16_t prox_data = ldc_data_buf[i];
				response_buf[0] = '0';
				response_buf[1] = '0';
				hex2ascii_int(prox_data, &response_buf[2], &response_buf[3], &response_buf[4], &response_buf[5]);
				response_size = 6;
				dbg_uart_send_string(response_buf,response_size);
				dbg_uart_send_byte(13);		//CR
				dbg_uart_send_byte(10);		//Line feed
			}
		} else if((strncmp(debug_cmd_buf,"ms",2)==0) && (debug_cmd_buf_ptr == 2)){
			//>massrun
			mass_run = 1;
		} else if((strncmp(debug_cmd_buf,"me",2)==0) && (debug_cmd_buf_ptr == 2)){
			//>massstop
			mass_end = 1;
		} else if((strncmp(debug_cmd_buf,"mg",2)==0) && (debug_cmd_buf_ptr == 2)){
			//>massget
			uint8_t i = 0;
			for(i = 0; i < NUM_MASS_SENSORS ; i++){
				uint16_t mass_data = mass_data_buf[i][mass_buf_ptr];
				response_buf[0] = '0';
				response_buf[1] = '0';
				hex2ascii_int(mass_data, &response_buf[2], &response_buf[3], &response_buf[4], &response_buf[5]);
				response_size = 6;
				dbg_uart_send_string(response_buf,response_size);
				dbg_uart_send_byte(9);
			}
			response_buf[0] = '0';
			response_buf[1] = '0';
			hex2ascii_int(hist_delta_bin, &response_buf[2], &response_buf[3], &response_buf[4], &response_buf[5]);
			response_size = 6;
			dbg_uart_send_string(response_buf,response_size);
			dbg_uart_send_byte(13);		//CR
			dbg_uart_send_byte(10);		//Line feed
			if(hist_delta_bin == 0){
				dbg_uart_send_string("Other",5);
			} else if(hist_delta_bin >= 16) {
				dbg_uart_send_string("Glass/Heavy Other",17);
			} else{
				dbg_uart_send_string("Plastic/Other",13);
			}

		} else if((strncmp(debug_cmd_buf,"os",2)==0) && (debug_cmd_buf_ptr == 2)){
			//>optrun
			opt_run = 1;
		} else if((strncmp(debug_cmd_buf,"oe",2)==0) && (debug_cmd_buf_ptr == 2)){
			//>optstop
			opt_end = 1;

		} else if((strncmp(debug_cmd_buf,"og",2)==0) && (debug_cmd_buf_ptr == 2)){
			//>optget
#ifdef OPTVAL
			response_buf[0] = '0';
			response_buf[1] = '0';
			hex2ascii_int(opt_max_time, &response_buf[2], &response_buf[3], &response_buf[4], &response_buf[5]);
			response_size = 6;
			dbg_uart_send_string(response_buf,response_size);
			dbg_uart_send_byte(9);   		//Tab

			response_buf[0] = '0';
			response_buf[1] = '0';
			hex2ascii_int(opt_low_val, &response_buf[2], &response_buf[3], &response_buf[4], &response_buf[5]);
			response_size = 6;
			dbg_uart_send_string(response_buf,response_size);
			dbg_uart_send_byte(13);		//CR
			dbg_uart_send_byte(10);		//Line feed
			if(opt_max_time == 0){
				dbg_uart_send_string("Other",5);
			} else if(opt_max_time < 20) {
				dbg_uart_send_string("Plastic/Glass",13);
			} else{
				dbg_uart_send_string("Other",5);
			}
#endif
#ifdef OPTDUMP
			uint8_t i = 0;
			uint8_t j = 0;
			uint8_t resp_buf_ptr = 0;
			/*
			for(i = opt_buf_ptr; i < OPT_BUF_SIZE; i++){
				resp_buf_ptr = 0;
				for(j=0; j < NUM_OPT_SENSORS; j++){
					hex2ascii_int(opt_data_buf[j][i], &response_buf[resp_buf_ptr], &response_buf[resp_buf_ptr+1], &response_buf[resp_buf_ptr+2], &response_buf[resp_buf_ptr+3]);
					response_buf[resp_buf_ptr+4] = 9;
					resp_buf_ptr += 5;
				}
				response_buf[resp_buf_ptr] = 10;
				response_buf[resp_buf_ptr+1] = 13;
				dbg_uart_send_string(response_buf,resp_buf_ptr+2);
			}
			for(i = 0; i < opt_buf_ptr; i++){
				resp_buf_ptr = 0;
				for(j=0; j < NUM_OPT_SENSORS; j++){
					hex2ascii_int(opt_data_buf[j][i], &response_buf[resp_buf_ptr], &response_buf[resp_buf_ptr+1], &response_buf[resp_buf_ptr+2], &response_buf[resp_buf_ptr+3]);
					response_buf[resp_buf_ptr+4] = 9;
					resp_buf_ptr += 5;
				}
				response_buf[resp_buf_ptr] = 10;
				response_buf[resp_buf_ptr+1] = 13;
				dbg_uart_send_string(response_buf,resp_buf_ptr+2);
			}

			*/
			for(i = 0; i < NUM_OPT_SENSORS; i++){
				uint16_t opt_data = opt_data_buf[i][opt_buf_ptr];
				response_buf[0] = '0';
				response_buf[1] = '0';
				hex2ascii_int(opt_data, &response_buf[2], &response_buf[3], &response_buf[4], &response_buf[5]);
				response_size = 6;
				dbg_uart_send_string(response_buf,response_size);
				dbg_uart_send_byte(9);   		//Tab
				//dbg_uart_send_byte(13);		//CR
				//dbg_uart_send_byte(10);		//Line feed
			}
			dbg_uart_send_byte(10);
#endif
#ifdef OPTVAL
		} else if((strncmp(debug_cmd_buf,"result",6)==0) && (debug_cmd_buf_ptr == 6)){
			if(opt_max_time == 0 && hist_delta_bin == 0){
				dbg_uart_send_string("Other",5);
			} else if(opt_max_time < 20 && hist_delta_bin >= 16) {
				dbg_uart_send_string("Glass",5);
			} else if (hist_delta_bin >= 16) {
				dbg_uart_send_string("Glass",5);
			} else if(opt_max_time < 20) {
				dbg_uart_send_string("Plastic",7);
			} else{
				dbg_uart_send_string("Other",5);
			}
#endif
		} else if((strncmp(debug_cmd_buf,"m1 en",5)==0) && (debug_cmd_buf_ptr == 5)){
			chimney_motor_enable = 1;
		} else if((strncmp(debug_cmd_buf,"m1 dis",6)==0) && (debug_cmd_buf_ptr == 6)){
			chimney_motor_enable = 0;
		} else if((strncmp(debug_cmd_buf,"m2 en",5)==0) && (debug_cmd_buf_ptr == 5)){
			compact_motor_enable = 1;
		} else if((strncmp(debug_cmd_buf,"m2 dis",6)==0) && (debug_cmd_buf_ptr == 6)){
			compact_motor_enable = 0;
		} else if((strncmp(debug_cmd_buf,"m3 en",5)==0) && (debug_cmd_buf_ptr == 5)){
			bin_motor_enable = 1;
		} else if((strncmp(debug_cmd_buf,"m3 dis",6)==0) && (debug_cmd_buf_ptr == 6)){
			bin_motor_enable = 0;
		} else if((strncmp(debug_cmd_buf,"m4 en",5)==0) && (debug_cmd_buf_ptr == 5)){
			sort_motor_enable = 1;
		} else if((strncmp(debug_cmd_buf,"m4 dis",6)==0) && (debug_cmd_buf_ptr == 6)){
			stop_pwm();
			stepper_disable();
		} else if((strncmp(debug_cmd_buf,"m1 r",4)==0) && (debug_cmd_buf_ptr == 4)){
			set_chimney_direction(MOT_CW);
		} else if((strncmp(debug_cmd_buf,"m1 l",4)==0) && (debug_cmd_buf_ptr == 4)){
			set_chimney_direction(MOT_CCW);
		} else if((strncmp(debug_cmd_buf,"m2 r",4)==0) && (debug_cmd_buf_ptr == 4)){
			set_compact_direction(MOT_CW);
		} else if((strncmp(debug_cmd_buf,"m2 l",4)==0) && (debug_cmd_buf_ptr == 4)){
			set_compact_direction(MOT_CCW);
		} else if((strncmp(debug_cmd_buf,"m3 r",4)==0) && (debug_cmd_buf_ptr == 4)){
			set_bin_direction(MOT_CW);
		} else if((strncmp(debug_cmd_buf,"m3 l",4)==0) && (debug_cmd_buf_ptr == 4)){
			set_bin_direction(MOT_CCW);
		} else if((strncmp(debug_cmd_buf,"m1s",3)==0) && (debug_cmd_buf_ptr == 7)){
			//>m1s <4-hex char speed>
			//>m1s 9C3
			uint16_t speed;
			speed = ascii2hex_byte(debug_cmd_buf[5], debug_cmd_buf[6]);
			speed |= (ascii2hex_byte('0',debug_cmd_buf[4])<<8);
			set_chimney_speed(speed);
		} else if((strncmp(debug_cmd_buf,"m2s",3)==0) && (debug_cmd_buf_ptr == 7)){
			//>m2s <4-hex char speed>
			//>m2s 9C3
			uint16_t speed;
			speed = ascii2hex_byte(debug_cmd_buf[5], debug_cmd_buf[6]);
			speed |= (ascii2hex_byte('0',debug_cmd_buf[4])<<8);
			set_compact_speed(speed);
		} else if((strncmp(debug_cmd_buf,"m3s",3)==0) && (debug_cmd_buf_ptr == 7)){
			//>m3s <4-hex char speed>
			//>m3s 9C3
			uint16_t speed;
			speed = ascii2hex_byte(debug_cmd_buf[5], debug_cmd_buf[6]);
			speed |= (ascii2hex_byte('0',debug_cmd_buf[4])<<8);
			set_bin_speed(speed);
		} else if((strncmp(debug_cmd_buf,"m4o",3)==0) && (debug_cmd_buf_ptr == 3)){
			stepper_enable(STEP_DIR_OPEN);
			start_pwm();
		} else if((strncmp(debug_cmd_buf,"m4c",3)==0) && (debug_cmd_buf_ptr == 3)){
			stepper_enable(STEP_DIR_CLOSE);
			start_pwm();
		} else if((strncmp(debug_cmd_buf,"m1 pos",6)==0) && (debug_cmd_buf_ptr == 6)){
			uint16_t currChimPos = get_curr_chimney_pos();
			response_buf[0] = '0';
			response_buf[1] = 'x';
			hex2ascii_int(currChimPos, &response_buf[2], &response_buf[3], &response_buf[4], &response_buf[5]);
			response_size = 6;
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"m2 pos",6)==0) && (debug_cmd_buf_ptr == 6)){
			uint16_t currCompactPos = get_curr_compact_pos();
			response_buf[0] = '0';
			response_buf[1] = 'x';
			hex2ascii_int(currCompactPos, &response_buf[2], &response_buf[3], &response_buf[4], &response_buf[5]);
			response_size = 6;
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"m3 pos",6)==0) && (debug_cmd_buf_ptr == 6)){
				uint16_t currBinPos = get_curr_bin_pos();
				response_buf[0] = '0';
				response_buf[1] = 'x';
				hex2ascii_int(currBinPos, &response_buf[2], &response_buf[3], &response_buf[4], &response_buf[5]);
				response_size = 6;
				dbg_uart_send_string(response_buf,response_size);
		} else if ((strncmp(debug_cmd_buf,"bin",3)==0) && (debug_cmd_buf_ptr == 3)) {
			bin_ready = 1;
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
/* Timer A0 Interrupt Handler
 * Ultrasonic bin fullness sensing
 */
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TimerA0(void){
	static uint16_t up_counter;
	if(TA0CCTL0 & CCI){
		up_counter = TA0CCR0;
	} else {
		distance = TA0CCR0 - up_counter;
		us_timer_done = 1;
	}
	//Clear interrupt flag
	TA0CTL &= ~TAIFG;
}

/* Timer A1 Interrupt Handler
 * Stepper motor pulsing
 */
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TimerA1(void){
	P3OUT ^= BIT7;	//Toggle P3.7 Stepper step pin
	step_position++;

}


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
	 * 6: A10 (MCU temp)
	 * 7: A11 (3.3V sense)
	 * 3: A3 (Hall Position Encoder 1 - Chimney )
	 * 4: A4 (Hall Position Encoder 2 - Compact)
	 * 5: A5 (Hall Position Encoder 3 - Bins )
	 * 10: A12 (Hall Position Encoder 6) UNUSED: Reassigned to LDC_CS2
	 * 11-18: A8 (Optical bank, 8 channels) UNUSED
	 * 19-26: A13 (Hall Bank A, 8 channels)
	 * 27-34: A14 (Hall Bank B, 8 channels)
	 * 35-42: A15 (Hall Bank C, 8 channels) UNUSED
	 * Note: A9 is used as digital output
	 */
	if(adc_seq2){
		//adc_internal_buffer[11+adc_ext_mux_ptr] = ADC12MEM[0];	//Gather conversions
		adc_internal_buffer[19+adc_ext_mux_ptr] = ADC12MEM[0];
		adc_internal_buffer[27+adc_ext_mux_ptr] = ADC12MEM[1];
		//adc_internal_buffer[35+adc_ext_mux_ptr] = ADC12MEM[3];
		adc_ext_mux_ptr++;
		if(adc_ext_mux_ptr > 7){
			adc12_int_done_flag = 1;			//Done with conversions, return to sm
		} else {
			ADC12CTL0 &= ~ADC12SC;	//Clear SC bit
			ADC12CTL0 |= ADC12SC;	//Start conversion for next sequence
			set_hall_A_chnl(adc_ext_mux_ptr);
			set_hall_B_chnl(adc_ext_mux_ptr);
		}
	} else { //Store conversions into internal buffer
		adc_internal_buffer[0] = ADC12MEM[0];	//5Vsense
		adc_internal_buffer[1] = ADC12MEM[1];	//12Vsense
		adc_internal_buffer[2] = ADC12MEM[2];	//6VA sense
		adc_internal_buffer[3] = ADC12MEM[3];	//Hall pos 1 (Chimney encoder)
		adc_internal_buffer[4] = ADC12MEM[4];	//Hall pos 2 (Compact encoder)
		adc_internal_buffer[5] = ADC12MEM[5];	//Hall pos 3 (Bin encoder)
		adc_internal_buffer[6] = ADC12MEM[6];	//Temp
		adc_internal_buffer[7] = ADC12MEM[7];	//3.3V sense
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
	}
}

#pragma vector=USCI_A1_VECTOR
__interrupt void USCIA1_ISR(void){
	if((UCA1IE & UCRXIE) && (UCA1IFG & UCRXIFG)){				//SPI Rxbuf full interrupt
		MOTOR_SPI_data.rx_bytes[MOTOR_SPI_data.rx_ptr] = UCA1RXBUF;	//Get latest byte from HW
		MOTOR_SPI_data.rx_ptr++;									//Flag reset with buffer read
		if(MOTOR_SPI_data.rx_ptr >= MOTOR_SPI_data.num_bytes){		//Done reading data
			if(MOTOR_SPI_data.motor == 1){							//Disable CS and disable interrupt
				MOTOR1_SPI_CS_DEASSERT;
			} else if(MOTOR_SPI_data.motor == 2){
				MOTOR2_SPI_CS_DEASSERT;
			} else if(MOTOR_SPI_data.motor == 3){
				MOTOR3_SPI_CS_DEASSERT;
			} else {
				issue_warning(WARN_ILLEGAL_MOTOR_SPI_CS2);
			}
			MOTOR_SPI_RXINT_DISABLE;
			MOTOR_SPI_data.data_ready = 1;
		}

	} else if((UCA1IE & UCTXIE) && (UCA1IFG & UCTXIFG)){
		UCA1TXBUF = MOTOR_SPI_data.tx_bytes[MOTOR_SPI_data.tx_ptr];	//Load next byte into HW buffer
		MOTOR_SPI_data.tx_ptr++;								//Flag reset with buffer write
		if(MOTOR_SPI_data.tx_ptr >= MOTOR_SPI_data.num_bytes){		//Done transmitting data
			MOTOR_SPI_TXINT_DISABLE;							//Disable Tx interrupt
		}
	} else {
		issue_warning(WARN_USCIA1_INT_ILLEGAL_FLAG);
	}
}

/* LDC SPI USCIB0 Interrupt Handler
 * SPI Rx:
 * SPI Tx:
 */
#pragma vector=USCI_B0_VECTOR
__interrupt void USCIB0_ISR(void){
	if((UCB0IE & UCRXIE) && (UCB0IFG & UCRXIFG)){				//SPI Rxbuf full interrupt
		LDC_SPI_data.rx_bytes[LDC_SPI_data.rx_ptr] = UCB0RXBUF;	//Get latest byte from HW
		LDC_SPI_data.rx_ptr++;									//Flag reset with buffer read
		if(LDC_SPI_data.rx_ptr >= LDC_SPI_data.num_bytes){		//Done reading data
			if(LDC_SPI_data.brd == 0){							//Disable CS and disable interrupt
				LDC0_SPI_CS_DEASSERT;
			} else if(LDC_SPI_data.brd == 1){
				LDC1_SPI_CS_DEASSERT;
			} else if(LDC_SPI_data.brd == 2){
				LDC2_SPI_CS_DEASSERT;
			} else {
				issue_warning(WARN_ILLEGAL_LDC_SPI_CS2);
			}
			LDC_SPI_RXINT_DISABLE;
			LDC_SPI_data.data_ready = 1;
		}

	} else if((UCB0IE & UCTXIE) && (UCB0IFG & UCTXIFG)){
		UCB0TXBUF = LDC_SPI_data.tx_bytes[LDC_SPI_data.tx_ptr];	//Load next byte into HW buffer
		LDC_SPI_data.tx_ptr++;								//Flag reset with buffer write
		if(LDC_SPI_data.tx_ptr >= LDC_SPI_data.num_bytes){		//Done transmitting data
			LDC_SPI_TXINT_DISABLE;							//Disable Tx interrupt
		}
	} else {
		issue_warning(WARN_USCIB0_INT_ILLEGAL_FLAG);
	}
}

/* I2C USCIB1 Interrupt Handler
 *
 */
#pragma vector = USCI_B1_VECTOR
__interrupt void USCI_B1_ISR(void){
	switch(__even_in_range(UCB1IV,12)){
	case  0: 		                           // Vector  0: No interrupts
		break;
	case  2: 		                           // Vector  2: ALIFG
		UCB1CTL1 |= UCTXSTP;				//Generate stop
		issue_warning(WARN_I2C_ARB_LOST);
		break;
	case  4: 		                           // Vector  4: NACKIFG
		UCB1CTL1 |= UCTXSTP;				//Generate stop
		issue_warning(WARN_I2C_NACK);
		break;
	case  6: 		                           // Vector  6: STTIFG
		break;
	case  8: 		                           // Vector  8: STPIFG
		break;
	case 10: 		                           // Vector 10: RXIFG
		break;
	case 12:                                  // Vector 12: TXIFG
		if(I2C_data.tx_ptr >= I2C_data.num_bytes){	//Done transmitting data, send stop
			UCB1CTL1 |= UCTXSTP;
			I2C_TXINT_DISABLE;
			I2C_data.data_ready = 1;
		} else {
			UCB1TXBUF = I2C_data.tx_bytes[I2C_data.tx_ptr];
			I2C_data.tx_ptr++;
		}
	    break;
	  default:
		  issue_warning(WARN_I2C_ILLEGAL_INTVECTOR);
		  break;
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
