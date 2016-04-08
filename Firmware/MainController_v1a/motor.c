/* ldc_spi_uscib0.c
 * Created 4/8/15 by Nishant Pol and Jaime Chu
 * 18-578 Mechatronics
 * MSP430F5521
 * Main Controller Code
 *
 *
 * Motor Pinout
 * Uses Timer B
 * P7.4/TB0.2: M3_PWM
 * P7.5/TB0.3: M2_PWM
 * P7.6/TB0.4: M1_PWM
 */

#include "motor.h"

void motor_pwm_setup(void){
	TB0CTL = TBSSEL_2 + MC_1 + TBCLR;//SMCLK div 8
	//TODO: Check CLLD selection
	TB0CCTL2 = CLLD_0 + OUTMOD_2;
	TB0CCR0 = 2500;	//10kHz PWM frequency
	TB0CCR2 = 0;	//Initially 0% duty to disable
	TB0CCR3 = 0;
	TB0CCR4 = 0;

}

void set_chimney_motor_speed(uint){

}




