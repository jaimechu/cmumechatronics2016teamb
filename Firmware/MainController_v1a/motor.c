/* ldc_spi_uscib0.c
 * Created 4/8/15 by Nishant Pol and Jaime Chu
 * 18-578 Mechatronics
 * MSP430F5521
 * Main Controller Code
 *
 *
 * Motor Pinout
 * Uses Timer B
 * P7.4/TB0.2: M3_PWM (Bins)
 * P7.5/TB0.3: M2_PWM (Compacting)
 * P7.6/TB0.4: M1_PWM (Chimney)
 * P2.3 M1_DIR (Chimney) (previously PJ.3)
 * P2.5 M2_DIR (Compacting) (Previously PJ.1)
 * P4.7 M3_DIR (Bins)
 */

#include "motor.h"

void motor_pwm_setup(void){
	TB0CTL = TBSSEL_2 + MC_1 + TBCLR;//SMCLK div 8

	TB0CCTL2 = CLLD_0 + OUTMOD_6;
	TB0CCTL3 = CLLD_0 + OUTMOD_6;
	TB0CCTL4 = CLLD_0 + OUTMOD_6;
	TB0CCR0 = 2500;	//10kHz PWM frequency
	TB0CCR2 = 0;	//Initially 0% duty to disable
	TB0CCR3 = 0;
	TB0CCR4 = 0;

	//Setup PWM pins
	P7SEL |= BIT4 + BIT5 + BIT6;
	P7DIR |= BIT4 + BIT5 + BIT6;
	//Setup direction pins
	P2DIR |= BIT3 + BIT5;
	P4DIR |= BIT7;
	P2OUT &= ~(BIT3 + BIT5);
	P4OUT &= ~BIT7;
}

/* Set PWM value for Chimney motor (M1)
 * speed: value between 0 to 2499
 */
void set_chimney_speed(uint16_t speed){
	if(speed >= 2500){
		speed = 2499;
	}
	TB0CCR4 = speed;
	return;
}

/* Set PWM value for Compact motor (M2)
 * speed: value between 0 to 2499
 */
void set_compact_speed(uint16_t speed){
	if(speed >= 2500){
		speed = 2499;
	}
	TB0CCR3 = speed;
	return;
}

/* Set PWM value for Bin motor (M3)
 * speed: value between 0 to 2499
 */
void set_bin_speed(uint16_t speed){
	if(speed >= 2500){
		speed = 2499;
	}
	TB0CCR2 = speed;
	return;
}

/* Set Motor Direction
 * dir: 0 is CW, 1 is CCW
 */
void set_chimney_direction(uint8_t dir){
	if(dir){
		P2OUT |= BIT3;
	} else {
		P2OUT &= ~BIT3;
	}
}

/* Set Motor Direction
 * dir: 0 is CW, 1 is CCW
 */
void set_compact_direction(uint8_t dir){
	if(dir){
		P2OUT |= BIT5;
	} else {
		P2OUT &= ~BIT5;
	}
}

/* Set Motor Direction
 * dir: 0 is CW, 1 is CCW
 */
void set_bin_direction(uint8_t dir){
	if(dir){
		P4OUT |= BIT7;
	} else {
		P4OUT &= ~BIT7;
	}
}


