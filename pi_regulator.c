#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>
#include "sensors/VL53L0X/VL53L0X.h"

//simple PI regulator implementation
static  uint16_t counter[NUM_COLORS] = {0};

int16_t pi_regulator(float line_position, float goal){

	float error = 0;
	float speed = 0;

	static float sum_error = 0;

	error = (line_position - goal)/150; //converted the error in cm

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error+ KI * sum_error;

    return (int16_t)speed;
}

void reset_counter(uint8_t color)
{
	//uint16_t list[NUM_COLORS] = {0};
	for (uint8_t i =0; i < NUM_COLORS; i++){
		if (i != color){
			counter[i] = 0;
		}
	}
}

void left_turn(void)
{
	right_motor_set_speed(BASESPEED/2);
	left_motor_set_speed(-BASESPEED/2);
}

void right_turn(void)
{
	right_motor_set_speed(-BASESPEED/2);
	left_motor_set_speed(BASESPEED/2);
}

void motor_correction(void)
{
	uint16_t line_position = 0; int16_t speed_correction = 0;

	line_position = get_line_position();
    //chprintf((BaseSequentialStream *)&SD3, "blue line position = %d\n", line_position);
	//computes a correction factor to keep the robot aligned with the black line
	speed_correction = pi_regulator(line_position, IMAGE_BUFFER_SIZE/2 - EMPIRICAL_CORRECTION);
	  //if the robot is nearly aligned with the black line, there is no need to correct the trajectory
    if(abs(speed_correction) < CORRECTION_THRESHOLD)
    {
    	speed_correction = 0;
    }
	//continue going forward and trying to correct the alignment with the black line
	right_motor_set_speed(BASESPEED - speed_correction);
	left_motor_set_speed(BASESPEED + speed_correction);
}

void motor_stop(void)
{
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}

void motor_straight(void)
{
	right_motor_set_speed(BASESPEED);
	left_motor_set_speed(BASESPEED);
}
static THD_WORKING_AREA(waPiRegulator, 300);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    uint8_t current_color = WHITE;

    while(1){
    	time = chVTGetSystemTime();
    	if(get_game_state())
    	{
			current_color = get_color();
			
	        //different scenarios in the game board
	        if(current_color != OLD_COLOR)
	        {
				if(current_color==BLACK)// || get_color_pid()==YELLOW) && VL53L0X_get_dist_mm()>100)
				{
					motor_correction();
					//TEST
					//chprintf((BaseSequentialStream *)&SD3, "color PID BLACK\t");
					if(counter[BLACK]>1){reset_counter(BLACK);}
					counter[BLACK]++;
					//chprintf((BaseSequentialStream *)&SD3, "blaCK counter = %d\n", counter[BLACK]);
					if(counter[BLACK]>2)
					{
						//chprintf((BaseSequentialStream *)&SD3, "color PID = %d\n", BLACK);
					}
				}
				else if(current_color==RED)
				{
					motor_correction();
					//TEST
					//chprintf((BaseSequentialStream *)&SD3, "color PID RED\t");
					if(counter[RED]>1){reset_counter(RED);}
					counter[RED]++;
					//chprintf((BaseSequentialStream *)&SD3, "rED counter = %d\n", counter[RED]);
					if(counter[RED]>2) //ensures the camera is indeed looking at a red spot
					{
						//chprintf((BaseSequentialStream *)&SD3, "color PID = %d\n", RED);
						motor_straight(); 	//cancel the speed correction. 
											//the robot needs to go straight forward before a turn
						chThdSleepMilliseconds(1950*250/BASESPEED);
						left_turn();
						chThdSleepMilliseconds(1300*500/BASESPEED); //turns 90 degrees
						//while(abs(get_line_position()-(IMAGE_BUFFER_SIZE/2 - EMPIRICAL_CORRECTION))>ALIGNEMENT_THRESHOLD)
						reset_counter(NUM_COLORS); //resets all counters
					}
				}
				else if(current_color==BLUE)
				{
					motor_correction();
					//TEST
					//chprintf((BaseSequentialStream *)&SD3, "color PID RED\t");
					if(counter[BLUE]>1){reset_counter(BLUE);}
					counter[BLUE]++;
					//chprintf((BaseSequentialStream *)&SD3, "rED counter = %d\n", counter[RED]);
					if(counter[BLUE]>2) //ensures the camera is indeed looking at a red spot
					{
						//chprintf((BaseSequentialStream *)&SD3, "color PID = %d\n", BLUE);
						motor_straight(); 	//cancel the speed correction. 
											//the robot needs to go straight forward before a turn
						chThdSleepMilliseconds(1950*250/BASESPEED);
						right_turn();
						chThdSleepMilliseconds(1300*500/BASESPEED);
						reset_counter(NUM_COLORS); //resets all counters
					}
				}
				else if(current_color==WHITE)
				{
					if(counter[WHITE]>1){reset_counter(WHITE);} //when the camera sees white three times in row 	
																//it resets the counters for the other colours
					
					if(counter[WHITE]<30){ 
						motor_correction();
						counter[WHITE]++;
					}
					else{ 
						set_game_state(STOP); //when the robot stays too long on white it stops
					}
				} 
				//VL53L0X_get_dist_mm()>100
			}
			//chprintf((BaseSequentialStream *)&SDU1, "capture time = %d\n", chVTGetSystemTime()-time);
	    }
	    else
	    {
	    	motor_stop();
	    }
	    chThdSleepUntilWindowed(time, time + MS2ST(10)); //probably need a semaphore instead
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
