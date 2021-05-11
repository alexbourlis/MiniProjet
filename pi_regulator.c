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

static uint8_t color_now = 0;
static uint8_t offset_correction = 0;


//simple PI regulator implementation
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

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed_correction = 0;

    while(1){
        time = chVTGetSystemTime();
        
        //computes a correction factor to keep the robot aligned with the black line
        speed_correction = pi_regulator(get_line_position(), IMAGE_BUFFER_SIZE/2);

        //if the robot is nearly aligned with the black line, there is no need to correct the trajectory
        if(abs(speed_correction) < CORRECTION_THRESHOLD)
        {
        	speed_correction = 0;
        }


        if(get_color_pid()==RED && offset_correction==0)
        {
        	color_now=RED;
        	offset_correction=1;
        }
        else if (get_color_pid()==BLACK && offset_correction==0)
        {
        	color_now=BLACK;
        }
        else if(get_color_pid()==YELLOW && offset_correction==0)
        {
        	color_now=YELLOW;
        }
        else if(get_color_pid()==BLUE && offset_correction==0)
        {
        	color_now=BLUE;
        	offset_correction=1;
        }
        //different scenarios in the game board
		if((color_now==BLACK || color_now==YELLOW)  && VL53L0X_get_dist_mm()>100)
		{
			//continue going forward and trying to correct the alignment with the black line
			right_motor_set_speed(BASESPEED - speed_correction);
			left_motor_set_speed(BASESPEED + speed_correction);
		}
		else if(color_now==RED && VL53L0X_get_dist_mm()>100)
		{
			if(offset_correction==1)
			{
			right_motor_set_speed(BASESPEED);
			left_motor_set_speed(BASESPEED);
			offset_correction=2;
			chThdSleepUntilWindowed(time, time + MS2ST(900));
			}
			else if(offset_correction==2)
			{
				right_motor_set_speed(BASESPEED);
				left_motor_set_speed(-BASESPEED);
				offset_correction=0;
				chThdSleepUntilWindowed(time, time + MS2ST(600));
			}

			//turn left
			//we need to adjust the values for the speeds to make a left turn that suits the board setup!
			//right_motor_set_speed(BASESPEED);
			//left_motor_set_speed(BASESPEED/2);
			//we need to buy time so we make the thread sleep for X amount of time, needs to be tested
		}
		else if(color_now==BLUE && VL53L0X_get_dist_mm()>100)
		{
			if(offset_correction==1)
			{
			right_motor_set_speed(BASESPEED);
			left_motor_set_speed(BASESPEED);
			offset_correction=2;
			chThdSleepUntilWindowed(time, time + MS2ST(900));
			}
			else if(offset_correction==2)
			{
				right_motor_set_speed(-BASESPEED);
				left_motor_set_speed(BASESPEED);
				offset_correction=0;
				chThdSleepUntilWindowed(time, time + MS2ST(600));
			}

					//turn left
					//we need to adjust the values for the speeds to make a left turn that suits the board setup!
					//right_motor_set_speed(BASESPEED);
					//left_motor_set_speed(BASESPEED/2);
					//we need to buy time so we make the thread sleep for X amount of time, needs to be tested
		}

		else
		{
			//useless code just for testing
			right_motor_set_speed(0);
			left_motor_set_speed(0);
		}

		/*else if(VL53L0X_get_dist_mm()<100)
				{
					right_motor_set_speed(BASESPEED);
					left_motor_set_speed(-BASESPEED);
					chThdSleepUntilWindowed(time, time + MS2ST(1300));

				}*/
		//USEFUL CODE JUST NEED TO GET THE BOARD READY
		/*else if(get_color_pid()==RED && VL53L0X_get_dist_mm()>100)
		{
			//turn left
			//we need to adjust the values for the speeds to make a left turn that suits the board setup!
			//right_motor_set_speed(BASESPEED);
			//left_motor_set_speed(BASESPEED/2);
			//we need to buy time so we make the thread sleep for X amount of time, needs to be tested
			//chThdSleepUntilWindowed(time, time + MS2ST(X))
		}else if(get_color_pid()==BLUE && VL53L0X_get_dist_mm()>100)
		{
			//turn right
			//we need to adjust the values for the speeds to make a left turn that suits the board setup!
			//right_motor_set_speed(BASESPEED/2);
			//left_motor_set_speed(BASESPEED);
			//we need to buy time so we make the thread sleep for X amount of time to complete the turn
			//chThdSleepUntilWindowed(time, time + MS2ST(X))
		}else if(VL53L0X_get_dist_mm()<100)
		{

			//perform a u-turn
			//right_motor_set_speed(-BASESPEED);
			//left_motor_set_speed(BASESPEED);
			//we need to buy time so we make the thread sleep for X amount of time to complete the turn
			//chThdSleepUntilWindowed(time, time + MS2ST(X))
		}*/

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
