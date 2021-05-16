#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>


#include <main.h>
#include <motors.h>
#include <leds.h>
#include <robot_movement.h>
#include <color_detection.h>
#include "sensors/VL53L0X/VL53L0X.h"


static  uint16_t counter[NUM_COLORS] = {0};    //a counter for each color to make the decision of the robot more robust

/*
 * PI regulator implementation
 */
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

/*
 * Resets the color counters
 */
void reset_counter(uint8_t color)
{
	for (uint8_t i =0; i < NUM_COLORS; i++){
		if (i != color){
			counter[i] = 0;
		}
	}
}

/*
 * Makes the robot turn left
 */
void left_turn(void)
{
	right_motor_set_speed(BASESPEED);
	left_motor_set_speed(-BASESPEED);
	chThdSleepMilliseconds(1300*250/BASESPEED); //turns 90 degrees
}

/*
 * Makes the robot turn right
 */
void right_turn(void)
{
	right_motor_set_speed(-BASESPEED);
	left_motor_set_speed(BASESPEED);
	chThdSleepMilliseconds(1300*250/BASESPEED); //turns 90 degrees
}

/*
 * Corrects the trajectory of the robot to be centered with the middle of the line
 */
void robot_correction(void)
{
	uint16_t line_position = 0; int16_t speed_correction = 0;
	if(get_line_width(BLUE)!=0){
		line_position = get_line_position(BLUE);  //for red/yellow/black detection
	}else{
		line_position = get_line_position(RED);	  //for blue/white detection
	}
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

/*
 * Stops the robot
 */
void robot_stop(void)
{
	right_motor_set_speed(0);
	left_motor_set_speed(0);
	set_game_state(STOP);
}

/*
 * Makes the robot go straight for about 55mm
 */
void robot_straight(void)
{
	right_motor_set_speed(BASESPEED);
	left_motor_set_speed(BASESPEED);
	chThdSleepMilliseconds(1750*250/BASESPEED);
}

/*
 * The robot pauses, letting the player think for his next move
 */
void robot_pause(void)
{
	right_motor_set_speed(0);
	left_motor_set_speed(0);
	chThdSleepMilliseconds(5000);	//gives 5 seconds to the player 
}

/*
 * Switches the players turn after an action has been made 
 * by the robot and sets the rgb leds to let the players know
 */
uint8_t update_turn(uint8_t player_turn, uint8_t nb_players)
{
	uint8_t turn = player_turn;
	if((turn+1)<nb_players){ 
		turn++; 
	}else{ 
		turn = 0; 
	}

	if(turn==0){
		//purple (player one)
		set_rgb_led(LED2, RGB_MAX_INTENSITY/2, 0, RGB_MAX_INTENSITY/4);
		set_rgb_led(LED4, RGB_MAX_INTENSITY/2, 0, RGB_MAX_INTENSITY/4);
		set_rgb_led(LED6, RGB_MAX_INTENSITY/2, 0, RGB_MAX_INTENSITY/4);
		set_rgb_led(LED8, RGB_MAX_INTENSITY/2, 0, RGB_MAX_INTENSITY/4);
	}else if(turn==1){
		//orange (player two)
		set_rgb_led(LED2, RGB_MAX_INTENSITY, 30, 0);
		set_rgb_led(LED4, RGB_MAX_INTENSITY, 30, 0);
		set_rgb_led(LED6, RGB_MAX_INTENSITY, 30, 0);
		set_rgb_led(LED8, RGB_MAX_INTENSITY, 30, 0);
	}

	return turn;
}

/*
 * Shows the current score of the player who just scored a point.
 * Flashes the body LEDs by the number of points the player has.
 */
void show_score(uint8_t * score, uint8_t player){
	if(score[player]==1){
		set_body_led(1);
	}else if(score[player]==2){
		set_body_led(1);
		chThdSleepMilliseconds(800);
		set_body_led(0);
		chThdSleepMilliseconds(800);
		set_body_led(1);
	}
	chThdSleepMilliseconds(900);
	set_body_led(0);
}

static THD_WORKING_AREA(waRobotMovement, 400);
static THD_FUNCTION(RobotMovement, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    uint8_t black_width_c = 0; 		//black width counter for detecting the black squares
    uint8_t ToF_counter = 0;		//the ToF can be inconsistent. A counter is neede to make robust decisions
    uint8_t current_color = WHITE;
    uint8_t game_end = false;

    uint8_t nb_players = 2;			//number of players
    uint8_t player_turn = 1; 		//0 is for player one, 1 is for player two
    player_turn = update_turn(player_turn, nb_players);  //initializes the rgb LEDs to purple(1st player)
    uint8_t scores[nb_players];
    memset(scores, 0, nb_players*sizeof(uint8_t));

    while(1){
    	if(get_game_state() && !game_end)
    	{
			//wait until a new color is available
	        wait_color_computed();
	        current_color = get_color();

	        //check for obstacle
	        if(VL53L0X_get_dist_mm()<100)
	        {
				ToF_counter++;
	        }else{
	        	ToF_counter = 0;
	        }

	        //different scenarios in the game board
	        if(ToF_counter>3)
	        {
	        	ToF_counter = 0;
	        	right_turn();
	        	right_turn();	//2*right turns = 180 degree turn
	        	player_turn = update_turn(player_turn, nb_players);
				robot_pause();
	        }else if(current_color==BLACK) //if the line its seeing is black the robot will just go forward. 
	        							   //If the width is 12mm its a black square and the robot will act
	        							   //accordingly.
			{
				robot_correction();

				if(counter[BLACK]>1){reset_counter(BLACK);}
				counter[BLACK]++;

				//the next lines of code detects if there is a black square
				if(get_line_width(BLUE)>BLACK_SQUARE_LOW && get_line_width(BLUE)<BLACK_SQUARE_HIGH){
					black_width_c++;
				}else{
					black_width_c = 0;
				}
				if(black_width_c>2){
					robot_straight();
					reset_counter(NUM_COLORS);
					black_width_c = 0;
					player_turn = update_turn(player_turn, nb_players);
					robot_pause();
				}
			}
			else if(current_color==RED)//if the color detected is RED it means the robot is on a RED square.
									   //this part will tell the robot to turn left once he reached the square
			{
				robot_correction();
			
				if(counter[RED]>1){reset_counter(RED);} //when the camera sees RED three times in row 	
														//it resets the counters for the other colours
				counter[RED]++;
				
				if(counter[RED]>2) //ensures the camera is indeed looking at a red square
				{
					robot_straight(); 	//cancel the speed correction. 
										//the robot needs to go straight forward before a turn
					left_turn();
					reset_counter(NUM_COLORS); //resets all counters
					player_turn = update_turn(player_turn, nb_players);
					robot_pause();
				}
			}
			else if(current_color==BLUE)	//if the color detected is BLUE it means the robot is on a BLUE square.
									  		//this part will tell the robot to turn right once he reached the square
			{
				robot_correction();
				
				if(counter[BLUE]>1){reset_counter(BLUE);}
				counter[BLUE]++;
	
				if(counter[BLUE]>2) //ensures the camera is indeed looking at a blue square
				{
					robot_straight(); 	//cancel the speed correction. 
										//the robot needs to go straight forward before a turn
					right_turn();
					reset_counter(NUM_COLORS); //resets all counters
					player_turn = update_turn(player_turn, nb_players);
					robot_pause();
				}
			}
			else if(current_color==YELLOW)	//if the color detected is YELLOW it means the robot is on a YELLOW square.
									  		//this part will tell the robot to give a point to the player who reached the
											//the square. It will also detect if the final score has been reached by this
											//player and will end the game if so.
			{
				robot_correction();

				if(counter[YELLOW]>1){reset_counter(YELLOW);}
				counter[YELLOW]++;

				if(counter[YELLOW]>2) //ensures the camera is indeed looking at a yellow square
				{
					robot_straight(); 	//cancel the speed correction. 
										//the robot needs to go straight forward before a turn
					reset_counter(NUM_COLORS); //resets all counters
					scores[player_turn]++;
					robot_stop();
					show_score(scores, player_turn);
					if(scores[player_turn] == FINAL_SCORE){
						end_melody();
						game_end = true; //the game ends here
					}else{
						player_turn = update_turn(player_turn, nb_players);
					}
				}
			}
			else if(current_color==WHITE) //when the robot sees white it either means it reached the and of a path or
										  //is going out of track. In the second case this part will stop the robot after
										  //a few steps in a white area
			{
				if(counter[WHITE]>1){reset_counter(WHITE);} //when the camera sees white three times in row 	
															//it resets the counters for the other colours
				if(counter[WHITE]<10){ 
					robot_correction();
					counter[WHITE]++;
				}
				else{ 
					robot_stop(); //when the robot stays too long on white it stops
				}
			} 
	    }
    }
}

void robot_movement_start(void){
	chThdCreateStatic(waRobotMovement, sizeof(waRobotMovement), NORMALPRIO, RobotMovement, NULL);
}
