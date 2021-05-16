#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <spi_comm.h>
#include <motors.h>
#include <button.h>
#include <leds.h>

#include <color_detection.h>
#include <robot_movement.h>
#include "sensors/VL53L0X/VL53L0X.h"
#include <audio/audio_thread.h>
#include "audio/play_melody.h"

static uint8_t play = STOP;

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

uint8_t get_game_state(void)
{
	return play;
}

void set_game_state(uint8_t state)
{
	play = state;
}

void end_melody(void)
{
	playMelody(SEVEN_NATION_ARMY, ML_SIMPLE_PLAY, NULL);
}

int main(void)
{
	halInit();
	chSysInit();
	mpu_init();

	 /** Inits the Inter Process Communication bus. */
	//messagebus_init(&bus, &bus_lock, &bus_condvar);

	//starts the serial communication
	serial_start();
	//start the USB communication
	usb_start();
	//starts the SPI communication
	spi_comm_start();
	//starts the camera
	dcmi_start();
	po8030_start();
	po8030_set_awb(0);
	//starts the speaker
	dac_start();
	playMelodyStart();

	//starts the time of flight sensor
	VL53L0X_start();

	//initializes the motors
	motors_init();

	//starts the threads for robot_movement and the processing of the image
	robot_movement_start();
	process_image_start();

    /* Infinite loop. */
    while (1) {
    	if(button_is_pressed())
    	{
    		if(!play){
    			chThdSleepMilliseconds(500); //let the player start the robot
    			play = START;
    			chThdSleepMilliseconds(1000); //prevents from stoping the game by releasing the button too late
    		}else{
    			play = STOP;
    			chThdSleepMilliseconds(1000); //prevents from restarting the game by releasing the button too late
    		}
    		
    	}
    	chThdSleepMilliseconds(100);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
