#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <spi_comm.h>
#include <chprintf.h>
#include <motors.h>
#include <button.h>
#include <audio/microphone.h>
#include <sensors/proximity.h>
#include <arm_math.h>

#include <process_image.h>
#include <pi_regulator.h>
#include "sensors/VL53L0X/VL53L0X.h"

/*messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);*/
static uint8_t play = STOP;

void SendUint8ToComputer(uint8_t* data, uint16_t size)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

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

//static void timer12_start(void){
//    //General Purpose Timer configuration
//    //timer 12 is a 16 bit timer so we can measure time
//    //to about 65ms with a 1Mhz counter
//    static const GPTConfig gpt12cfg = {
//        1000000,        /* 1MHz timer clock in order to measure uS.*/
//        NULL,           /* Timer callback.*/
//        0,
//        0
//    };
//
//    gptStart(&GPTD12, &gpt12cfg);
//    //let the timer count to max value
//    gptStartContinuous(&GPTD12, 0xFFFF);
//}
uint8_t get_game_state(void)
{
	return play;
}

void set_game_state(uint8_t state)
{
	play = state;
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
	//po8030_set_awb(0);
	//po8030_set_rgb_gain(0x40, 0xFF, 0x40);

	//starts the time of flight sensor
	VL53L0X_start();

	//initializes the motors
	motors_init();

	palClearPad(GPIOD, GPIOD_LED7);
	//starts the threads for the pi regulator and the processing of the image
	pi_regulator_start();
	process_image_start();

	bool play_flag = true;

    /* Infinite loop. */
    while (1) {
    	//chprintf((BaseSequentialStream *)&SDU1, "button state = %d \n", button_is_pressed());
    	if(button_is_pressed())
    	{
    		if(play_flag){
    			chThdSleepMilliseconds(500); //let the player start the robot
    			play = START;
    			play_flag = false;
    			chThdSleepMilliseconds(1000); //prevents from stoping the game too fast
    		}else{
    			play = STOP;
    			play_flag = true;
    			chThdSleepMilliseconds(1000); //prevents from restarting the game too fast
    		}
    		
    	}
    	chThdSleepMilliseconds(200);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
