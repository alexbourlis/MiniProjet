#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>


//static float distance_cm = 0;
static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle
static uint16_t line_position_r = IMAGE_BUFFER_SIZE/2;
static uint16_t line_position_g = IMAGE_BUFFER_SIZE/2;
static uint16_t line_position_b = IMAGE_BUFFER_SIZE/2;

static uint16_t lineWidth_red = 0;
static uint16_t lineWidth_blue = 0;
static uint16_t lineWidth_green = 0;
static uint8_t color_detected = WHITE;
static bool color_retrieved = false; 
//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */

//returns the color seen by the camera (choices between: red,blue,yellow,black)
uint8_t compute_color(void){
	//uint8_t mean = ((uint16_t)lineWidth_red+(uint16_t)lineWidth_blue+(uint16_t)lineWidth_green)/3;
	if(lineWidth_red>0){
		if(lineWidth_blue>0){
			if(lineWidth_green>0){
				return BLACK;
			}else{
				return GREEN;
			}
		}else{
			return BLUE;
		}
	}else if(lineWidth_green>0){
		return RED;
	}else if(lineWidth_blue>0){
		return YELLOW;
	}else{
		return WHITE;
	}
}

uint16_t extract_line_width(uint8_t *buffer){

	uint16_t i = 0, begin = 0, end = 0, width = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;

	//static uint16_t last_width = PXTOCM/GOAL_DISTANCE;

	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	do{
		wrong_line = 0;
		//search for a begin
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{ 
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean)
		    {
		        begin = i;
		        stop = 1;
		    }
		    i++;
		}
		//if a begin was found, search for an end
		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
		{
		    stop = 0;
		    
		    while(stop == 0 && i < IMAGE_BUFFER_SIZE)
		    {
		        if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean)
		        {
		            end = i;
		            stop = 1;
		        }
		        i++;
		    }
		    //if an end was not found
		    if (i > IMAGE_BUFFER_SIZE || !end)
		    {
		        line_not_found = 1;
		    }
		}
		else//if no begin was found
		{
		    line_not_found = 1;
		}

		//if a line too small has been detected, continues the search
		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			wrong_line = 1;
		}
	}while(wrong_line);

	if(line_not_found){
		begin = 0;
		end = 0;
		width = 0;//last_width;
		line_position = IMAGE_BUFFER_SIZE/2 - EMPIRICAL_CORRECTION;
	}else{
		/*last_width =*/ width = (end - begin);
		line_position = (begin + end)/2; //gives the line position.
	}

	return width;
	//sets a maximum width or returns the measured width
	/*if((PXTOCM/width) > MAX_DISTANCE){
		return PXTOCM/MAX_DISTANCE;
	}else{
		return width;
	}*/
}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 200, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

	//systime_t time;

    while(1){
    	//compute threads execution time
		//time = chVTGetSystemTime();
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
		//
		//chprintf((BaseSequentialStream *)&SDU1, "capture time = %d\n", chVTGetSystemTime()-time);
    }
}


static THD_WORKING_AREA(waProcessImage, 2304);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t red[IMAGE_BUFFER_SIZE] = {0};
	uint8_t blue[IMAGE_BUFFER_SIZE] = {0};
	uint8_t green[IMAGE_BUFFER_SIZE] = {0};
	//uint8_t counter[NUM_COLORS] = {0};
	
	//static uint8_t batch_counter = 0;
	//uint8_t current_color = WHITE;
	bool send_to_computer = true;

	systime_t time;

    while(1){
    	//waits until an image has been captured
    	
        chBSemWait(&image_ready_sem);
        time = chVTGetSystemTime();

        //chprintf((BaseSequentialStream *)&SD3, "capture time = %d\n", chVTGetSystemTime()-time);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts only the red pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
			red[i/2] = (uint8_t)(img_buff_ptr[i]&0xF8)>>3;
			blue[i/2] = (uint8_t)img_buff_ptr[i+1]&0x1F;
			green[i/2] = ((uint8_t)(img_buff_ptr[i]&0x07)<<3) + ((uint8_t)(img_buff_ptr[i+1]&0xE0)>>5);
		}

		//search for a line in the image and gets its width in pixels
		lineWidth_red = extract_line_width(red);
		line_position_r = line_position;
		lineWidth_green = extract_line_width(green);
		line_position_g = line_position;
		lineWidth_blue = extract_line_width(blue);
		line_position_b = line_position;
		
		//chprintf((BaseSequentialStream *)&SDU1, "blue line position = %d\t", line_position);
		//returns color
		/*current_color = get_color();
		if(batch_counter<1){
			if (current_color == BLACK){ counter[BLACK]++; }
			else if (current_color == RED){ counter[RED]++; }
			else if (current_color == YELLOW){ counter[YELLOW]++; }
			else if (current_color == BLUE){ counter[BLUE]++; }
			else if (current_color == GREEN){ counter[GREEN]++; }
			else if (current_color == WHITE){ counter[WHITE]++; }
			batch_counter++;
		}*/
		//if(batch_counter==1){
		color_detected = compute_color();
		color_retrieved = false;
		/*if (color_detected == BLACK){ chprintf((BaseSequentialStream *)&SD3, "black\t"); }
		else if (color_detected == RED){ chprintf((BaseSequentialStream *)&SD3, "RED\t"); }
		else if (color_detected == YELLOW){ chprintf((BaseSequentialStream *)&SD3, "YELLOW\t"); }
		else if (color_detected == BLUE){ chprintf((BaseSequentialStream *)&SD3, "BLUE\t"); }
		else if (color_detected == GREEN){ chprintf((BaseSequentialStream *)&SD3, "GREEN\t"); }
		else if(color_detected==WHITE){ chprintf((BaseSequentialStream *)&SD3, "WHITE\t"); }
		chprintf((BaseSequentialStream *)&SD3, "\n");*/
		/*batch_counter=0; //reset the batch counter
		for (uint8_t i = 0; i<NUM_COLORS; i++){
			counter[i]=0; //reset all the counters for the colors
		}*/
		//}
		//chprintf((BaseSequentialStream *)&SDU1, "red width = %d,  green width = %d,  blue width = %d \n", 
												//lineWidth_red, lineWidth_green, lineWidth_blue);
		//converts the width into a distance between the robot and the camera
		//if(lineWidth){
		//	distance_cm = PXTOCM/lineWidth;
		//}

		/*if(send_to_computer){
			//sends to the computer the image
			SendUint8ToComputer(blue, IMAGE_BUFFER_SIZE);
		}
		//invert the bool
		send_to_computer = !send_to_computer;*/

		//chprintf((BaseSequentialStream *)&SD3, "capture time = %d\n", chVTGetSystemTime()-time);
    }
}

//float get_distance_cm(void){
//	return distance_cm;
//}

uint16_t get_line_position(void){
	return line_position;
}

/*uint8_t get_dominant_color(uint8_t * list){
	uint8_t dominant_color = 0;
	uint8_t color_count = 0;
	for (uint8_t i =0; i < NUM_COLORS; i++){
		if (list[i] > color_count){
			dominant_color = i;
			color_count = list[i];
		}
	}
	return dominant_color;
}*/

uint16_t get_color(void){
	if(color_retrieved == false){
		color_retrieved = true;
		return color_detected;
	}else{
		return OLD_COLOR;
	}
	
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
