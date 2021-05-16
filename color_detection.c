#include "ch.h"
#include "hal.h"
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>
#include <color_detection.h>

static uint16_t line_position[NUM_RGB] = {IMAGE_BUFFER_SIZE/2};
static uint16_t lineWidth[NUM_RGB] = {0};
static uint8_t color_detected = WHITE;
//semaphores
static BSEMAPHORE_DECL(image_ready_sem, TRUE);
static BSEMAPHORE_DECL(color_computed_sem, TRUE); //indicates when a color has been computed and is ready to be read
												  //by some external function

/*
 * Returns the color seen by the camera (choices between: red,blue,yellow,black)
 */
uint8_t compute_color(void){
	if(lineWidth[RED]>0){
		if(lineWidth[BLUE]>0){
			if(lineWidth[GREEN]>0){
				return BLACK;
			}else{
				return GREEN;
			}
		}else{
			return BLUE;
		}
	}else if(lineWidth[GREEN]>0){
		return RED;
	}else if(lineWidth[BLUE]>0){
		return YELLOW;
	}else{
		return WHITE;
	}
}

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */

uint16_t extract_line_width(uint8_t *buffer, uint8_t color){

	uint16_t i = 0, begin = 0, end = 0, width = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;
	uint32_t max = 0;

	//finds max values
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		if(buffer[i]>max){
			max = buffer[i];
		}
	}
	mean = max/2;

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
		width = 0;
		line_position[color] = IMAGE_BUFFER_SIZE/2 - EMPIRICAL_CORRECTION;
	}else{
		width = (end - begin);
		line_position[color] = (begin + end)/2; //gives the line position.
	}

	return width;
}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 300, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
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

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
       
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts rgb pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			red[i/2] = (uint8_t)(img_buff_ptr[i]&0xF8)>>3;	//red mask
			blue[i/2] = (uint8_t)img_buff_ptr[i+1]&0x1F;	//blue mask
			green[i/2] = ((uint8_t)(img_buff_ptr[i]&0x07)<<3) + ((uint8_t)(img_buff_ptr[i+1]&0xE0)>>5);	//green mask
		}

		//search for a line in the image and gets its width in pixels
		lineWidth[RED] = extract_line_width(red, RED);
		lineWidth[GREEN] = extract_line_width(green, GREEN);
		lineWidth[BLUE] = extract_line_width(blue, BLUE);

		color_detected = compute_color();
		chBSemSignal(&color_computed_sem);
    }
}

uint16_t get_line_position(uint8_t color){
	if(color<NUM_RGB){
		return line_position[color];
	}
	return 0;
}

uint16_t get_line_width(uint8_t color){
	if(color<NUM_RGB){
		return lineWidth[color];
	}
	return 0;
}

void wait_color_computed(void){
	chBSemWait(&color_computed_sem);
}

uint16_t get_color(void){
	return color_detected;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
