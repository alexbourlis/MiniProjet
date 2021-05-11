#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

#define BLACK 	0 
#define RED		1
#define YELLOW	2
#define BLUE 	3 
#define WHITE	4
#define NUM_COLORS	5

float get_distance_cm(void);
uint16_t get_line_position(void);
void process_image_start(void);
uint8_t get_color(void);
uint8_t get_dominant_color(uint8_t * list);
uint16_t get_color_pid(void);

#endif /* PROCESS_IMAGE_H */
