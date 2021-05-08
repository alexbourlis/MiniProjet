#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

#define BLACK 	0 
#define RED		1
#define YELLOW	2
#define BLUE 	3 
#define NONE	4
#define NUM_COLORS	4

float get_distance_cm(void);
uint16_t get_line_position(void);
void process_image_start(void);
uint8_t get_color(void);
uint8_t get_dominant_color(uint8_t * list);

#endif /* PROCESS_IMAGE_H */