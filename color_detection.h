#ifndef COLOR_DETECTION_H
#define COLOR_DETECTION_H

#define RED			0
#define GREEN		1
#define BLUE 		2
#define BLACK 		3 
#define YELLOW		4
#define WHITE		5
#define NUM_COLORS	6
#define NUM_RGB		3

#define EMPIRICAL_CORRECTION 5	// by experimentation we found this value to perfectly center the robot on the lines

/**Returns the middle pixel of the line from the given color 
 */
uint16_t get_line_position(uint8_t color);
/**Capture images and proccesses them
 */
void process_image_start(void);
/**Returns the current detected color
 */
uint16_t get_color(void);
/**wait for a new color to be detected through the semaphore
 */
void wait_color_computed(void);
/**Returns the line width
 */
uint16_t get_line_width(uint8_t color);

#endif /* COLOR_DETECTION_H */
