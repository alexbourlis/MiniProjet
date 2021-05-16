#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			100
#define CORRECTION_THRESHOLD	10
#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera
#define KP						500.0f
#define KI 						0.5f	//must not be zero
#define MAX_SUM_ERROR 			100
#define STOP  					0		//the game is paused
#define START					1		//the game is on
#define ALIGNEMENT_THRESHOLD	10
#define FINAL_SCORE				2

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;
/**returns the game state: either START or STOP
 */
uint8_t get_game_state(void);
/**sets the game state: either START or STOP
 */
void set_game_state(uint8_t state);
/**plays a small melody for the end of the game
 */
void end_melody(void);

#ifdef __cplusplus
}
#endif

#endif
