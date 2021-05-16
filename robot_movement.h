#ifndef ROBOT_MOVEMENT_H
#define ROBOT_MOVEMENT_H

#define BASESPEED			500	//speed of the robot
#define BLACK_SQUARE_LOW	184	//Black squares have a with of 12mm wich coressponds to, approximately, a width of 195 pixels
#define BLACK_SQUARE_HIGH	220	

/**starts the game, the robot will take decision depending on the situation and keep track of the score and players turn
 */
void robot_movement_start(void);

#endif /* ROBOT_MOVEMENT */
