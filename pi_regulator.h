#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

#define BASESPEED 500

//start the PI regulator thread
void pi_regulator_start(void);
void reset_counter(uint8_t color);
void left_turn(void);
void right_turn(void);

#endif /* PI_REGULATOR_H */
