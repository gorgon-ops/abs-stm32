
#ifndef L9110S_H
#define L9110S_H
#include "board_config.h"
#ifndef BOARD_L9110S_HAS_COAST
#define BOARD_L9110S_HAS_COAST 0
#endif

void l9110s_init(void);

/* Forward drive, 0.0 to 1.0. */
void l9110s_drive(float duty);

/* Braking, 0.0 to 1.0 .  */
void l9110s_brake(float duty);

void l9110s_stop(void);

#endif 
