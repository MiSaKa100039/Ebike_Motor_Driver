#ifndef USERAPP_H
#define USERAPP_H

void Init(void);
void loop(void);

#include "main.h"
#include "Platform_Motor.h"
#include "Platform_VOFA.h"

#define constrain(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))

#endif //USERAPP_H
