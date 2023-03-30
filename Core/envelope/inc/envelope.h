//
// Created by adam slaymark on 28/03/2023.
//

#ifndef F7_LOOPBACK_ENVELOPE_H
#define F7_LOOPBACK_ENVELOPE_H

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <float.h>
#include <string.h>
#include "arm_math.h"

///////////////////
// User defined Private Definitions
///////////////////


/////////////////
// Static Definitions
/////////////////




///////////////////////
// Function Prototypes
///////////////////////
void envelope(int16_t*, float32_t*);
void envelope_alloc();

#endif //F7_LOOPBACK_ENVELOPE_H
