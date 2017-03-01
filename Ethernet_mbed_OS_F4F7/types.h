#ifndef TYPES_H
#define TYPES_H

#include <config.h>

struct messageOut 
{
    float fastParameterValues[REQUESTED_FAST_PARAMETER_COUNT];
    int loopStartTime;
    int lastLoopDuration;
    int parameterNumber;
    float parameterValue;
};

struct messageIn 
{
    int parameterNumber;
    float value;
};




#endif
