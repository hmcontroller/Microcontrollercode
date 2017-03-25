#ifndef TYPES_H
#define TYPES_H

#include <config.h>


typedef struct PidParameters
{
    float *error;
    float *kpSwitch;
    float *kpValue;
    float *pPortion;
    float *kiSwitch;
    float *kiValue;
    float *integral;
    float *iPortion;
    float *kdSwitch;
    float *kdValue;
    float *lastError;
    float *intervalLength;
    float *dPortion;
    float *setPoint;
} PidParameters;

typedef struct SignalGeneratorParameters
{
    float timeInS;
    float *generatorNumber;
    float *diracStartTime;
    float *diracNow;
    float *diracLow;
    float *diracHigh;
    float *diracDuration;
    float *stepState;
    float *stepLow;
    float *stepHigh;
    float *rampStartTime;
    float *rampState;
    float *rampLastState;
    float *rampGradient;
    float *rampLow;
    float *rampHigh;
    float *sinusAmplitude;
    float *sinusOmega;
    float *sinusOffset;
    float *squareLow;
    float *squareHigh;
    float *squareFrequency;
    float *squareState;
    float *squareLastToggle;
} SignalGeneratorParameters;

#endif
