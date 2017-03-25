#ifndef SIGNAL_GENERATOR_H
#define SIGNAL_GENERATOR_H

#include "math.h"
#include "types.h"

float generateSignal(SignalGeneratorParameters *params);

float generateDirac(
    float timeInS,
    float *diracStartTime,
    float *diracNow,
    float *diracLow,
    float *diracHigh,
    float *diracDuration);

float generateHeavySide(
    float *stepState,
    float *stepLow,
    float *stepHigh);

float generateRamp(
    float timeInS,
    float *rampStartTime,
    float *rampState,
    float *rampLastState,
    float *rampGradient,
    float *rampLow,
    float *rampHigh);

float generateSinus(
    float timeInS,
    float *sinusAmplitude,
    float *sinusOmega,
    float *sinusOffset);

float generateSquareWave(
    float timeInS,
    float *squareLow,
    float *squareHigh,
    float *squareFrequency,
    float *squareState,
    float *squareLastToggle);

#endif
