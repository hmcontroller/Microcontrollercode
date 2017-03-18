#include "signal_generator.h"

float generateSignal(SignalGeneratorParameters *params)
{
    float signalValue = 0.0f;
    switch ((int)( *(params->generatorNumber) + 0.5f))
    {
        case 0:
            // generates a dirac impulse
            signalValue = generateDirac(
                params->timeInS,
                params->diracStartTime,
                params->diracNow,
                params->diracLow,
                params->diracHigh,
                params->diracDuration);
            break;
        case 1:
            // generates a heaviside function
            signalValue = generateHeavySide(
                params->stepState,
                params->stepLow,
                params->stepHigh);
            break;
        case 2:
            // generates a ramp
            signalValue = generateRamp(
                params->timeInS,
                params->rampStartTime,
                params->rampState,
                params->rampLastState,
                params->rampGradient,
                params->rampLow,
                params->rampHigh);
            break;
        case 3:
            // generates a sinus
            signalValue = generateSinus(
                params->timeInS,
                params->sinusAmplitude,
                params->sinusOmega,
                params->sinusOffset);
            break;
        case 4:
            // generates a square wave
            signalValue = generateSquareWave(
                params->timeInS,
                params->squareLow,
                params->squareHigh,
                params->squareFrequency,
                params->squareState,
                params->squareLastToggle);
            break;
    }
    return signalValue;
}

float generateDirac(
    float timeInS,
    float *diracStartTime,
    float *diracNow,
    float *diracLow,
    float *diracHigh,
    float *diracDuration)
{
    float signalValue = 0.0f;

    if (*diracNow > 0.5f)
    {
        *diracStartTime = timeInS;
        *diracNow = 0.0f;
    }

    if (timeInS - *diracStartTime < *diracDuration)
    {
        signalValue = *diracHigh;
    }
    else
    {
        signalValue = *diracLow;
    }
    return signalValue;
}



float generateHeavySide(
    float *stepState,
    float *stepLow,
    float *stepHigh)
{
    float signalValue = 0.0f;

    if (*stepState > 0.5f)
    {
        signalValue = *stepHigh;
    }
    else
    {
        signalValue = *stepLow;
    }
    return signalValue;
}

float generateRamp(
    float timeInS,
    float *rampStartTime,
    float *rampState,
    float *rampLastState,
    float *rampGradient,
    float *rampLow,
    float *rampHigh)
{
    float signalValue = 0.0f;

    // if ramp gets toggled between upslope and downslope
    if (*rampState != *rampLastState)
    {
        *rampLastState = *rampState;
        *rampStartTime = timeInS;
    }

    if ((*rampGradient == 0.0f) || (timeInS == 0.0f))
    {
        signalValue = 0.0f;
    }

    // if upslope wanted
    if (*rampState < 0.5f)
    {
        float offset = (*rampLow) -
            (*rampGradient * *rampStartTime);

        float rampValue = *rampGradient * timeInS + offset;


        if (rampValue > *rampHigh)
        {
            signalValue = *rampHigh;
        }
        else
        {
            signalValue = rampValue;
        }
    }
    // if downslope wanted -> the gradient must be multiplied by -1 !!
    if (*rampState > 0.5f)
    {
        float offset = *rampHigh +
            (*rampGradient * *rampStartTime);

        float rampValue = -1.0f * *rampGradient * timeInS + offset;


        if (rampValue < *rampLow)
        {
            signalValue = *rampLow;
        }
        else
        {
            signalValue = rampValue;
        }
    }
    return signalValue;
}

float generateSinus(
    float timeInS,
    float *sinusAmplitude,
    float *sinusOmega,
    float *sinusOffset)
{
    float signalValue = 0.0f;

    signalValue = *sinusAmplitude * (float)sin(*sinusOmega * timeInS) + *sinusOffset;
    return signalValue;
}

float generateSquareWave(
    float timeInS,
    float *squareLow,
    float *squareHigh,
    float *squareFrequency,
    float *squareState,
    float *squareLastToggle)
{
    float signalValue = 0.0f;

    // toggle between high and low
    float half_T = 0.5f * (1.0f/ (*squareFrequency));

    if (timeInS - *squareLastToggle > half_T)
    {
        *squareLastToggle = timeInS;
        if (*squareState < 0.5f)
        {
            *squareState = 1.0f;
        }
        else
        {
            *squareState = 0.0f;
        }
    }

    // set high or low
    if (*squareState < 0.5f)
    {
        signalValue = *squareLow;
    }
    else
    {
        signalValue = *squareHigh;
    }
    return signalValue;
}
