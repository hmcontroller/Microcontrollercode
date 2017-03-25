#include "mbed.h"

// own stuff
#include "defines.h"
#include "config.h"
#include "types.h"
#include "controllers.h"
#include "signal_generator.h"




//#define DEBUG




// ----------------------    global variables    -------------------------------

Serial pc(USBTX, USBRX);
AnalogIn availableADCs[] = {A0, A1, A2, A3, A4, A5};
PwmOut pwmGenerator(PA_5);
Timer loopTimer;
Timer timeSinceStart;
//Ticker mainLoopTrigger;





// sensor channels, that will be measured
// [0]: switchCase for aquisition method, see "void aquireSensordata()"
// [1]: a parameter for the aquistion method
#define SENSOR_COUNT 6
int sensorMapping[SENSOR_COUNT][2] =
{
    {1, 0},
    {1, 1},
    {1, 2},
    {1, 3},
    {1, 4},
    {1, 5}
};


// storage for sensor values
float sensorValues[SENSOR_COUNT] = {0.0f};


// set points array for the set point generator (TODO)
float setPoints[5] = { 0.0f };


int loopStartTime = 0;
float pwmPerc = 0.0;
int loopCounter = 0;

// initialize parameters for a PID controller
PidParameters pidOneParameters;
float pidOneIntervalLength = (float)LOOP_CYCLE_TIME_US / 1000000.0f;

// initialize parameters for a signal generator
SignalGeneratorParameters signalGeneratorOneParams;


// ----------------------    function prototypes    ----------------------------
void init();
void loop();
void aquireSensordata();
int actuate();






// ----------------------    functions    --------------------------------------


int main()
{
    init();

    // gives the error "sys_sem_new create error", dont know why
    //// attach the main loop to a timer (ticker)
    // mainLoopTrigger.attach(&loop, 1.0);
    // while(1){}

    // patch:
    while(1)
    {
        loopTimer.start();
        loop();
        while(loopTimer.read_us() < LOOP_CYCLE_TIME_US)
        {
            // do nothing
        }
        loopTimer.reset();
    }
}

void init()
{
    initControlDemonstrator();

    pc.printf("ControlDemonstrator\n");

    pwmGenerator.period_us(100);




    // only for testing, set some initial values for controlledParameters
    SLOW_PWM_ON = 1.0f;
    SLOW_PWM_PERCENT = 0.5f;



    pidOneParameters.error          = &PID1_ERROR;
    pidOneParameters.kpSwitch       = &PID1_KP_SWITCH;
    pidOneParameters.kpValue        = &PID1_KP_VALUE;
    pidOneParameters.pPortion       = &PID1_P_PORTION;
    pidOneParameters.kiValue        = &PID1_KI_VALUE;
    pidOneParameters.kiSwitch       = &PID1_KI_SWITCH;
    pidOneParameters.integral       = &PID1_INTEGRAL;
    pidOneParameters.iPortion       = &PID1_I_PORTION;
    pidOneParameters.kdSwitch       = &PID1_KD_SWITCH;
    pidOneParameters.kdValue        = &PID1_KD_VALUE;
    pidOneParameters.lastError      = &PID1_LAST_ERROR;
    pidOneParameters.intervalLength = &pidOneIntervalLength;
    pidOneParameters.dPortion       = &PID1_D_PORTION;
    pidOneParameters.setPoint       = &PID1_SET_POINT;


    signalGeneratorOneParams.generatorNumber    = &SP_GEN1_NUMBER;
    signalGeneratorOneParams.diracStartTime     = &SP_GEN1_DIRAC_START_TIME;
    signalGeneratorOneParams.diracNow           = &SP_GEN1_DIRAC_NOW;
    signalGeneratorOneParams.diracLow           = &SP_GEN1_DIRAC_LOW;
    signalGeneratorOneParams.diracHigh          = &SP_GEN1_DIRAC_HIGH;
    signalGeneratorOneParams.diracDuration      = &SP_GEN1_DIRAC_DURATION;
    signalGeneratorOneParams.stepState          = &SP_GEN1_STEP_STATE;
    signalGeneratorOneParams.stepLow            = &SP_GEN1_STEP_LOW;
    signalGeneratorOneParams.stepHigh           = &SP_GEN1_STEP_HIGH;
    signalGeneratorOneParams.rampStartTime      = &SP_GEN1_RAMP_START_TIME;
    signalGeneratorOneParams.rampState          = &SP_GEN1_RAMP_STATE;
    signalGeneratorOneParams.rampLastState      = &SP_GEN1_RAMP_LAST_STATE;
    signalGeneratorOneParams.rampGradient       = &SP_GEN1_RAMP_GRADIENT;
    signalGeneratorOneParams.rampLow            = &SP_GEN1_RAMP_LOW;
    signalGeneratorOneParams.rampHigh           = &SP_GEN1_RAMP_HIGH;
    signalGeneratorOneParams.sinusAmplitude     = &SP_GEN1_SIN_AMPLITUDE;
    signalGeneratorOneParams.sinusOmega         = &SP_GEN1_SIN_OMEGA;
    signalGeneratorOneParams.sinusOffset        = &SP_GEN1_SIN_OFFSET;
    signalGeneratorOneParams.squareLow          = &SP_GEN1_SQUARE_LOW;
    signalGeneratorOneParams.squareHigh         = &SP_GEN1_SQUARE_HIGH;
    signalGeneratorOneParams.squareFrequency    = &SP_GEN1_SQUARE_FREQUENCY;
    signalGeneratorOneParams.squareState        = &SP_GEN1_SQUARE_STATE;
    signalGeneratorOneParams.squareLastToggle   = &SP_GEN1_SQUARE_LAST_TOGGLE;

}

void loop()
{
    loopStartTime = timeSinceStart.read_us();
    // debug LED blinkiblinki
    if (loopCounter == 10)
    {
        loopCounter = 0;
        //greenLed = !greenLed;
    }
    else
    {
        loopCounter += 1;
    }


    // PWM output controlled by pc for little testing
    if (SLOW_PWM_ON > 0.5f)
    {
        pwmGenerator.write(SLOW_PWM_PERCENT);
    }
    else
    {
        pwmGenerator.write(0.0f);
    }


    // for debugging - store a controlled parameter value in channels
    // to be able to plot the value on the pc
    FAST_PWM_ON = SLOW_PWM_ON;
    FAST_PWM_PERCENT = SLOW_PWM_PERCENT;


    // read all sensors
    aquireSensordata();

    // get a set point
    signalGeneratorOneParams.timeInS = ((float)loopStartTime / 1000000.0f);
    setPoints[0] = generateSignal(&signalGeneratorOneParams);
    SP_GEN_1_OUTPUT = setPoints[0];

    // control a tank level or so
    PID1_ERROR = setPoints[0] - sensorValues[3];
    PID_Controller(&pidOneParameters);


    actuate();


    communicate();

//    // store the time for loop duration measurement
//    dutyCycleFinishTime = dutyCycleTimer.read_us();
}


void aquireSensordata()
{
    int i;
    for (i = 0; i < SENSOR_COUNT; i++)
    {
        // decide which kind of aquisition is needed
        switch (sensorMapping[i][0])
        {
            // read builtin ADC
            case 1:
                sensorValues[i] = availableADCs[sensorMapping[i][1]].read_u16();

                // refactor this dirty hack
                switch (i)
                {
                    case 0:
                        ANALOG_IN_0 = sensorValues[i];
                        break;
                    case 1:
                        ANALOG_IN_1 = sensorValues[i];
                        break;
                    case 2:
                        ANALOG_IN_2 = sensorValues[i];
                        break;
                    case 3:
                        ANALOG_IN_3 = sensorValues[i];
                        break;
                    case 4:
                        ANALOG_IN_4 = sensorValues[i];
                        break;
                    case 5:
                        ANALOG_IN_5 = sensorValues[i];
                        break;
                }
                break;

            // implement alternate measuring method here
            case 2:
                break;
        }
    }
}




// to be implemented
int actuate()
{
    //pwmGenerator.write(0.5f);
    return -1;
}
