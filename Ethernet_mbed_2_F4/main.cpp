#include "mbed.h"
#include "EthernetInterface.h"
#include <math.h>

// own stuff
#include "defines.h"
#include "config.h"
#include "types.h"
#include "controllers.h"
//#include "functionGenerator.h"




// #define DEBUG




// ----------------------    global variables    -------------------------------

Serial pc(USBTX, USBRX);
AnalogIn availableADCs[] = {A0, A1, A2, A3, A4, A5};
DigitalOut greenLed(LED1);
DigitalOut blueLed(LED2);
DigitalOut redLed(LED3);
PwmOut pwmGenerator(PA_5);
Timer dutyCycleTimer;
Timer loopTimer;
Timer debugTimer;
Ticker mainLoopTrigger;

EthernetInterface eth;
UDPSocket socket;
Endpoint endpoint_server;


#define SERVER_IP      "192.168.0.133"
#define PORT           10000


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

// storage for parameters to be send like sensor channels
float fastParameterValues[FAST_PARAMETER_COUNT] = {0.0f};

// storage for parameters, that could be set from the pc
float controlledParameterValues[CONTROLLED_PARAMETER_COUNT] = {0.0f};
// a buffer for incoming messages
float receivedParameters[REQUESTED_CONTROLLED_PARAMETER_COUNT] = {0.0f};

// set points array for the set point generator (TODO)
float setPoints[5] = { 0.0f };

struct messageOut messageToSend;
struct messageIn messageReceived;
int dutyCycleFinishTime = 0;
float pwmPerc = 0.0;
int parameterSendCounter = 0;
int receivedBytesCount = 0;
int sendBytesCount = 0;
int loopCounter = 0;
bool diracInProgress = false;




// ----------------------    function prototypes    ----------------------------
void init();
void loop();
void aquireSensordata();
void communicate();
float generateSetPoint(float time);
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
        }
        loopTimer.reset();
    }
}

void init()
{

    greenLed = 1;
    blueLed = 0;
    redLed = 0;

    pc.printf("ControlDemonstrator\n");

    pwmGenerator.period_us(100);

    // Bring up the network interface

    // laut Dokumentation Initialisierung mit static IP eigentlich möglich bei mbed OS, geht aber nicht
    // es gibt wohl einen Branch von mbed mit einem Patch
    //// eth.init(&ownIP, &mask, &Gateway);

//    // Ethernetconnection with DHCP
//    eth.connect();
//    const char *ip = eth.get_ip_address();
//    const char *mac = eth.get_mac_address();
//    pc.printf("IP address is: %s\n", ip ? ip : "No IP");
//    pc.printf("MAC address is: %s\n", mac ? mac : "No MAC");


    // Ethernetconnection with fixed IP and mbed classic library
    //eth.init(OWN_IP, "255.255.255.0", "192.168.178.1");
    eth.init();
    eth.connect();
    pc.printf("Client IP Address is %s \n", eth.getIPAddress());
    pc.printf("Client MAC Address is %s \n", eth.getMACAddress());


    // Open a socket on the network interface
    socket.bind(PORT);
    socket.init();
    socket.set_blocking(false, 0);


    // this must be done AFTER the socket initialisation
    endpoint_server.set_address(SERVER_IP, PORT);




    // only for testing, set some initial values for controlledParameters
    controlledParameterValues[SLOW_PWM_ON] = 1.0f;
    controlledParameterValues[SLOW_PWM_PERCENT] = 0.5f;

    // the timer is used for duty cycle duration measurement
    dutyCycleTimer.reset();
    dutyCycleTimer.start();

}

void loop()
{
    // debug LED blinkiblinki
    if (loopCounter == 10)
    {
        loopCounter = 0;
        greenLed = !greenLed;
    }
    else
    {
        loopCounter += 1;
    }

    // Calculate duration of last duty cycle
    // at this point the startTime of the last loop is still stored in messageToSend.loopStartTime
    messageToSend.lastLoopDuration = dutyCycleFinishTime - messageToSend.loopStartTime;
    fastParameterValues[LOOP_DURATION] = messageToSend.lastLoopDuration;


    // now overwrite messageToSend.loopStartTime with the actual loop start time
    messageToSend.loopStartTime = dutyCycleTimer.read_us();

    // the dutyCycleTimer is not set to 0 on start of a loop, because the pc
    // needs an absolute time for sorting and plotting the sensor values
    // TODO - check what happens on timer overflow (after approx. 30min)

    // PWM output controlled by pc for little testing
    if (controlledParameterValues[SLOW_PWM_ON] > 0.5f)
    {
        pwmGenerator.write(controlledParameterValues[SLOW_PWM_PERCENT]);
    }
    else
    {
        pwmGenerator.write(0.0f);
    }


    // for debugging - store a controlledParameter value in fastParameters
    // to be able to plot the value on the pc
    fastParameterValues[FAST_PWM_ON] = controlledParameterValues[SLOW_PWM_ON];
    fastParameterValues[FAST_PWM_PERCENT] = controlledParameterValues[SLOW_PWM_PERCENT];


    // read all sensors
    aquireSensordata();

    // get all set points
    setPoints[0] = generateSetPoint((float)messageToSend.loopStartTime / 1000000.0f);

    // control a tank level or so
    fastParameterValues[PID1_ERROR] = setPoints[0] - sensorValues[3];

    float intervalLength = (float)LOOP_CYCLE_TIME_US / 1000000.0f;

    PID_Controller(
        &fastParameterValues[PID1_ERROR],
        &controlledParameterValues[PID1_KP_SWITCH],
        &controlledParameterValues[PID1_KP_VALUE],
        &fastParameterValues[PID1_P_PORTION],
        &controlledParameterValues[PID1_KI_SWITCH],
        &controlledParameterValues[PID1_KI_VALUE],
        &fastParameterValues[PID1_INTEGRAL],
        &fastParameterValues[PID1_I_PORTION],
        &controlledParameterValues[PID1_KD_SWITCH],
        &controlledParameterValues[PID1_KD_VALUE],
        &fastParameterValues[PID1_LAST_ERROR],
        &intervalLength,
        &fastParameterValues[PID1_D_PORTION],
        &fastParameterValues[PID1_SET_POINT]
    );


    actuate();

    communicate();

    // store the time for loop duration measurement
    dutyCycleFinishTime = dutyCycleTimer.read_us();
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
                        fastParameterValues[ANALOG_IN_0] = sensorValues[i];
                        break;
                    case 1:
                        fastParameterValues[ANALOG_IN_1] = sensorValues[i];
                        break;
                    case 2:
                        fastParameterValues[ANALOG_IN_2] = sensorValues[i];
                        break;
                    case 3:
                        fastParameterValues[ANALOG_IN_3] = sensorValues[i];
                        break;
                    case 4:
                        fastParameterValues[ANALOG_IN_4] = sensorValues[i];
                        break;
                    case 5:
                        fastParameterValues[ANALOG_IN_5] = sensorValues[i];
                        break;
                }
                break;

            // implement alternate measuring method here
            case 2:
                break;
        }
    }
}


void communicate()
{
    // map all requested data to the outgoing message struct

    int i;

//    // map sensor values
//    int n = 0;
//    for (i = 0; i < SENSOR_COUNT; i++)
//    {
//        messageToSend.measuredValues[n] = sensorValues[i];
//        n += 1;
//    }

    // map fast parameters
    for (i = 0; i < REQUESTED_FAST_PARAMETER_COUNT; i++)
    {
        messageToSend.fastParameterValues[i] = fastParameterValues[requestedFastParameters[i]];
    }

    // map controlled (rotating) parameters
    // on each cycle, only one of the "controlled parameters" is send to the pc
    messageToSend.parameterNumber = parameterSendCounter;
    //messageToSend.parameterValue = controlledParameterValues[requestedControlledParameters[parameterSendCounter]];
    messageToSend.parameterValue = controlledParameterValues[parameterSendCounter];

    // increment the counter for sending the "slow parameters"
    parameterSendCounter += 1;
    if (parameterSendCounter >= CONTROLLED_PARAMETER_COUNT)
    {
        parameterSendCounter = 0;
    }


    blueLed = 0;
    redLed = 0;




    // loopStartTime and lastLoopDuration are allready set from within the loop

    // send sensor data to the pc via Ethernet with mbed classic
    sendBytesCount = socket.sendTo(endpoint_server, (char *)&messageToSend, sizeof(messageToSend));

    #ifdef DEBUG
    pc.printf("SEND: %d %d %f\n", sendBytesCount, messageToSend.parameterNumber, messageToSend.parameterValue);
    #endif


    debugTimer.reset();
    debugTimer.start();


    // receive a command
    fastParameterValues[RECEIVED_BYTES_COUNT] = (float)socket.receiveFrom(endpoint_server, (char *)&messageReceived, sizeof(messageReceived));


    fastParameterValues[DEBUG_TIMER] = (float)debugTimer.read_us();



    if (fastParameterValues[RECEIVED_BYTES_COUNT] > 0.5f)
    {
        controlledParameterValues[messageReceived.parameterNumber] = messageReceived.value;

        fastParameterValues[LAST_COMMAND_ID] = messageReceived.parameterNumber;
        fastParameterValues[LAST_COMMAND_VALUE] = messageReceived.value;
    }
    fastParameterValues[LAST_COMMAND_FROM_ARRAY_VALUE] = controlledParameterValues[messageReceived.parameterNumber];





    // receive bytes from socket while buffer is not empty - TODO

    // listen to the pc with mbed OS
    // ACHTUNG: hier server_adress als pointer übergeben
    // fastParameterValues[RECEIVED_BYTES_COUNT] = (float)socketRX.recvfrom(&udp_server_address_TX, &receivedParameters, sizeof(receivedParameters));


    #ifdef DEBUG
    pc.printf("RECV: %f %d %f\n", fastParameterValues[RECEIVED_BYTES_COUNT], messageReceived.parameterNumber, messageReceived.value);
    #endif

    if (fastParameterValues[RECEIVED_BYTES_COUNT] > 0)
    {
//        // map reveived parameters to controlledParameterValues array
//        for (i = 0; i < REQUESTED_CONTROLLED_PARAMETER_COUNT; i++)
//        {
//            controlledParameterValues[requestedControlledParameters[i]] = receivedParameters[i];
//        }

        // blue led blinkiblinki on successfull receive
        blueLed = 1;

        #ifdef DEBUG
        pc.printf("SET: %f %d %f\n", fastParameterValues[RECEIVED_BYTES_COUNT], messageReceived.parameterNumber, controlledParameterValues[messageReceived.parameterNumber]);
        #endif
    }
    else
    {
        // red led blinkiblinki on nothing received
        redLed = 1;
    }
}


// not yet tested
float generateSetPoint(float timeInS)
{
    float setPoint = 0.0f;

    switch ((int)(controlledParameterValues[SP_GEN1_NUMBER] + 0.5f))
    {
        case 0:
            // generates a dirac impulse
            if ((controlledParameterValues[SP_GEN1_DIRAC_NOW] > 0.5f))
            {
                controlledParameterValues[SP_GEN1_DIRAC_START_TIME] = (float)timeInS;
                controlledParameterValues[SP_GEN1_DIRAC_NOW] = 0.0f;
            }

            if ((float)timeInS - controlledParameterValues[SP_GEN1_DIRAC_START_TIME] < controlledParameterValues[SP_GEN1_DIRAC_DURATION])
            {
                setPoint = controlledParameterValues[SP_GEN1_DIRAC_HIGH];
            }
            else
            {
                setPoint = controlledParameterValues[SP_GEN1_DIRAC_LOW];
            }

            break;

        case 1:
            // generates a heaviside function
            if (controlledParameterValues[SP_GEN1_STEP_STATE] > 0.5f)
            {
                setPoint = controlledParameterValues[SP_GEN1_STEP_HIGH];
            }
            else
            {
                setPoint = controlledParameterValues[SP_GEN1_STEP_LOW];
            }
            break;

        case 2:
            // generates a ramp

            // if ramp gets toggled between upslope and downslope
            if (controlledParameterValues[SP_GEN1_RAMP_STATE] != controlledParameterValues[SP_GEN1_RAMP_LAST_STATE])
            {
                controlledParameterValues[SP_GEN1_RAMP_LAST_STATE] = controlledParameterValues[SP_GEN1_RAMP_STATE];
                controlledParameterValues[SP_GEN1_RAMP_START_TIME] = timeInS;
            }

            if ((controlledParameterValues[SP_GEN1_RAMP_GRADIENT] == 0.0f) || (timeInS == 0.0f))
            {
                setPoint = 0.0f;
                break;
            }

            // if upslope wanted
            if (controlledParameterValues[SP_GEN1_RAMP_STATE] < 0.5f)
            {
                float offset = (controlledParameterValues[SP_GEN1_RAMP_LOW]) -
                    (controlledParameterValues[SP_GEN1_RAMP_GRADIENT] * controlledParameterValues[SP_GEN1_RAMP_START_TIME]);

                float rampValue = controlledParameterValues[SP_GEN1_RAMP_GRADIENT] * timeInS + offset;


                if (rampValue > controlledParameterValues[SP_GEN1_RAMP_HIGH])
                {
                    setPoint = controlledParameterValues[SP_GEN1_RAMP_HIGH];
                }
                else
                {
                    setPoint = rampValue;
                }
            }
            // if downslope wanted -> the gradient must be multiplied by -1 !!
            if (controlledParameterValues[SP_GEN1_RAMP_STATE] > 0.5f)
            {
                float offset = controlledParameterValues[SP_GEN1_RAMP_HIGH] +
                    (controlledParameterValues[SP_GEN1_RAMP_GRADIENT] * controlledParameterValues[SP_GEN1_RAMP_START_TIME]);

                float rampValue = -1.0f * controlledParameterValues[SP_GEN1_RAMP_GRADIENT] * timeInS + offset;


                if (rampValue < controlledParameterValues[SP_GEN1_RAMP_LOW])
                {
                    setPoint = controlledParameterValues[SP_GEN1_RAMP_LOW];
                }
                else
                {
                    setPoint = rampValue;
                }
            }



            break;

        case 3:
            // generates a sinus
            setPoint = controlledParameterValues[SP_GEN1_SIN_AMPLITUDE] * (float)sin(controlledParameterValues[SP_GEN1_SIN_OMEGA] * timeInS) + controlledParameterValues[SP_GEN1_SIN_OFFSET];
            break;

        case 4:
            // generates a square wave

            // toggle between high and low
            float half_T = 0.5f * (1.0f/controlledParameterValues[SP_GEN1_SQUARE_FREQUENCY]);

            if (timeInS - controlledParameterValues[SP_GEN1_SQUARE_LAST_TOGGLE] > half_T)
            {
                controlledParameterValues[SP_GEN1_SQUARE_LAST_TOGGLE] = timeInS;
                if (controlledParameterValues[SP_GEN1_SQUARE_STATE] < 0.5f)
                {
                    controlledParameterValues[SP_GEN1_SQUARE_STATE] = 1.0f;
                }
                else
                {
                    controlledParameterValues[SP_GEN1_SQUARE_STATE] = 0.0f;
                }
            }

            // set high or low
            if (controlledParameterValues[SP_GEN1_SQUARE_STATE] < 0.5f)
            {
                setPoint = controlledParameterValues[SP_GEN1_SQUARE_LOW];
            }
            else
            {
                setPoint = controlledParameterValues[SP_GEN1_SQUARE_HIGH];
            }


            break;
    }
    fastParameterValues[SP_GEN_1_OUTPUT] = setPoint;
    return setPoint;
}




// to be implemented
int actuate()
{
    //pwmGenerator.write(0.5f);
    return -1;
}
