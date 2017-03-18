#include "mbed.h"
//#include "UDPSocket.h"
//#include "lwip-interface/EthernetInterface.h"
#include "EthernetInterface.h"
#include "SocketAddress.h"

// own stuff
#include "defines.h"
#include "config.h"
#include "types.h"
#include "controllers.h"
#include "signal_generator.h"


#define SERVER_IP       "192.168.0.133"
#define OWN_IP          "192.168.0.15"
#define PORT            10000
#define NET_MASK        "255.255.0.0"
#define GATEWAY         "192.168.0.1"


//#define DEBUG




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
//Ticker mainLoopTrigger;

EthernetInterface eth;
UDPSocket socket;
SocketAddress udp_server_address(SERVER_IP, PORT);
SocketAddress dump_address(SERVER_IP, PORT);




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

// storage for channels
float channels[AVAILABLE_CHANNEL_COUNT] = {0.0f};

// storage for parameters, that could be set from the pc
float parameters[AVAILABLE_PARAMETER_COUNT] = {0.0f};

// a buffer for incoming messages
//float receivedParameters[REQUESTED_PARAMETER_COUNT] = {0.0f};

// set points array for the set point generator (TODO)
float setPoints[5] = { 0.0f };

MessageOut messageOutBuffer;
MessageIn messageInBuffer;

int dutyCycleFinishTime = 0;
float pwmPerc = 0.0;
int parameterSendCounter = 0;
int receivedBytesCount = 0;
int sendBytesCount = 0;
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
void prepareMessage();
void communicate();
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

    greenLed = 1;
    blueLed = 0;
    redLed = 0;

    pc.printf("ControlDemonstrator\n");

    pwmGenerator.period_us(100);

    // Bring up the network interface

    // laut Dokumentation Initialisierung mit static IP eigentlich möglich bei mbed OS, geht aber nicht
    // es gibt wohl einen Branch von mbed mit einem Patch
    //// eth.init(&ownIP, &mask, &Gateway);

    // Ethernetconnection with DHCP
    eth.set_network(OWN_IP, NET_MASK, GATEWAY);
    eth.connect();
    const char *ip = eth.get_ip_address();
    const char *mac = eth.get_mac_address();

    pc.printf("IP address is: %s\n", ip ? ip : "No IP");
    pc.printf("MAC address is: %s\n", mac ? mac : "No MAC");

    socket.open(&eth);
    socket.bind(PORT);
    socket.set_blocking(false);



    // only for testing, set some initial values for controlledParameters
    parameters[SLOW_PWM_ON] = 1.0f;
    parameters[SLOW_PWM_PERCENT] = 0.5f;

    // the timer is used for duty cycle duration measurement
    dutyCycleTimer.reset();
    dutyCycleTimer.start();


    pidOneParameters.error          = &channels[PID1_ERROR];
    pidOneParameters.kpSwitch       = &parameters[PID1_KP_SWITCH];
    pidOneParameters.kpValue        = &parameters[PID1_KP_VALUE];
    pidOneParameters.pPortion       = &channels[PID1_P_PORTION];
    pidOneParameters.kiValue        = &parameters[PID1_KI_VALUE];
    pidOneParameters.kiSwitch       = &parameters[PID1_KI_SWITCH];
    pidOneParameters.integral       = &channels[PID1_INTEGRAL];
    pidOneParameters.iPortion       = &channels[PID1_I_PORTION];
    pidOneParameters.kdSwitch       = &parameters[PID1_KD_SWITCH];
    pidOneParameters.kdValue        = &parameters[PID1_KD_VALUE];
    pidOneParameters.lastError      = &channels[PID1_LAST_ERROR];
    pidOneParameters.intervalLength = &pidOneIntervalLength;
    pidOneParameters.dPortion       = &channels[PID1_D_PORTION];
    pidOneParameters.setPoint       = &channels[PID1_SET_POINT];


    signalGeneratorOneParams.generatorNumber    = &parameters[SP_GEN1_NUMBER];
    signalGeneratorOneParams.diracStartTime     = &parameters[SP_GEN1_DIRAC_START_TIME];
    signalGeneratorOneParams.diracNow           = &parameters[SP_GEN1_DIRAC_NOW];
    signalGeneratorOneParams.diracLow           = &parameters[SP_GEN1_DIRAC_LOW];
    signalGeneratorOneParams.diracHigh          = &parameters[SP_GEN1_DIRAC_HIGH];
    signalGeneratorOneParams.diracDuration      = &parameters[SP_GEN1_DIRAC_DURATION];
    signalGeneratorOneParams.stepState          = &parameters[SP_GEN1_STEP_STATE];
    signalGeneratorOneParams.stepLow            = &parameters[SP_GEN1_STEP_LOW];
    signalGeneratorOneParams.stepHigh           = &parameters[SP_GEN1_STEP_HIGH];
    signalGeneratorOneParams.rampStartTime      = &parameters[SP_GEN1_RAMP_START_TIME];
    signalGeneratorOneParams.rampState          = &parameters[SP_GEN1_RAMP_STATE];
    signalGeneratorOneParams.rampLastState      = &parameters[SP_GEN1_RAMP_LAST_STATE];
    signalGeneratorOneParams.rampGradient       = &parameters[SP_GEN1_RAMP_GRADIENT];
    signalGeneratorOneParams.rampLow            = &parameters[SP_GEN1_RAMP_LOW];
    signalGeneratorOneParams.rampHigh           = &parameters[SP_GEN1_RAMP_HIGH];
    signalGeneratorOneParams.sinusAmplitude     = &parameters[SP_GEN1_SIN_AMPLITUDE];
    signalGeneratorOneParams.sinusOmega         = &parameters[SP_GEN1_SIN_OMEGA];
    signalGeneratorOneParams.sinusOffset        = &parameters[SP_GEN1_SIN_OFFSET];
    signalGeneratorOneParams.squareLow          = &parameters[SP_GEN1_SQUARE_LOW];
    signalGeneratorOneParams.squareHigh         = &parameters[SP_GEN1_SQUARE_HIGH];
    signalGeneratorOneParams.squareFrequency    = &parameters[SP_GEN1_SQUARE_FREQUENCY];
    signalGeneratorOneParams.squareState        = &parameters[SP_GEN1_SQUARE_STATE];
    signalGeneratorOneParams.squareLastToggle   = &parameters[SP_GEN1_SQUARE_LAST_TOGGLE];

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
    // at this point the startTime of the last loop is still stored in messageOutBufferloopStartTime
    messageOutBuffer.lastLoopDuration = dutyCycleFinishTime - messageOutBuffer.loopStartTime;
    channels[LOOP_DURATION] = messageOutBuffer.lastLoopDuration;


    // now overwrite messageOutBuffer.loopStartTime with the actual loop start time
    messageOutBuffer.loopStartTime = dutyCycleTimer.read_us();

    // the dutyCycleTimer is not set to 0 on start of a loop, because the pc
    // needs an absolute time for sorting and plotting the sensor values
    // TODO - check what happens on timer overflow (after approx. 30min)

    // PWM output controlled by pc for little testing
    if (parameters[SLOW_PWM_ON] > 0.5f)
    {
        pwmGenerator.write(parameters[SLOW_PWM_PERCENT]);
    }
    else
    {
        pwmGenerator.write(0.0f);
    }


    // for debugging - store a controlledParameter value in fastParameters
    // to be able to plot the value on the pc
    channels[FAST_PWM_ON] = parameters[SLOW_PWM_ON];
    channels[FAST_PWM_PERCENT] = parameters[SLOW_PWM_PERCENT];


    // read all sensors
    aquireSensordata();

    // get a set point
    signalGeneratorOneParams.timeInS = ((float)messageOutBuffer.loopStartTime / 1000000.0f);
    setPoints[0] = generateSignal(&signalGeneratorOneParams);
    channels[SP_GEN_1_OUTPUT] = setPoints[0];

    // control a tank level or so
    channels[PID1_ERROR] = setPoints[0] - sensorValues[3];
    PID_Controller(&pidOneParameters);


    actuate();

    prepareMessage();
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
                        channels[ANALOG_IN_0] = sensorValues[i];
                        break;
                    case 1:
                        channels[ANALOG_IN_1] = sensorValues[i];
                        break;
                    case 2:
                        channels[ANALOG_IN_2] = sensorValues[i];
                        break;
                    case 3:
                        channels[ANALOG_IN_3] = sensorValues[i];
                        break;
                    case 4:
                        channels[ANALOG_IN_4] = sensorValues[i];
                        break;
                    case 5:
                        channels[ANALOG_IN_5] = sensorValues[i];
                        break;
                }
                break;

            // implement alternate measuring method here
            case 2:
                break;
        }
    }
}

void prepareMessage()
{
    // map all requested data to the outgoing message struct
    // loopStartTime and lastLoopDuration are allready set from within the loop

    // map channels
    int i;
    for (i = 0; i < REQUESTED_CHANNEL_COUNT; i++)
    {
        messageOutBuffer.channels[i] = channels[requestedChannels[i]];
    }

    // map rotating parameters
    // on each cycle, only one of the "controlled parameters" is send to the pc
    messageOutBuffer.parameterNumber = parameterSendCounter;
    //messageOutBuffer.parameterValue = parameters[requestedControlledParameters[parameterSendCounter]];
    messageOutBuffer.parameterValue = parameters[parameterSendCounter];

    // increment the counter for sending the "slow parameters"
    parameterSendCounter += 1;
    if (parameterSendCounter >= AVAILABLE_PARAMETER_COUNT)
    {
        parameterSendCounter = 0;
    }
}

void communicate()
{
    // for debugging
    blueLed = 0;
    redLed = 0;

    // send data to the pc via Ethernet with mbed os
    debugTimer.reset();
    debugTimer.start();
    sendBytesCount = socket.sendto(udp_server_address, (char *)&messageOutBuffer, sizeof(messageOutBuffer));
    channels[SEND_TIMER] = (float)debugTimer.read_us();

    #ifdef DEBUG
    pc.printf("SEND: %d %d %f\n", sendBytesCount, messageOutBuffer.parameterNumber, messageOutBuffer.parameterValue);
    #endif

    // receive a command
    debugTimer.reset();
    debugTimer.start();
    channels[RECEIVED_BYTES_COUNT] = (float)socket.recvfrom(&dump_address, (char *)&messageInBuffer, sizeof(messageInBuffer));
    channels[RECEIVE_TIMER] = (float)debugTimer.read_us();

    // if a command has been received
    if (channels[RECEIVED_BYTES_COUNT] > 0.5f)
    {
        parameters[messageInBuffer.parameterNumber] = messageInBuffer.value;

        // for debugging
        channels[LAST_COMMAND_ID] = messageInBuffer.parameterNumber;
        channels[LAST_COMMAND_VALUE] = messageInBuffer.value;
        // blue led blinkiblinki on successfull receive
        blueLed = 1;
    }
    else
    {
        // red led blinkiblinki on nothing received
        redLed = 1;
    }

    // for debugging
    channels[LAST_COMMAND_FROM_ARRAY_VALUE] = parameters[messageInBuffer.parameterNumber];

    #ifdef DEBUG
    pc.printf("RECV: %f %d %f\n", channels[RECEIVED_BYTES_COUNT], messageInBuffer.parameterNumber, messageInBuffer.value);
    #endif
}



// to be implemented
int actuate()
{
    //pwmGenerator.write(0.5f);
    return -1;
}
