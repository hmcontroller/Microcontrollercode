#include "config.h"

//#include "UDPSocket.h"
//#include "lwip-interface/EthernetInterface.h"
#include "EthernetInterface.h"
#include "SocketAddress.h"

#define SERVER_IP       "192.168.0.133"
#define OWN_IP          "192.168.0.15"
#define PORT            10000
#define NET_MASK        "255.255.0.0"
#define GATEWAY         "192.168.0.1"

EthernetInterface eth;
UDPSocket socket;
SocketAddress udp_server_address(SERVER_IP, PORT);
SocketAddress dump_address(SERVER_IP, PORT);

Timer dutyCycleTimer;
Timer debugTimer;
Serial pcSerialPort(USBTX, USBRX);

DigitalOut greenLed(LED1);
DigitalOut blueLed(LED2);
DigitalOut redLed(LED3);

// storage for channels
float channels[AVAILABLE_CHANNEL_COUNT] = {0.0f};

// storage for parameters, that could be set from the pc
float parameters[AVAILABLE_PARAMETER_COUNT] = {0.0f};

int parameterSendCounter = 0;
int receivedBytesCount = 0;
int sendBytesCount = 0;

MessageOut messageOutBuffer;
MessageIn messageInBuffer;


// Channel values that will be send to the pc at every loop cycle
int requestedChannels[REQUESTED_CHANNEL_COUNT] = {
    0,
    1,
    2,
    3,
    4,
    5,
    6,
    7,
    8
};

// Parameters that can be set from the pc.
int requestedParameters[REQUESTED_PARAMETER_COUNT] = {
    0,
    1,
    2,
    3,
    4,
    5,
    6,
    7,
    8,
    9,
    10,
    11,
    12,
    13,
    14,
    15,
    16,
    17,
    18,
    19,
    20,
    21,
    22,
    23,
    24,
    25,
    26,
    27,
    28,
    29,
    30,
    31,
    32,
    33,
    34,
    35,
    36,
    37,
    38,
    39,
    40,
    41,
    42,
    43,
    44,
    45,
    46,
    47,
    48,
    49,
    50,
    51,
    52,
    53,
    54,
    55
};


void initControlDemonstrator()
{

    // Bring up the network interface

    // laut Dokumentation Initialisierung mit static IP eigentlich möglich bei mbed OS, geht aber nicht
    // es gibt wohl einen Branch von mbed mit einem Patch
    //// eth.init(&ownIP, &mask, &Gateway);

    // Ethernetconnection with DHCP
    eth.set_network(OWN_IP, NET_MASK, GATEWAY);
    eth.connect();
    const char *ip = eth.get_ip_address();
    const char *mac = eth.get_mac_address();

    pcSerialPort.printf("IP address is: %s\n", ip ? ip : "No IP");
    pcSerialPort.printf("MAC address is: %s\n", mac ? mac : "No MAC");

    socket.open(&eth);
    socket.bind(PORT);
    socket.set_blocking(false);

    // the timer is used for duty cycle duration measurement
    dutyCycleTimer.reset();
    dutyCycleTimer.start();

}

void prepareMessage()
{
    greenLed = 1;
    blueLed = 0;
    redLed = 0;

    // map all requested data to the outgoing message struct
    // loopStartTime and lastLoopDuration are allready set from within the loop

    // map channels
    int i;
    for (i = 0; i < REQUESTED_CHANNEL_COUNT; i++)
    {
        messageOutBuffer.channels[i] = channels[requestedChannels[i]];
    }

//    // Calculate duration of last duty cycle
//    // at this point the startTime of the last loop is still stored in messageOutBufferloopStartTime
//    messageOutBuffer.lastLoopDuration = dutyCycleFinishTime - messageOutBuffer.loopStartTime;
//    LOOP_DURATION = messageOutBuffer.lastLoopDuration;


    // now overwrite messageOutBuffer.loopStartTime with the actual loop start time
    messageOutBuffer.loopStartTime = dutyCycleTimer.read_us();

    // the dutyCycleTimer is not set to 0 on start of a loop, because the pc
    // needs an absolute time for sorting and plotting the sensor values
    // TODO - check what happens on timer overflow (after approx. 30min)




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
    prepareMessage();

    // for debugging
    blueLed = 0;
    redLed = 0;

    sendMessage();
    receiveMessage();
}

void sendMessage()
{
    // send data to the pc via Ethernet with mbed os
    debugTimer.reset();
    debugTimer.start();
    sendBytesCount = socket.sendto(udp_server_address, (char *)&messageOutBuffer, sizeof(messageOutBuffer));
    SEND_TIMER = (float)debugTimer.read_us();

    #ifdef DEBUG
    pcSerialPort.printf("SEND: %d %d %f\n", sendBytesCount, messageOutBuffer.parameterNumber, messageOutBuffer.parameterValue);
    #endif

}

void receiveMessage()
{
    // receive a command
    debugTimer.reset();
    debugTimer.start();
    RECEIVED_BYTES_COUNT = (float)socket.recvfrom(&dump_address, (char *)&messageInBuffer, sizeof(messageInBuffer));
    RECEIVE_TIMER = (float)debugTimer.read_us();
    // if a command has been received
    if (RECEIVED_BYTES_COUNT > 0.5f)
    {
        parameters[messageInBuffer.parameterNumber] = messageInBuffer.value;

        // for debugging
        LAST_COMMAND_ID = messageInBuffer.parameterNumber;
        LAST_COMMAND_VALUE = messageInBuffer.value;
        // blue led blinkiblinki on successfull receive
        blueLed = 1;
    }
    else
    {
        // red led on until next loop cycle
        redLed = 1;
    }

    // for debugging
    LAST_COMMAND_FROM_ARRAY_VALUE = parameters[messageInBuffer.parameterNumber];

    #ifdef DEBUG
    pcSerialPort.printf("RECV: %f %d %f\n", RECEIVED_BYTES_COUNT, messageInBuffer.parameterNumber, messageInBuffer.value);
    #endif

}
