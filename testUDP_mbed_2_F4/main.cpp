#include "mbed.h"
#include "EthernetInterface.h"

#define SERVER_IP      "192.168.0.133"
#define OWN_IP         "192.168.0.12"
#define PORT           10000
#define NETMASK        "255.255.0.0"
#define GATEWAY        "192.168.0.1"

Serial pc(USBTX, USBRX);
Timer debugTimer;
Timer loopTimer;
DigitalOut greenLed(LED1);
DigitalOut blueLed(LED2);
DigitalOut redLed(LED3);

EthernetInterface eth;
UDPSocket socket;
Endpoint udp_server_address;
Endpoint dump_address;

int sendBytesCount = 0;
int loopCounter = 0;

struct Message
{
    uint32_t loopTime;
    uint32_t sendTimer;
    uint32_t receiveTimer;
    float data[30];
};

volatile struct Message sendBuffer;
volatile struct Message receiveBuffer;


int main() {
    pc.printf("Starting...\n");

    sendBuffer.sendTimer = 0;
    sendBuffer.receiveTimer = 0;

    int i = 0;
    for (i = 0; i < 30; i++)
    {
        sendBuffer.data[i] = i;
    }


// eth.connect() funktioniert nicht !!!!!!!!!!!!!!!!!!!!!!!!!!! Error code -1

    int ethInitReturnCode = eth.init(OWN_IP, "255.255.255.0", "192.168.178.1");
    // DHCP geht irgendwie nicht mit mbed 2 und F4 ????????????
    // es gibt zwar keinen Errorcode, aber auch keine IP
    //int ethInitReturnCode = eth.init();

    int ethConnectReturnCode = eth.connect();

    if (ethInitReturnCode != 0)
    {
        pc.printf("Eth init error: %d \n", ethInitReturnCode);
    }

    if (ethConnectReturnCode != 0)
    {
        pc.printf("Eth connect error: %d \n", ethConnectReturnCode);
    }

    pc.printf("Client IP Address is %s \n", eth.getIPAddress());
    pc.printf("Client MAC Address is %s \n", eth.getMACAddress());



    socket.init();
    socket.bind(PORT);
    socket.set_blocking(false, 0);

    udp_server_address.set_address(SERVER_IP, PORT);

    loopTimer.reset();
    loopTimer.start();

    while(1)
    {
        debugTimer.reset();
        debugTimer.start();
        sendBytesCount = socket.sendTo(udp_server_address, (char *)&sendBuffer, sizeof(sendBuffer));
        sendBuffer.sendTimer = debugTimer.read_us();

        debugTimer.reset();
        debugTimer.start();
        int n = socket.receiveFrom(dump_address, (char *)&receiveBuffer, sizeof(receiveBuffer));
        sendBuffer.receiveTimer = debugTimer.read_us();

        if (n > 0)
        {
            blueLed = !blueLed;
        }

        if (loopCounter > 1000)
        {
            loopCounter = 0;
            pc.printf("Received %d bytes from server: %d, %d\n", n, receiveBuffer.sendTimer, receiveBuffer.receiveTimer);
            greenLed = !greenLed;
        }

        loopCounter += 1;
        sendBuffer.loopTime = loopTimer.read_us();
        //wait(0.001);
    }
}
