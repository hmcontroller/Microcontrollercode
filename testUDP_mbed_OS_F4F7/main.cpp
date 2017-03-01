#include "mbed.h"
#include "EthernetInterface.h"

#define SERVER_IP      "192.168.0.133"
#define OWN_IP         "192.168.0.10"
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
SocketAddress address_for_receive("192.168.0.133", 10000);
SocketAddress udp_server_address("192.168.0.133", 10000);


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
    printf("Starting...\n");


    sendBuffer.sendTimer = 0;
    sendBuffer.receiveTimer = 0;

    int i = 0;
    for (i = 0; i < 30; i++)
    {
        sendBuffer.data[i] = i;
    }

//    // with static ip
//    int ethInitReturnCode = eth.init(OWN_IP, NETMASK, GATEWAY);
//    if (ethInitReturnCode != 0)
//    {
//        pc.printf("Eth init error: %d \n", ethInitReturnCode);
//    }

    // Ethernetconnection with DHCP
    int ethConnectReturnCode = eth.connect();
    if (ethConnectReturnCode != 0)
    {
        pc.printf("Eth connect error: %d \n", ethConnectReturnCode);
    }
    const char *ip = eth.get_ip_address();
    const char *mac = eth.get_mac_address();

    pc.printf("IP address is: %s\n", ip ? ip : "No IP");
    pc.printf("MAC address is: %s\n", mac ? mac : "No MAC");


    // NEVER bind before port is open, NOTHING WORKS IN THAT CASE !!!!!!
    socket.open(&eth);
    socket.bind(PORT);
    socket.set_blocking(false);

    loopTimer.reset();
    loopTimer.start();


    while(1)
    {
        debugTimer.reset();
        debugTimer.start();
        sendBytesCount = socket.sendto(udp_server_address, (char *)&sendBuffer, sizeof(sendBuffer));
        sendBuffer.sendTimer = debugTimer.read_us();

        debugTimer.reset();
        debugTimer.start();
        int n = socket.recvfrom(&address_for_receive, (char *)&receiveBuffer, sizeof(receiveBuffer));
        sendBuffer.receiveTimer = debugTimer.read_us();

        if (n > 0)
        {
            blueLed = !blueLed;
        }

        if (loopCounter > 1000)
        {
            loopCounter = 0;
            printf("Received %d bytes from server: %d, %d\n", n, receiveBuffer.sendTimer, receiveBuffer.receiveTimer);
            greenLed = !greenLed;
        }

        loopCounter += 1;
        sendBuffer.loopTime = loopTimer.read_us();
        //wait(0.001);

    }
}
