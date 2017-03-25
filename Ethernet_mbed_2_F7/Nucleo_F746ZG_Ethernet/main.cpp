#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"
#include "NTPClient.h"
#include <stdio.h>

EthernetInterface eth;

DigitalOut led1(LED1);

int main()
{
    NTPClient ntp;
    char buff[64];
    printf("\n\n*** Ethernet Demo uses NTP to set the clock ***\r\n");

    if(eth.init()!=0)                    //for DHCP Server
    {
        //if(eth.init(IP,MASK,GATEWAY)!=0) { //for Static IP Address
        printf("EthernetInterface Initialize Error \r\n");

        while (1)
        {
        }
    }
    if(eth.connect()!=0)
    {
        printf("EthernetInterface Connect Error \r\n");
        while (1)
        {
        }
    }
    printf("IP Address is %s\r\n", eth.getIPAddress());
    printf("NetMask is %s\r\n", eth.getNetworkMask());
    printf("Gateway Address is %s\r\n", eth.getGateway());
    printf("Ethernet Setup OK\r\n");
    printf("Getting time, 10s timeout. \r\n");
 
    if (ntp.setTime("0.uk.pool.ntp.org") == 0)
    {
        time_t ctTime;
        ctTime = time(NULL);
        printf("Time is set to : %s \r\n", ctime(&ctTime));
    }
    else
    {
        printf("Error getting time \r\n");
    }    

    printf("end of programm\r\n");
    while (true) {
        led1 = !led1;
        Thread::wait(500);
    }
}
