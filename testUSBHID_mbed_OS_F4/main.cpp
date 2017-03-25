#include "mbed.h"
#include "USBHID.h"

//We declare a USBHID device. Input out output reports have a length of 8 bytes
USBHID hid(64, 64);

//This report will contain data to be sent
HID_REPORT send_report;
HID_REPORT recv_report;

Serial pc(USBTX, USBRX);
Timer debugTimer;

int main(void) {
    debugTimer.start();

    send_report.length = 64;
    recv_report.length = 64;

    //Fill an initial report
    for (int i = 0; i < send_report.length; i++)
    {
        send_report.data[i] = rand() & 0xff;
    }

    while (1)
    {


        //Send the report.
        hid.send(&send_report);


        //try to read a msg
        bool readSuccess = hid.read(&recv_report);

        if(readSuccess == true)
        {
            pc.printf("received at %d, length of report %d: ", debugTimer.read_us(), recv_report.length);
            for(int i = 0; i < recv_report.length; i++)
            {
                pc.printf("%d ", recv_report.data[i]);
            }
            pc.printf("\r\n");
        }
        wait(0.2);
    }
}
