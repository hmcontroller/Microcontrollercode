#include "mbed.h"
#include "USBHID.h"

//We declare a USBHID device. Input out output reports have a length of 8 bytes
USBHID hid(8, 8);

//This report will contain data to be sent
HID_REPORT send_report;
HID_REPORT recv_report;

Serial pc(USBTX, USBRX);
Timer debugTimer;

int main(void) {
    debugTimer.start();

    send_report.length = 8;

    while (1)
    {

        //Fill the report
        for (int i = 0; i < send_report.length; i++)
        {
            send_report.data[i] = rand() & 0xff;
        }

        //Send the report
        hid.send(&send_report);


        //try to read a msg
        if(hid.read(&recv_report))
        {
            pc.printf("received at %d, length of report %d: ", debugTimer.read_us(), recv_report.length);
            for(int i = 0; i < 8; i++)
            {
                pc.printf("%d ", recv_report.data[i]);
            }
            pc.printf("\r\n");
        }
        wait(0.2);
    }
}
