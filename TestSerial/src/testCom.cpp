#include <iostream>
#include <unistd.h>
#include "smartmotor.h"
#include "rs232.h"

using namespace std;

int main()
{
    SmartMotor m1(1, 4000); //Left wheel, Address = 1, CountsPerRev=4000
    
    //Open the comport
    int portState = SmartMotor::openPort(16, 115200); //Serial Port ID (Port Id = 0 for ttyS0) and baud rate 115200
    if (portState != 1)
        return portState;

    //Need to address when the motors have been powered off and back on
    if (SmartMotor::CheckMotorsAddress())
    {
        printf("Motors successfully addressed.\n");
    }
    else
    {
        printf("Fail to address motors.\n");
        return 1;
    }

    /*char writeBuffer[25];         //Buffer to send data to serial port
    unsigned char readBuffer[25]; //Buffer to receive data from serial port
    int cport_nr = 16;
    int readData = 0;*/

    m1.initialize(0, 0);
    m1.setPositionMode(500,400000);
    
    //m1.setVelocityMode(500);
    //m1.sendVelocity(0);

    //m1.sendPosition(0);

    long count = 8000;
    while(count<8000*20)
    {
        m1.sendPosition(count);
        count=count+500;
        printf("%ld\n",count);
        usleep(100000);
    }
    
    usleep(500000);  
    RS232_CloseComport(16);
    return 0;
}
