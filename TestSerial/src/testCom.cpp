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

    m1.initialize(0, 0);

    //*****************Position Mode****************//
    /*m1.setPositionMode(500,400000);
    printf("Returning to 0...\n");
    m1.sendPosition(0);
    usleep(5000000);
    long count = 1;
    long pos = 8000;
    while(count<100)
    {
        m1.sendPosition(pos);
        pos+=500;
        count++;
        printf("%ld\n",count);
        usleep(1000);
    }*/
    //**********************************************//

    //*****************Velocity Mode****************//
    m1.setVelocityMode(500);
    printf("Stopping the motor...\n");
    m1.sendVelocity(0);
    usleep(1000000);

    long ticks;
    long count = 1;
    int dataMissed = 0;
    while (count < 100)
    {
        if (m1.sendVelocity(30) != 0)
        {
            printf("Error reading vel command\n");
            m1.stop();
            break;
        }
        if (m1.readEncoders(&ticks) != 0)
        {
            printf("Data missed\n");
            dataMissed++;
        }

        count++;
        printf("Count: %ld\n", count);
    }
    //**********************************************//
    printf("Missed read data: %d\n",dataMissed);

    m1.sendVelocity(0);
    usleep(500000);
    RS232_CloseComport(16);
    return 0;
}
