#include <iostream>
#include <unistd.h>
#include "smartmotor.h"
#include "rs232.h"

using namespace std;

int main()
{
    SmartMotor m1(1, 4000, 8000, 2000.0); //Left wheel, Address = 1, CountsPerRev=4000, Sampling Rate=8000Hz
    
    //Open the comport
    cout << "Openning COM Port..." << endl;
    int portState = SmartMotor::openPort(16, 115200); //Serial Port ID (Port Id = 0 for ttyS0) and baud rate 115200
    if (portState != 0)
        return portState;
    cout << "COM Port successfully opened" << endl;

    cout << "Checking Motors Addresses..." << endl;
    //Need to address when the motors have been powered off and back on
    if (SmartMotor::CheckMotorsAddress() == 0)
    {
        cout << "Motors addresses sucessfully checked" << endl;
    }
    else
    {
        cout << "Motors addresses wrong" << endl;
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
    int dataMissed = 0;
    while(count<100)
    {
        m1.sendPosition(pos);
        if (m1.sendPosition(pos) != 0)
        {
            cout << "Error reading pos command" << endl;
            m1.stop();
            break;
        }
        usleep(50000);
        if (m1.updateTicks() != 0)
        {
            cout << "Warning: Data Missed" << endl;
            dataMissed++;
        }
        count++;
        cout << "Count: " << count << endl;
        cout << "Sent Pos: " << pos << endl;
        cout << "Current Ticks: " << m1.getCurrTicks() << endl;

        pos+=500;
    }
    usleep(10000); //Give some time for final read
    m1.updateTicks();
    cout << "Current Ticks: " << m1.getCurrTicks() << endl;*/
    //**********************************************//



    //*****************Velocity Mode****************//
    m1.setVelocityMode(5000);
    printf("Stopping the motor...\n");
    m1.sendVelocity(0);
    usleep(1000000);

    long ticks;
    long count = 1;
    int dataMissed = 0;
    while (count < 100)
    {
        if (m1.sendVelocity(20) != 0)  //Velocity in rev/sec
        {
            cout << "Error reading vel command" << endl;
            m1.stop();
            break;
        }
        usleep(20000); //Wait for velocity update
        if (m1.updateVel() != 0)
        {
            cout << "Warning: Data Missed" << endl;
            dataMissed++;
        }
        count++;
        //cout << "Count: " << count << endl;
        cout << "Current Velocity: " << m1.getVelocity() << endl;        
    }
    printf("Missed read data: %d\n",dataMissed);

    usleep(1000000);
    cout << "Last Velocity: " << m1.getVelocity() << endl;
    m1.sendVelocity(0);        
    //**********************************************//


    //Send any command
    //m1.sendCommand("RSAMP");

    usleep(500000);
    cout << "Closing COM Port..." << endl;
    RS232_CloseComport(16);
    cout << "COM Port closed." << endl;
    return 0;
}


