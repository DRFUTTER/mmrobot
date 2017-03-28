#ifndef SMARTMOTOR_H
#define SMARTMOTOR_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

class SmartMotor
{
    private:
        static int cport_nr;    //Id of the comport
        static int count;       //Number of motors so far
        static int MaxAddress;  //Maximum address
        int address;            //Address of the motor
        int countsPerRev;       //Number of counts per revolution
        float velGain;          //Transformation from rpm to VT
        int lowLimit;
        int highLimit;
        long lastTicks;  //last encoder value
        long currTicks;  //current encoder value
    public:
        SmartMotor(int address, int countsPerRev);
        static int openPort(int serialPortID, int baudRate);    //When addressing the motors we give the serialPortId and BaudRate to open the serial port
        static int addressMotors(); 
        static int CheckMotorsAddress(); 
        int initialize(int lowLimit, int highLimit);
        void sendCommand(const char* command);
        static void sendCommandAll(const char* command);
        
        //Methods to set the different modes
        int setPositionMode(int accel, int vel);
        int setVelocityMode(int accel);
        int setTorqueMode();

        //Methods to send the parameters for each mode
        int sendPosition(long position);
        int sendVelocity(int rpm);
        int sendTorque(int torque);
        void stop();

        //methods to read data
        int readEncoders(long *newTicks);
};


#endif