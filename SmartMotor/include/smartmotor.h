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
    int res;                //Number of counts per revolution
    int samp;               //Sampling rate in Hz
    int maxVT;     //Maximum value for VT
    float velGain;          //Transformation from rpm to VT
    int lowLimit;
    int highLimit;
    long lastTicks;         //last encoder value
    long currTicks;         //current encoder value
    float vel;              //Current velocity in rev/sec
  public:
    SmartMotor(int address, int resolution, int samplingRate, float maxRevPerMin);
    long getLastTicks()
    {
        return lastTicks;
    }
    long getCurrTicks()
    {
        return currTicks;
    }
    float getVelocity()
    {
        return vel;
    }
    static int openPort(int serialPortID, int baudRate); //When addressing the motors we give the serialPortId and BaudRate to open the serial port
    static int addressMotors();
    static int CheckMotorsAddress();
    static int sendCommandAll(const char *command); //Send a command to all the motors
    int initialize(int lowLimit, int highLimit);    //Initialize the motors with low and high limits

    //Methods to set the different modes
    int setPositionMode(int accel, int vel);
    int setVelocityMode(int accel);
    int setTorqueMode();

    //Methods to send data for each mode
    int sendPosition(long position);
    int sendVelocity(int revPerSec);
    int sendTorque(int torque);
    int stop();

    //methods to read data
    int sendCommand(const char*);
    int updateTicks();    
    int updateVel();    

  protected:
    int readECHO(int len); //Read the ECHO data from the Serial Port
};

#endif