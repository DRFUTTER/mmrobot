#include "smartmotor.h"
#include "rs232.h"
#include <string>
#include <iostream>
#include <ostream>

int SmartMotor::count = 0;        //Initialize the count to 0
int SmartMotor::MaxAddress = 128; //Initialize the MaxAddress to 128
int SmartMotor::cport_nr = 0;     //Use ttyS0 as default
int addrSleepTime = 100 * 1000;   //100ms between SADDR=
int readSleepTime = 100 * 1000;   //100ms to wait for data
int commandSleepTime = 50000;     //10ms to wait after sending each command

char writeBuffer[40];         //Buffer to send data to serial port
unsigned char readBuffer[40]; //Buffer to receive data from serial port
char addrFlag[20];            //A comparisson char

//Constructor
SmartMotor::SmartMotor(int address, int countsPerRev)
{
    this->address = 128 + address;
    this->countsPerRev = countsPerRev;
    this->velGain = countsPerRev / 8000.0 * 65536.0 / 60.0; //Page 22 UserGuide v523
    this->lastTicks = 0;
    this->currTicks = 0;
    count++; //Add one motor to the count variable
    MaxAddress = 128 + count;
}

int SmartMotor::openPort(int serialPortID, int baudRate)
{
    //Store the port number
    SmartMotor::cport_nr = serialPortID;

    //Communication paramenters
    char mode[] = {'8', 'N', '1', 0};

    if (RS232_OpenComport(cport_nr, baudRate, mode))
    {
        printf("Can not open comport\n");
        return 0; //Communication error
    }
    RS232_flushRX(cport_nr);
    return 1;
}

//Address the available motors
int SmartMotor::addressMotors()
{
    //First check if there are any SmartMotor instance
    int count = SmartMotor::count;
    if (count != 0)
    {

        printf("Addressing %d motors.\n", count);
        for (int i = 1; i <= count; i++)
        {
            //(128+i)SADDR(i)\r
            sprintf(writeBuffer, "%cSADDR%d\r", 128, i);
            RS232_cputs(cport_nr, writeBuffer);
            printf("sent: %dSADDR%d\n", 0, i);
            usleep(addrSleepTime);

            //iECHO
            RS232_SendByte(cport_nr, 128 + i);
            RS232_cputs(cport_nr, "ECHO\r");
            printf("sent: %dECHO\n", i);
            usleep(addrSleepTime);

            //iSLEEP
            RS232_SendByte(cport_nr, 128 + i);
            RS232_cputs(cport_nr, "SLEEP\r");
            printf("sent: %dSLEEP\n", i);
            usleep(addrSleepTime);
        }
        //0WAKE
        RS232_SendByte(cport_nr, 128);
        RS232_cputs(cport_nr, "WAKE\r");
        printf("sent: 0WAKE\n");
        usleep(addrSleepTime);

        //Check the motors were successfully addressed by asking the address
        RS232_flushRX(cport_nr);
        sprintf(writeBuffer, "RADDR\r");
        RS232_cputs(cport_nr, writeBuffer);
        printf("sent: RADDR\n");
        usleep(addrSleepTime);

        sprintf(addrFlag, "RADDR\r1\r"); //RADR1 checks all the motors have been addressed

        //Read the data
        int nBytes = RS232_PollComport(cport_nr, readBuffer, 50);
        if (nBytes > 0)
        {
            readBuffer[nBytes] = 0; //put null at the end of the string
        }
        else
        {
            printf("No data received from motors.\n");
        }

        //Check byte by byte the contents of the read data
        for (int i = 0; i <= nBytes; i++)
        {
            printf("%d %d\n", addrFlag[i], (char)readBuffer[i]);
        }

        if (strcmp(addrFlag, (char *)readBuffer) == 0)
        {
            return 1; //Successfully addressed
        }
        else
        {
            return 0; //Error during addressing
        }
    }
    else
    {
        printf("There are no motors assigned.\n");
    }
}

int SmartMotor::CheckMotorsAddress()
{
    //Check the motors were successfully addressed by asking the address
    sprintf(writeBuffer, "RADDR\r");
    RS232_cputs(cport_nr, writeBuffer);
    printf("sent: RADDR\n");
    usleep(addrSleepTime);

    sprintf(addrFlag, "RADDR\r1\r"); //RADR1 checks all the motors have been addressed

    //Read the data
    int nBytes = RS232_PollComport(cport_nr, readBuffer, 50);
    if (nBytes > 0)
    {
        readBuffer[nBytes] = 0; //put null at the end of the string
    }
    else
    {
        printf("No data received from motors.\n");
    }

    //Check byte by byte the contents of the read data
    for (int i = 0; i <= nBytes; i++)
    {
        printf("%d %d\n", addrFlag[i], (char)readBuffer[i]);
    }

    if (strcmp(addrFlag, (char *)readBuffer) == 0)
    {
        return 1; //Successfully addressed
    }
    else
    {
        return 0; //Error during addressing
    }
}

//Initialize the motor and clear error flags
int SmartMotor::initialize(int lowLimit, int highLimit)
{
    //Erase the write buffer
    memset(&writeBuffer[0], 0, sizeof writeBuffer);
    //Erase the read buffer
    memset(&readBuffer[0], 0, sizeof readBuffer);

    //int cport_nr = this->cport_nr;
    //Disable the low and high limits
    //!!!!!!!!!!!!!!!Check the limits for position mode!!!!!!!!!!!!!!!!!!! (TO DO TASK)
    //disable limit & reset errors
    printf("Initialazing motor %d...\n", this->address);

    RS232_SendByte(cport_nr, this->address);
    sprintf(writeBuffer, "EIGN(2) EIGN(3) ZS\r");
    RS232_cputs(cport_nr, writeBuffer);

    //RS232_cputs(cport_nr, "EIGN(2) EIGN(3) ZS\r");
    printf("sent: %d%s\n", this->address, writeBuffer);
    
    usleep(1000000); //Wait one second to initialize
    
    //Read back the command because of ECHO, instead of waiting some time
    int len = static_cast<int>(strlen(writeBuffer)) + 1;

    printf("Length: %d\n", len);

    while (RS232_availableBytes(cport_nr) < len)
    {

    }
    RS232_PollComport(cport_nr, readBuffer, len);
    printf("%s\n", readBuffer);
}

//Set position mode
void SmartMotor::setPositionMode(int accel, int vel)
{
    //Erase the write buffer
    memset(&writeBuffer[0], 0, sizeof writeBuffer);
    //Erase the read buffer
    memset(&readBuffer[0], 0, sizeof readBuffer);

    //Send the data
    RS232_SendByte(cport_nr, this->address);
    sprintf(writeBuffer, "MP ADT=%d VT=%d G\r",accel, vel);
    RS232_cputs(cport_nr, writeBuffer);
    printf("sent: %d%s\n", this->address, writeBuffer);

    //Read back the command because of ECHO, instead of waiting some time
    usleep(1000000); //Wait for an answer
    
    
    //Read back the command because of ECHO, instead of waiting some time
    int len = static_cast<int>(strlen(writeBuffer)) + 1;
    printf("Length: %d\n", len);
    while (RS232_availableBytes(cport_nr) < len)
    {

    }
    RS232_PollComport(cport_nr, readBuffer, len);
    printf("%s\n", readBuffer);
}

//Send the position
int SmartMotor::sendPosition(long position)
{
    char wrBuffer[40];         //Buffer to send data to serial port
    unsigned char rdBuffer[40]; //Buffer to receive data from serial port

    RS232_SendByte(cport_nr, this->address);
    sprintf(wrBuffer, "PT=%ld G\r", position);
    RS232_cputs(cport_nr, wrBuffer);

    int len = static_cast<int>(strlen(wrBuffer)) + 1;
    printf("Length: %d\n", len);
    printf("sent: %d%s\n", this->address, wrBuffer);

    usleep(commandSleepTime); //Give some time to answer
    //Read the ECHO data
    int checkCount = 0;
    while (RS232_availableBytes(cport_nr) < len)
    {
        usleep(500);
        checkCount++;
        if(checkCount>30)
            break;
    }
    RS232_PollComport(cport_nr, rdBuffer, len);
    printf("%s\n", rdBuffer);
    return 0;
}

//Set velocity mode
void SmartMotor::setVelocityMode(int accel)
{
    //Erase the write buffer
    memset(&writeBuffer[0], 0, sizeof writeBuffer);
    //Erase the read buffer
    memset(&readBuffer[0], 0, sizeof readBuffer);

    //Send the data
    RS232_SendByte(cport_nr, this->address);
    sprintf(writeBuffer, "MV ADT=%d G\r",accel);
    RS232_cputs(cport_nr, writeBuffer);
        
    //Read back the command because of ECHO, instead of waiting some time
    usleep(1000000); //Wait for an answer

    printf("sent: %d%s\n", this->address, writeBuffer);
    
    //Read back the command because of ECHO, instead of waiting some time
    int len = static_cast<int>(strlen(writeBuffer)) + 1;

    printf("Length: %d\n", len);

    while (RS232_availableBytes(cport_nr) < len)
    {

    }
    RS232_PollComport(cport_nr, readBuffer, len);
    printf("%s\n", readBuffer);
}

//Send the velocity
int SmartMotor::sendVelocity(int rpm)
{
    //Erase the write buffer
    memset(&writeBuffer[0], 0, sizeof writeBuffer);
    //Erase the read buffer
    memset(&readBuffer[0], 0, sizeof readBuffer);

    //RS232_flushRX(cport_nr);
    int vt = (int)(rpm * this->velGain);
    RS232_SendByte(cport_nr, this->address);
    sprintf(writeBuffer, "VT=%d G\r", vt);
    RS232_cputs(cport_nr, writeBuffer);

    //size_t len = strlen(writeBuffer);
    int len = static_cast<int>(strlen(writeBuffer)) + 1;

    printf("Length: %d\n", len);
    //usleep(commandSleepTime); //Give some time to answer

    int countRead = 0;
    unsigned char c = 0;

    //Read the ECHO data
    while (RS232_availableBytes(cport_nr) < len)
    {

    }
    RS232_PollComport(cport_nr, readBuffer, len);
    printf("sent: VT=%d G\n", vt);
    printf("%s\n", readBuffer);
    return 0;
}

//Set torque mode
void SmartMotor::setTorqueMode()
{
    //TO DO TASK

    /*RS232_SendByte(cport_nr, this->address);    
    sprintf(writeBuffer, "MT G\r");
    RS232_cputs(cport_nr, writeBuffer);
  	usleep(commandSleepTime);*/
}

//Send the torque
int SmartMotor::sendTorque(int torque)
{
    //TO DO TASK

    /*RS232_SendByte(cport_nr, 128 + this->address);
    sprintf(writeBuffer, "T=%d G\r", torque);
    RS232_cputs(cport_nr, writeBuffer);
  	usleep(commandSleepTime);*/
}

void SmartMotor::stop()
{
    RS232_SendByte(cport_nr, 128);
    sprintf(writeBuffer, "OFF\r");
    RS232_cputs(cport_nr, writeBuffer);
    usleep(commandSleepTime); //Give some time to answer
    //Read back the command because of ECHO, instead of waiting some time
    
}

void SmartMotor::sendCommandAll(const char *command)
{
    //Flush the RX buffer
    RS232_flushRX(cport_nr);

    RS232_SendByte(cport_nr, 128);
    //Check the motors were successfully addressed by asking the address
    sprintf(writeBuffer, "%s ", command);
    RS232_cputs(cport_nr, writeBuffer);
    printf("sent: %s\n", command);

    //Read back the command because of ECHO, instead of waiting some time
    //RS232_ReadUntil(cport_nr, readBuffer, '\r');

    //Read the data
    int nBytes = RS232_PollComport(cport_nr, readBuffer, 50);
    if (nBytes > 0)
    {
        readBuffer[nBytes] = 0; //put null at the end of the string
    }
    else
    {
        printf("No data received from motors.\n");
    }

    //Check byte by byte the contents of the read data
    for (int i = 0; i <= nBytes; i++)
    {
        printf("%d ", readBuffer[i]);
    }
    printf("\n");
}

void SmartMotor::sendCommand(const char *command)
{
    RS232_SendByte(cport_nr, 128 + this->address);
    sprintf(writeBuffer, "%s%c", command, 32);
    RS232_cputs(cport_nr, writeBuffer);
    printf("sent: %d%s\n", 128 + this->address, command);
    usleep(commandSleepTime); //Give some time to answer
}

char number[11];
int SmartMotor::readEncoders(long *newTicks)
{
    //RS232_flushRX(cport_nr);
    usleep(2000);
    this->sendCommand("RPA");
    unsigned char c = 0;
    memset(&readBuffer[0], 0, sizeof readBuffer);
    int readBytes;
    int compFlag = -1;
    int address = 0;
    int countRead = 0;

    /*
    //Read the address
    while (RS232_availableBytes(cport_nr) < 3)
    {
    }
    while (!(c >= 128 && c <= this->MaxAddress))
    {
        if (RS232_PollComport(cport_nr, &c, 1) == 0)
        {
            usleep(1000);
            countRead++;
            if (countRead > 5) //In case it gets stuck reading
            {
                RS232_flushRX(cport_nr);
                return 1; //Error reading
            }
        }
        else
        {
            if (c == 0)
            {
                RS232_flushRX(cport_nr);
                return 1; //Error reading
            }
        }
        printf("%d ", c);
    }
    printf("\n");
    countRead = 0;
    usleep(1000);

    address = c - 128;
    readBytes = RS232_ReadUntil(cport_nr, readBuffer, '\r');
    printf("RAP_Read: %s\n", readBuffer);

    //Check if the receive data is the RPA command answer
    compFlag = strncmp("RPA ", (char *)readBuffer, 4);
    if (compFlag == 0)
    {
    }
    else //If not read again until next terminator
    {
        while (RS232_availableBytes(cport_nr) < 1)
        {
            sleep(110);
            printf("Stuck here...2\n");
        }
        countRead = 0;
        while (!(c >= 128 && c <= this->MaxAddress))
        {
            RS232_PollComport(cport_nr, &c, 1);
            usleep(110);
            countRead++;
            if (countRead > 20) //In case it gets stuck reading
            {
                return 1; //Error reading
            }
        }
        address = c - 128;
        memset(&readBuffer[0], 0, sizeof readBuffer);
        readBytes = RS232_ReadUntil(cport_nr, readBuffer, '\r');

        //Check if the receive data is the RPA command answer
        compFlag = strncmp("RPA", (char *)readBuffer, 3);
    }
    
    if (compFlag == 0)
    {
        int n = 4;
        memset(&number[0], 0, sizeof number);
        while (c != '\r')
        {
            c = readBuffer[n];
            if (c != '\r')
            {
                number[n - 4] = (char)c;
                n++;
            }
        }
        number[n + 1] = 0;
        *newTicks = atol(number);
        printf("Number: %ld\n", *newTicks);
        return 0;
    }
    else
    {
        return 2;
    }*/

    
    printf("\n");
}

/*
unsigned char c = 1;
    int n = 0;

    int readBytes;
    int readFlag = 1;
    int countDel = 0;
    //Read the available data from the serial port
    while (readFlag)
    {
        readBytes = RS232_PollComport(cport_nr, &c, 1);
        if (readBytes != 0)
        {
            printf("%d %c\n", c, c);
            //readBuffer[n] = c;
            n++;
            if (c == '\r')
            {
                countDel++;
            }
            if (countDel == 2)
            {
                readFlag = 0;
            }
        }
        else
        {
            //usleep(100);
        }
    }
    printf("\n");
*/