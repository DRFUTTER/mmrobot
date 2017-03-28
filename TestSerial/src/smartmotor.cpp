#include "smartmotor.h"
#include "rs232.h"
#include <string>
#include <iostream>
#include <ostream>

int SmartMotor::count = 0;        //Initialize the count to 0
int SmartMotor::MaxAddress = 128; //Initialize the MaxAddress to 128
int SmartMotor::cport_nr = 0;     //Use ttyS0 as default
int addrSleepTime = 100 * 1000;   //100ms
int waitSleepTime = 100 * 1000;   //100ms to wait for data
int commandSleepTime = 1800;      //1.8ms to wait after sending each command

char writeBuffer[40];         //Buffer to send data to serial port
unsigned char readBuffer[40]; //Buffer to receive data from serial port
char addrFlag[20];            //A comparisson char

int readBytes;

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
        readBytes = RS232_PollComport(cport_nr, readBuffer, 50);
        if (readBytes > 0)
        {
            readBuffer[readBytes] = 0; //put null at the end of the string
        }
        else
        {
            printf("No data received from motors.\n");
        }

        //Check byte by byte the contents of the read data
        for (int i = 0; i <= readBytes; i++)
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
    int readBytes = RS232_PollComport(cport_nr, readBuffer, 50);
    if (readBytes > 0)
    {
        readBuffer[readBytes] = 0; //put null at the end of the string
    }
    else
    {
        printf("No data received from motors.\n");
    }

    //Check byte by byte the contents of the read data
    for (int i = 0; i <= readBytes; i++)
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
    char wrBuffer[25];          //Buffer to send data to serial port
    unsigned char rdBuffer[25]; //Buffer to receive data from serial port
    //Clear data from the arrays
    for (int i = 0; i < 25; i++)
    {
        wrBuffer[i] = 0;
    }
    for (int i = 0; i < 25; i++)
    {
        rdBuffer[i] = 0;
    }

    //Disable the low and high limits
    //!!!!!!!!!!!!!!!Check the limits for position mode!!!!!!!!!!!!!!!!!!! (TO DO TASK)
    //disable limit & reset errors
    printf("Initialazing motor %d...\n", this->address - 128);

    RS232_SendByte(cport_nr, this->address);
    sprintf(wrBuffer, "EIGN(2) EIGN(3) ZS\r");
    RS232_cputs(cport_nr, wrBuffer);

    //RS232_cputs(cport_nr, "EIGN(2) EIGN(3) ZS\r");
    printf("sent: %d%s\n", this->address, wrBuffer);

    //Read the ECHO data
    usleep(1000000); //Wait one second to initialize
    int len = static_cast<int>(strlen(wrBuffer)) + 1;

    printf("Length: %d\n", len);

    while (RS232_availableBytes(cport_nr) < len)
    {
    }
    RS232_PollComport(cport_nr, rdBuffer, len);
    printf("%s\n", rdBuffer);
}

//Set position mode
int SmartMotor::setPositionMode(int accel, int vel)
{
    char wrBuffer[20];          //Buffer to send data to serial port
    unsigned char rdBuffer[20]; //Buffer to receive data from serial port

    //Clear data from the arrays
    for (int i = 0; i < 20; i++)
    {
        wrBuffer[i] = 0;
    }
    for (int i = 0; i < 20; i++)
    {
        rdBuffer[i] = 0;
    }

    //Send the data
    RS232_SendByte(cport_nr, this->address);
    sprintf(wrBuffer, "MP ADT=%d VT=%d G\r", accel, vel);
    RS232_cputs(cport_nr, wrBuffer);

    //Get the size of the write buffer
    int len = static_cast<int>(strlen(wrBuffer)) + 1;
    printf("Length: %d\n", len);
    printf("sent: %d%s\n", this->address, wrBuffer);

    //Read the ECHO data
    usleep(waitSleepTime); //Wait some time for the answer
    int checkCount = 0;
    while (RS232_availableBytes(cport_nr) < len)
    {
        usleep(500);
        checkCount++;
        if (checkCount > 30)
            break;
    }
    RS232_PollComport(cport_nr, rdBuffer, len);
    printf("%s\n", rdBuffer);
    return 0;
}

//Send the position
int SmartMotor::sendPosition(long position)
{
    char wrBuffer[20];          //Buffer to send data to serial port
    unsigned char rdBuffer[20]; //Buffer to receive data from serial port
    //Clear data from the arrays
    for (int i = 0; i < 20; i++)
    {
        wrBuffer[i] = 0;
    }
    for (int i = 0; i < 20; i++)
    {
        rdBuffer[i] = 0;
    }

    RS232_SendByte(cport_nr, this->address);
    sprintf(wrBuffer, "PT=%ld G\r", position);
    RS232_cputs(cport_nr, wrBuffer);

    //Get the size of the write buffer
    int len = static_cast<int>(strlen(wrBuffer)) + 1;
    printf("Length: %d\n", len);
    printf("sent: %d%s\n", this->address, wrBuffer);

    //Read the ECHO data
    usleep(commandSleepTime); //Give some time to answer
    int checkCount = 0;
    while (RS232_availableBytes(cport_nr) < len)
    {
        usleep(500);
        checkCount++;
        if (checkCount > 30)
            break;
    }
    readBytes = RS232_PollComport(cport_nr, rdBuffer, len);
    if (readBytes < len) //If the data was not read correctly
    {
        return 1;
    }
    printf("%s\n", rdBuffer);
    return 0;
}

//Set velocity mode
int SmartMotor::setVelocityMode(int accel)
{
    char wrBuffer[20];          //Buffer to send data to serial port
    unsigned char rdBuffer[20]; //Buffer to receive data from serial port

    //Clear data from the arrays
    for (int i = 0; i < 20; i++)
    {
        wrBuffer[i] = 0;
    }
    for (int i = 0; i < 20; i++)
    {
        rdBuffer[i] = 0;
    }

    //Send the data
    RS232_SendByte(cport_nr, this->address);
    sprintf(wrBuffer, "MV ADT=%d G\r", accel);
    RS232_cputs(cport_nr, wrBuffer);

    //Get the size of the write buffer
    int len = static_cast<int>(strlen(wrBuffer)) + 1;
    printf("Length: %d\n", len);
    printf("sent: %d%s\n", this->address, wrBuffer);

    //Read back the command because of ECHO, instead of waiting some time
    usleep(waitSleepTime); //Wait for an answer
    int checkCount = 0;
    while (RS232_availableBytes(cport_nr) < len)
    {
        usleep(100);
        checkCount++;
        if (checkCount > 30)
            break;
    }
    usleep(500);
    readBytes = RS232_PollComport(cport_nr, rdBuffer, len);
    if (readBytes < len) //If the data was not read correctly
    {
        return 1;
    }
    printf("%s\n", rdBuffer);
    return 0;
}

//Send the velocity
int SmartMotor::sendVelocity(int rpm)
{
    char wrBuffer[20];          //Buffer to send data to serial port
    unsigned char rdBuffer[20]; //Buffer to receive data from serial port
    //Clear data from the arrays
    for (int i = 0; i < 20; i++)
    {
        wrBuffer[i] = 0;
    }
    for (int i = 0; i < 20; i++)
    {
        wrBuffer[i] = 0;
    }
    for (int i = 0; i < 20; i++)
    {
        rdBuffer[i] = 0;
    }

    int vt = (int)(rpm * this->velGain);
    RS232_SendByte(cport_nr, this->address);
    sprintf(wrBuffer, "VT=%d G\r", vt);
    RS232_cputs(cport_nr, wrBuffer);

    int len = static_cast<int>(strlen(wrBuffer)) + 1;
    printf("Length: %d\n", len);
    printf("sent: %d%s\n", this->address, wrBuffer);

    //Read the ECHO data
    usleep(commandSleepTime); //Give some time to answer
    int checkCount = 0;
    while (RS232_availableBytes(cport_nr) < len)
    {
        usleep(100);
        checkCount++;
        if (checkCount > 20)
            break;
    }
    usleep(500); //Small delay before reading
    readBytes = RS232_PollComport(cport_nr, rdBuffer, len);
    if (readBytes < len) //If the data was not read correctly
    {
        return 1;
    }
    printf("%s\n", rdBuffer);
    return 0;
}

//Set torque mode
int SmartMotor::setTorqueMode()
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
    char wrBuffer[20];          //Buffer to send data to serial port
    unsigned char rdBuffer[25]; //Buffer to receive data from serial port
    char data[25];

    //Clear data from the arrays
    for (int i = 0; i < 20; i++)
    {
        wrBuffer[i] = 0;
    }
    for (int i = 0; i < 25; i++)
    {
        rdBuffer[i] = 0;
    }
    for (int i = 0; i < 25; i++)
    {
        data[i] = 0;
    }

    RS232_SendByte(cport_nr, this->address);
    sprintf(wrBuffer, "RPA ");
    RS232_cputs(cport_nr, wrBuffer);

    int len = static_cast<int>(strlen(wrBuffer)) + 1;
    printf("Length: %d\n", len);
    printf("sent: %d%s\n", this->address, wrBuffer);

    //Read the ECHO data
    usleep(commandSleepTime); //Give some time to answer
    int checkCount = 0;
    while (RS232_availableBytes(cport_nr) < len + 2) //Read at least (address)RPA 0\r
    {
        usleep(100);
        checkCount++;
        if (checkCount > 30)
            break;
    }
    usleep(500); //Small delay before reading
    readBytes = RS232_PollComport(cport_nr, rdBuffer, 25);
    if (readBytes < len)
    {
        printf("Error reading data\n");
        return 1;
    }
    printf("%s\n", rdBuffer);

    if (rdBuffer[0] == this->address) //Check right address in first character
    {
        for (int i = 5; i < readBytes; i++)
        {
            if (rdBuffer[i] != '\r')
            {
                data[i - 5] = (char)rdBuffer[i];
            }
            else
            {
                data[i - 5] = 0; //add the null character
                break;
            }
        }
        *newTicks = atol(data);
        printf("Enc: %ld\n", *newTicks);
    }
    else //Otherwise try a different approach to read the enconder
    {
        printf("Wrong address: %d\n", rdBuffer[0]);
        //Check last character is '\r' before processing the data
        if (rdBuffer[readBytes - 1] == 13)
        {
            int i = 0;
            //Read until the address
            while (true)
            {
                if (rdBuffer[i] == this->address)
                {
                    break;
                }
                i++;
            }

            int k = 0;
            //Then try to read the data
            while (rdBuffer[i] != 13)
            {
                printf("%d ", rdBuffer[i]);
                if (rdBuffer[i] >= 48 && rdBuffer[i] <= 57) //Just get the numbers
                {
                    data[k] = (char)rdBuffer[i];
                    k++;
                }
                i++;
            }
            printf("\n");
            data[k] = 0; //Add the null character
            printf("Data: %s\n", data);
            *newTicks = atol(data);
            printf("Enc: %ld\n", *newTicks);
        }
        else
        {
            return 1;
        }
    }

    return 0;
}
