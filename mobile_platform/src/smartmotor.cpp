#include <mobile_platform/smartmotor.h>
#include <mobile_platform/rs232.h>
#include <string>
#include <iostream>
#include <ostream>
#include <stdlib.h>

int SmartMotor::count = 0;        //Initialize the count to 0
int SmartMotor::MaxAddress = 128; //Initialize the MaxAddress to 128
int SmartMotor::cport_nr = 0;     //Use ttyS0 as default
int addrSleepTime = 2000 * 1000;  //1s
int initSleepTime = 1000 * 1000;  //1s to wait for data
int modeSleepTime = 100 * 1000;   //100ms to wait for data
int commandSleepTime = 1800;      //1.8ms to wait after sending each command

char writeBuffer[40];         //Buffer to send data to serial port
unsigned char readBuffer[40]; //Buffer to receive data from serial port
char addrFlag[20];            //A comparisson char array for addresses
//std::string addrString;       //A comparisson String for addresses

char wrBuffer[25];          //Buffer to send data to serial port
unsigned char rdBuffer[25]; //Buffer to receive data from serial port
char data[25];              //Array to store the data read from enconders
int wrBufferLen = 25;
int rdBufferLen = 25;
int dataLen = 25;

int readBytes;

//Constructor
SmartMotor::SmartMotor(int address, int resolution, int samplingRate, float maxRPM)
{
    this->address = 128 + address;
    this->res = resolution;
    this->samp = samplingRate;
    this->velGain = (float)resolution / samplingRate * 65536.0; //Page 22 UserGuide v523
    this->maxVT = (int)(maxRPM / 60.0 * this->velGain);
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
        return 1; //Port openning error
    }
    RS232_flushRX(cport_nr);
    return 0;
}

void SmartMotor::closePort(int serialPortID)
{
    RS232_CloseComport(serialPortID);
}

//Address the available motors
int SmartMotor::addressMotors()
{
    //Changes pending
    /*
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
        usleep(commandSleepTime);

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
    }*/
}

int SmartMotor::CheckMotorsAddress()
{
    //Clear data from the arrays
    for (int i = 0; i < wrBufferLen; i++)
    {
        wrBuffer[i] = 0;
    }
    for (int i = 0; i < rdBufferLen; i++)
    {
        rdBuffer[i] = 0;
    }

    //Check the motors were successfully addressed by asking the address
    sprintf(wrBuffer, "%cRADDR ", 128);
    RS232_cputs(cport_nr, wrBuffer);
    printf("Sent: %s\n", wrBuffer);

    //Get the size of the write buffer
    int len = static_cast<int>(strlen(wrBuffer)) + 1;

    //Read back the command from the ECHO
    usleep(commandSleepTime); //Wait some time for the answer
    int checkCount = 0;
    while (RS232_availableBytes(cport_nr) < len + (SmartMotor::count)*2 - 1) //Read: RADDR (address\r) there are two characters per address minus the first
    {
        usleep(100);
        checkCount++;
        if (checkCount > 30)
            break;
    }
    usleep(500); //Small delay before reading
    readBytes = RS232_PollComport(cport_nr, rdBuffer, len + (SmartMotor::count)*2);

    //First check at least the command RADDR has been echoed back
    if (readBytes < len)
    {
        printf("Error reading data\n");
        return 1;
    }
    else
    {
        //Convert read buffer to String
        std::string rdBufferStr((char *)rdBuffer);

        //Create comparisson string: RADDR (address\r) there are two characters per address
        std::string addrStr = wrBuffer;
        for (int i = SmartMotor::count; i > 0; i--)
        {
            addrStr.append(1, i + 48);
            addrStr.append(1, '\r');
        }
        if (addrStr.compare(rdBufferStr) == 0)
        {
            return 0; //Successfully addressed
        }
        else
        {
            return 1; //Error during addressing
        }
    }
}

//Initialize the motor and clear error flags
int SmartMotor::initialize(int lowLimit, int highLimit)
{
    //Clear data from write buffer
    for (int i = 0; i < wrBufferLen; i++)
    {
        wrBuffer[i] = 0;
    }

    //Disable the low and high limits
    //!!!!!!!!!!!!!!!Check the limits for position mode!!!!!!!!!!!!!!!!!!! (TO DO TASK)
    //disable limit & reset errors
    printf("Initialazing motor %d...\n", this->address - 128);

    //Send the data
    RS232_SendByte(cport_nr, this->address);
    sprintf(wrBuffer, "EIGN(2) EIGN(3) ZS\r");
    RS232_cputs(cport_nr, wrBuffer);

    //Get the size of the write buffer
    int len = static_cast<int>(strlen(wrBuffer)) + 1;

    //Read back the command because of ECHO
    usleep(initSleepTime); //Wait initialization time
    return readECHO(len);
}

//Set position mode
int SmartMotor::setPositionMode(int accel, int vel)
{
    //Clear data from write buffer
    for (int i = 0; i < wrBufferLen; i++)
    {
        wrBuffer[i] = 0;
    }

    //Send the data
    RS232_SendByte(cport_nr, this->address);
    sprintf(wrBuffer, "MP ADT=%d VT=%d G\r", accel, vel);
    RS232_cputs(cport_nr, wrBuffer);

    //Get the size of the write buffer
    int len = static_cast<int>(strlen(wrBuffer)) + 1;

    //Read back the command because of ECHO
    usleep(modeSleepTime); //Wait some time for the answer
    return readECHO(len);
}

//Set velocity mode
int SmartMotor::setVelocityMode(int accel)
{
    //Clear data from write buffer
    for (int i = 0; i < wrBufferLen; i++)
    {
        wrBuffer[i] = 0;
    }

    //Send the data
    RS232_SendByte(cport_nr, this->address);
    sprintf(wrBuffer, "MV ADT=%d G\r", accel);
    RS232_cputs(cport_nr, wrBuffer);

    //Get the size of the write buffer
    int len = static_cast<int>(strlen(wrBuffer)) + 1;

    //Read back the command because of ECHO
    usleep(modeSleepTime); //Wait for an answer
    return readECHO(len);
}

//Set torque mode
int SmartMotor::setTorqueMode()
{
    //TO DO TASK
    return 0;
}

//Send the position
int SmartMotor::sendPosition(long position)
{
    //Clear data from write buffer
    for (int i = 0; i < wrBufferLen; i++)
    {
        wrBuffer[i] = 0;
    }

    //Send the data
    RS232_SendByte(cport_nr, this->address);
    sprintf(wrBuffer, "PT=%ld G\r", position);
    RS232_cputs(cport_nr, wrBuffer);

    //Get the size of the write buffer
    int len = static_cast<int>(strlen(wrBuffer)) + 1;

    //Read back the command because of ECHO
    usleep(commandSleepTime); //Wait some time for the answer
    return readECHO(len);
}

//Send the velocity
int SmartMotor::sendVelocity(float revPerSec)
{
    //Calculate the value for VT
    int vt = (int)(revPerSec * this->velGain);
    //printf("RevPerSec: %.2f\n", revPerSec);

    //Check for maximum velocity
    if (abs(vt) > this->maxVT)
    {
        if (vt > 0)
            vt = this->maxVT;
        else
            vt = -1 * this->maxVT;

        printf("Maximum velocity reached. Using maximum velocity: %.2fRPM\n", vt * 60.0 / this->velGain);
    }

    //Clear data from write buffer
    for (int i = 0; i < wrBufferLen; i++)
    {
        wrBuffer[i] = 0;
    }

    //Send the velocity command with the desired velocity
    RS232_SendByte(cport_nr, this->address);
    sprintf(wrBuffer, "VT=%d G\r", vt);
    RS232_cputs(cport_nr, wrBuffer);

    //Get the size of the data sent
    int len = static_cast<int>(strlen(wrBuffer)) + 1;

    //Read back the command because of ECHO
    usleep(commandSleepTime); //Wait some time for the answer
    return readECHO(len);
}

//Send the torque
int SmartMotor::sendTorque(int torque)
{
    //TO DO TASK
    return 0;
}

int SmartMotor::stop()
{
    //Clear data from write buffer
    for (int i = 0; i < wrBufferLen; i++)
    {
        wrBuffer[i] = 0;
    }

    //Send STOP Command to the motor
    RS232_SendByte(cport_nr, this->address);
    sprintf(wrBuffer, "OFF\r");
    RS232_cputs(cport_nr, wrBuffer);

    //Get the size of the data sent
    int len = static_cast<int>(strlen(wrBuffer)) + 1;

    //Read back the command because of ECHO
    usleep(commandSleepTime); //Wait some time for the answer
    return readECHO(len);
}

int SmartMotor::sendCommandAll(const char *command)
{
    //Clear data from write buffer
    for (int i = 0; i < wrBufferLen; i++)
    {
        wrBuffer[i] = 0;
    }

    //Send the data
    RS232_SendByte(cport_nr, 128);
    sprintf(wrBuffer, "%s\n", command);
    RS232_cputs(cport_nr, wrBuffer);

    //Get the size of the write buffer
    int len = static_cast<int>(strlen(wrBuffer)) + 1;

    //Read back the command because of ECHO
    usleep(modeSleepTime); //Wait for an answer

    //Clear data from read buffer
    for (int i = 0; i < rdBufferLen; i++)
    {
        rdBuffer[i] = 0;
    }

    //Wait for data to be available
    int checkCount = 0;
    while (RS232_availableBytes(cport_nr) < len)
    {
        usleep(100);
        checkCount++;
        if (checkCount > 30)
            break;
    }
    usleep(500); //Small delay before reading

    //Read the data
    readBytes = RS232_PollComport(cport_nr, rdBuffer, len);
    if (readBytes < len) //If the data was not read correctly
    {
        return 1;
    }
    return 0;
}

int SmartMotor::sendCommand(const char *command)
{
    //Clear data from the arrays
    for (int i = 0; i < wrBufferLen; i++)
    {
        wrBuffer[i] = 0;
    }
    for (int i = 0; i < rdBufferLen; i++)
    {
        rdBuffer[i] = 0;
    }

    RS232_SendByte(cport_nr, this->address);
    sprintf(wrBuffer, "%s ", command);
    RS232_cputs(cport_nr, wrBuffer);

    int len = static_cast<int>(strlen(wrBuffer)) + 1;
    printf("Length: %d\n", len);
    printf("sent: %d%s\n", this->address, wrBuffer);

    //Read the ECHO data
    usleep(commandSleepTime); //Give some time to answer
    int checkCount = 0;
    while (RS232_availableBytes(cport_nr) < len + 4) //Read at least (address)RRES ****\r
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
        printf("Error receiving ECHO\n");
        return 1;
    }
    std::cout << rdBuffer << std::endl;
}

int SmartMotor::updateTicks()
{
    //Clear data from the arrays
    for (int i = 0; i < wrBufferLen; i++)
    {
        wrBuffer[i] = 0;
    }
    for (int i = 0; i < rdBufferLen; i++)
    {
        rdBuffer[i] = 0;
    }
    for (int i = 0; i < dataLen; i++)
    {
        data[i] = 0;
    }

    RS232_SendByte(cport_nr, this->address);
    sprintf(wrBuffer, "RPA ");
    RS232_cputs(cport_nr, wrBuffer);

    int len = static_cast<int>(strlen(wrBuffer)) + 1;

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
        printf("Error reading current position\n");
        return 1;
    }

    //Check right address in first character
    if (rdBuffer[0] == this->address)
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
        //Update the enconder ticks
        this->lastTicks = this->currTicks;
        this->currTicks = atol(data);
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
            //Update the enconder ticks
            this->lastTicks = this->currTicks;
            this->currTicks = atol(data);
        }
        else
        {
            return 1;
        }
    }
    return 0;
}

int SmartMotor::updateVel()
{
    //VA*((SAMP/65536.0)/RES) Page 764 Developers Guide
    //Clear data from the arrays
    for (int i = 0; i < wrBufferLen; i++)
    {
        wrBuffer[i] = 0;
    }
    for (int i = 0; i < rdBufferLen; i++)
    {
        rdBuffer[i] = 0;
    }
    for (int i = 0; i < dataLen; i++)
    {
        data[i] = 0;
    }

    RS232_SendByte(cport_nr, this->address);
    sprintf(wrBuffer, "RVA ");
    RS232_cputs(cport_nr, wrBuffer);

    int len = static_cast<int>(strlen(wrBuffer)) + 1;

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
        printf("Error reading current position\n");
        return 1;
    }

    //Check right address in first character
    if (rdBuffer[0] == this->address)
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
        //Update the velocity
        long velRaw = atol(data);
        this->vel = velRaw / this->velGain;
    }
    else //Otherwise try a different approach to read the enconder
    {
        printf("Warning: wrong address: %d\n", rdBuffer[0]);
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
            //Update the velocity
            long velRaw = atol(data);
            this->vel = velRaw / this->velGain;
        }
        else
        {
            return 1;
        }
    }
    return 0;
}

int SmartMotor::readECHO(int len)
{
    //Clear data from read buffer
    for (int i = 0; i < rdBufferLen; i++)
    {
        rdBuffer[i] = 0;
    }

    //Wait for data to be available
    int checkCount = 0;
    while (RS232_availableBytes(cport_nr) < len)
    {
        usleep(100);
        checkCount++;
        if (checkCount > 30)
            break;
    }
    usleep(500); //Small delay before reading

    //Read the data
    readBytes = RS232_PollComport(cport_nr, rdBuffer, len);
    if (readBytes < len) //If the data was not read correctly
    {
        return 1;
    }
    return 0;
}
