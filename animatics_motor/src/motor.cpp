#include <iostream>
#include <unistd.h>
#include <animatics_motor/smartmotor.h>
#include <animatics_motor/rs232.h>
#include "ros/ros.h"
#include "std_msgs/Float32.h"

using namespace std;

//The motor must be a global variable for the function call back to see it
SmartMotor m1(1, 4000, 8000, 800.0); //Address=1, CountsPerRev=4000, Sampling Rate=8000Hz, MaxRPM = 800

//Callback function for subscriber
void cmdVelCallback(const std_msgs::Float32::ConstPtr &msg)
{
    float revPerSec = msg->data / 60;
    m1.sendVelocity(revPerSec); //Send the velocity in rev/sec
    ROS_INFO("Moving at [%.2f]RPM", msg->data);
}

int main(int argc, char **argv)
{
    int portN = 16;
    //Open the comport
    cout << "Openning COM Port..." << endl;
    int portState = SmartMotor::openPort(portN, 115200); //Serial Port ID (Port Id = 16 for ttyS0) and baud rate 115200
    if (portState != 0)
        return portState;
    cout << "COM Port successfully opened\n";

    cout << "Checking Motors Addresses...\n";
    //Need to address when the motors have been powered off and back on
    if (SmartMotor::CheckMotorsAddress() == 0)
    {
        cout << "Motors addresses sucessfully checked\n";
    }
    else
    {
        cout << "Motors addresses wrong. Check if the COM port is connected and the power is ON\n";
        return 1;
    }

    m1.initialize(0, 0);
    cout << "Setting velocity mode...\n";
    if (m1.setVelocityMode(5000) == 0)
        cout << "Velocity mode successfully set\n";

    cout << "Stopping the motor...\n";
    m1.sendVelocity(0);

    usleep(2000000); //Sleep for two seconds

    //Initialize ROS
    ros::init(argc, argv, "motor");

    //Create the ROS node handler
    ros::NodeHandle n;

    //Tell the master we are going to publish a message of type std_msgs/Float32
    ros::Publisher pub = n.advertise<std_msgs::Float32>("curr_vel", 1000);

    //Also tell the master we will subscribe to a message of type std_msgs/Float32
    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, cmdVelCallback);

    cout << "Waiting for new velocity command...\n";

    //Set the loop rate
    ros::Rate loop_rate(30);

    //Send the velocity
    while (ros::ok())
    {
        m1.updateVel(); //Update the current velocity
        std_msgs::Float32 msg;
        msg.data = m1.getVelocity(); //Store it in the message

        pub.publish(msg); //Send the message
        ros::spinOnce();  //Check for callbacks
        loop_rate.sleep();  //Sleep until getting to the given rate
    }

    cout << "\nStopping the motor...\n";
    //Stop the motor
    m1.sendVelocity(0);
    //Close the COM port
    usleep(2000000); //Sleep for two seconds
    cout << "Closing COM Port...\n";
    SmartMotor::closePort(portN); //Close
    cout << "DONE\n";

    return 0;
}
