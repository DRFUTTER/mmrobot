#include <iostream>
#include <unistd.h>
#include <mobile_platform/smartmotor.h>
#include <mobile_platform/rs232.h>
#include "ros/ros.h"
#include <mobile_platform/Vels.h>

using namespace std;

int portN = 0; //Serial Port ID

//The motor must be a global variable for the function call back to see it
SmartMotor motor_L(1, 4000, 8000, 800.0); //Address=2, CountsPerRev=4000, Sampling Rate=8000Hz, MaxRPM = 800
SmartMotor motor_R(2, 4000, 8000, 800.0); //Address=1, CountsPerRev=4000, Sampling Rate=8000Hz, MaxRPM = 800
SmartMotor motor_Z(3, 8000, 8000, 10.0);  //Address=1, CountsPerRev=8000, Sampling Rate=8000Hz, MaxRPM = 800

//Callback function for subscriber
void cmdVelCallback(const mobile_platform::Vels::ConstPtr& msg)
{
    //Transform velocities to rev/sec
    float Rvel = msg->R_vel / 60.0;
    float Lvel = msg->L_vel / 60.0;
    ROS_INFO("New vel R: [%.2f] L:[%.2f] RPM", Rvel, Lvel);
    motor_R.sendVelocity(Rvel);
    motor_L.sendVelocity(Lvel);
}

int main(int argc, char **argv)
{
    
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

    //Initialize motors
    motor_R.initialize(0, 0);
    motor_L.initialize(0, 0);

    cout << "Setting velocity mode for motor R...\n";
    if (motor_R.setVelocityMode(5000) == 0)
        cout << "Velocity mode successfully set\n";

    cout << "Setting velocity mode for motor L...\n";
    if (motor_L.setVelocityMode(5000) == 0)
        cout << "Velocity mode successfully set\n";

    cout << "Stopping the motors...\n";
    motor_R.sendVelocity(0);
    motor_L.sendVelocity(0);

    usleep(1000000); //Sleep for one second

    //Initialize ROS
    ros::init(argc, argv, "motor");

    //Create the ROS node handler
    ros::NodeHandle n;

    //Tell the master we are going to publish a message of type Vels
    ros::Publisher pub = n.advertise<mobile_platform::Vels>("curr_vel", 100);

    //Also tell the master we will subscribe to a message of type Vels
    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, cmdVelCallback);

    cout << "Waiting for new velocity command...\n";

    //Set the loop rate
    ros::Rate loop_rate(30);

    //Instatiate the velocities message
    mobile_platform::Vels vels_msg;

    //Send the velocity
    while (ros::ok())
    {
        //Update velocities
        motor_R.updateVel();
        motor_L.updateVel();
        vels_msg.R_vel = motor_R.getVelocity();
        vels_msg.L_vel = motor_L.getVelocity();

        pub.publish(vels_msg); //Send the message
        ros::spinOnce();      //Check for callbacks
        loop_rate.sleep();    //Sleep until getting to the given rate
    }

    cout << "\nStopping the motors...\n";
    //Stop the motors
    motor_R.sendVelocity(0);
    motor_L.sendVelocity(0);
    //Close the COM port
    usleep(1000000); //Sleep for two seconds
    cout << "Closing COM Port...\n";
    SmartMotor::closePort(portN); //Close
    cout << "DONE\n";

    return 0;
}
