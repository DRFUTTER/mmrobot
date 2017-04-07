#include <iostream>
#include <unistd.h>
#include <mobile_platform/smartmotor.h>
#include <mobile_platform/rs232.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#define _USE_MATH_DEFINES
#include <math.h>
//#include <mobile_platform/Vels.h>

using namespace std;

int portN = 0;  //Serial Port ID
float L = 0.42; //Distance between the wheels
float Lby2 = L / 2.0;
float wheelRadius = 0.075;
float gearRatio = 10.0;    //Gear box ratio in each motor
float revToRad = 2 * M_PI; //Constant to transform from rev/sec to rad/sec

//The motor must be a global variable for the function call back to see it
SmartMotor motor_L(1, 4000, 8000, 5.6); //Address=1, CountsPerRev=4000, Sampling Rate=8000Hz, MaxRPS = 28 for a 0.2rad/sec mobile platform turn
SmartMotor motor_R(2, 4000, 8000, 5.6); //Address=2, CountsPerRev=4000, Sampling Rate=8000Hz, MaxRPS = 28 for a 0.2rad/sec mobile platform turn
SmartMotor motor_Z(3, 8000, 8000, 0.4); //Address=3, CountsPerRev=8000, Sampling Rate=8000Hz, MaxRPS = 0.4

void twistToWheelsVels(float linearVel, float angularVel, float *Rvel, float *Lvel)
{
    //First calculate the linear velocity of each wheel
    float linVelR = linearVel + angularVel * Lby2; //angular velocity in rad/sec
    float linVelL = linearVel - angularVel * Lby2; //angular velocity in rad/sec

    //Then calculate the angular velocity of each wheel
    *Rvel = gearRatio * linVelR / (revToRad * wheelRadius);        //Multiply by gear ratio an transform to rev/sec
    *Lvel = -1.0 * gearRatio * linVelL / (revToRad * wheelRadius); //Times -1 becase the rotation is opposite
}

void wheelsVelsToTwist(float Rvel, float Lvel, float *linearVel, float *angularVel)
{
    //First calculate the linear velocity of each wheel
    float linVelR = (Rvel * revToRad * wheelRadius) / gearRatio;      //Transform to angular velocity to rad/sec and also divide by gear ratio
    float linVelL = -1 * (Lvel * revToRad * wheelRadius) / gearRatio; //Times -1 because the rotation is opposite

    //Then calculate the linear and angular velocity of the mobile platform
    *linearVel = (linVelR + linVelL) * 0.5;
    *angularVel = (linVelR - linVelL) / L; //angular velocity in rad/sec
}

//Callback function for subscriber
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &twist)
{
    float Rvel;
    float Lvel;
    //Transform twist to velocity wheels
    twistToWheelsVels(twist->linear.x, twist->angular.z, &Rvel, &Lvel);

    //Send the velocities to the motors
    ROS_INFO("New vel R: [%.2f] L:[%.2f] RPS", Rvel, Lvel);
    motor_R.sendVelocity(Rvel); //Positive CW
    motor_L.sendVelocity(Lvel); //Positive CCW
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
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("mob_plat/curr_vel", 100);

    //Also tell the master we will subscribe to a message of type Vels
    ros::Subscriber sub = n.subscribe("mob_plat/cmd_vel", 1000, cmdVelCallback);

    cout << "Waiting for new velocity command...\n";

    //Set the loop rate
    ros::Rate loop_rate(30);

    //Instatiate the twist message
    geometry_msgs::Twist twist;

    //Send the velocity
    while (ros::ok())
    {
        //Update velocities
        motor_R.updateVel();
        motor_L.updateVel();

        float linearVel, angularVel;

        //Transform wheel velocities to twist
        wheelsVelsToTwist(motor_R.getVelocity(), motor_L.getVelocity(), &linearVel, &angularVel);

        twist.linear.x = linearVel;
        twist.angular.z = angularVel;

        pub.publish(twist); //Send the message
        ros::spinOnce();    //Check for callbacks
        loop_rate.sleep();  //Sleep until getting to the given rate
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
