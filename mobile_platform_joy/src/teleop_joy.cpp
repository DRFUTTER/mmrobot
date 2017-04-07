#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class TeleopMobPlat
{
public:
  TeleopMobPlat();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);

  ros::NodeHandle nh_;

  int accel_, brake_, turn_, stop_;
  double vel_scale, turn_scale;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
};

TeleopMobPlat::TeleopMobPlat() : accel_(1),
                                 turn_(2),
                                 stop_(3)
{
  //Linear and angular velocity obtained from parameter server
  nh_.param("axis_accel", accel_, accel_);
  nh_.param("axis_brake", brake_, brake_);
  nh_.param("axis_turn",turn_, turn_);

  //Linear and angular scale obtained from parameter server
  nh_.param("scale_accel", vel_scale, vel_scale);
  nh_.param("scale_turn", turn_scale, turn_scale);

  //Stop button obtained from parameter server
  nh_.param("stop_button", stop_, 1);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("mob_plat/cmd_vel", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopMobPlat::joyCallback, this);
}

void TeleopMobPlat::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
  geometry_msgs::Twist twist;
  float accel;
  float brake;
  float turn;
  //Check if stop button was pressed
  if (joy->buttons[stop_] == 0)
  {
    accel = -1 * vel_scale * (joy->axes[accel_] - 1.0);
    brake = vel_scale * (joy->axes[brake_] - 1.0);
    turn = turn_scale * (joy->axes[turn_] - 1.0) + turn_scale;
    twist.linear.x = accel + brake;
    twist.angular.z = turn;
  }
  else
  {
    twist.angular.z = 0.0;
    twist.linear.x = 0.0;
  }
  vel_pub_.publish(twist);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teleop_mob_plat");
  TeleopMobPlat teleop_mop_plat;

  ros::spin();
}