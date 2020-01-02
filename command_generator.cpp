#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <iostream>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "command_generator");
    ros::NodeHandle n;

    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    ros::Rate loop_rate(10);

    while(ros::ok())
    {   
        geometry_msgs::Twist vel_cmd;

        vel_cmd.linear.x = 0.4;
        vel_cmd.linear.y = 0.0;
        vel_cmd.linear.z = 0.0;

        vel_cmd.angular.x = 0.0;
        vel_cmd.angular.y = 0.0;
        vel_cmd.angular.z = 0.4;

        cmd_vel_pub.publish(vel_cmd);
        //ROS_INFO("publish message");

        loop_rate.sleep();
    }

    return 0;
}