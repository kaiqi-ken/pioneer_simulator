#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <tf/transform_broadcaster.h>
#include "robot_simulator/LevelController.h"

#define TIME_LENGTH 5
enum ControlMode {Stop, Circle, Line};

ControlMode control_mode = Line;

void set_control(geometry_msgs::Twist& _control, const ControlMode& _control_mode);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "control_test");
    ros::NodeHandle n;
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Publisher cmd_pos_pub = n.advertise<geometry_msgs::Pose>("cmd_pos", 1);

    ros::Rate loop_rate(30);

    int count = 0;
    while(ros::ok())
    {
        geometry_msgs::Twist cmd_vel;
        set_control(cmd_vel, control_mode);
        //set_control_in_gz(cmd_vel, robot_pos_client, robot_pos_client_get);
        cmd_vel_pub.publish(cmd_vel);
        
        if(count%100==0)
        {
            geometry_msgs::Pose cmd_pos;
            cmd_pos.position.x = 0;
            cmd_pos.position.y = 0;
            cmd_pos.position.z = 0.2;
            ROS_INFO("%d", count);
            ROS_INFO("%f", cmd_pos.position.x);

            cmd_pos_pub.publish(cmd_pos);
        }
        count++;
        loop_rate.sleep();
    }
    
    return 0;
}

void set_control(geometry_msgs::Twist& _control, const ControlMode& _control_mode)
{
    switch(_control_mode)
    {
        case Circle:
            _control.linear.x = 0.75;
            _control.linear.y = 0.25;
            _control.linear.z = 0.0;

            _control.angular.x = 0.0;
            _control.angular.y = 0.0;
            _control.angular.z = 0.0;
            break;
        case Line:
            _control.linear.x = 0.75;
            _control.linear.y = 0.75;
            _control.linear.z = 0.0;

            _control.angular.x = 0.0;
            _control.angular.y = 0.0;
            _control.angular.z = 0.0;
            break;
        case Stop:
            _control.linear.x = 0.0;
            _control.linear.y = 0.0;
            _control.linear.z = 0.0;

            _control.angular.x = 0.0;
            _control.angular.y = 0.0;
            _control.angular.z = 0.0;
            break;
    }

}
