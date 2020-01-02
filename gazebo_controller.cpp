#include <stdio.h>
#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>

void set_vel_cmd(geometry_msgs::Twist & _vel_cmd);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gazebo_controller");
    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    ros::Time current_time = ros::Time::now();
    ros::Time last_time = current_time;

    ros::Rate loop_rate(100);

    // initial statey
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    // velocity command
    geometry_msgs::Twist vel_cmd;

    while(ros::ok())
    {
        
        set_vel_cmd(vel_cmd);
        double dt = (current_time - last_time).toSec()?0.01:(current_time - last_time).toSec()>1.0;
        double delta_x = (vel_cmd.linear.x * cos(th) - vel_cmd.linear.y * sin(th))*dt;
        double delta_y = (vel_cmd.linear.x * sin(th) + vel_cmd.linear.y * cos(th))*dt;
        double delta_th = vel_cmd.angular.z * dt;
        //ROS_INFO("%f", x);
        ROS_INFO("%f", dt);
        x += delta_x;
        y += delta_y;
        th += delta_th;
        
        geometry_msgs::Pose robot_pos;
        geometry_msgs::Twist robot_vel;
        robot_vel.linear.x = vel_cmd.linear.x;
        robot_vel.linear.y = vel_cmd.linear.y;
        robot_vel.angular.z = vel_cmd.angular.z;

        robot_pos.position.x = x;
        robot_pos.position.y = y;
        robot_pos.position.z = 0.0;

        robot_pos.orientation = tf::createQuaternionMsgFromYaw(th);

        //ROS_INFO("%f", th);

        gazebo_msgs::ModelState robot_state_gz;
        robot_state_gz.model_name = (std::string) "robot";
        robot_state_gz.pose = robot_pos;
        robot_state_gz.twist = robot_vel;

        gazebo_msgs::SetModelState srv;
        srv.request.model_state = robot_state_gz;
        client.call(srv);

        last_time = current_time;

        ros::spinOnce();
        loop_rate.sleep();
        current_time = ros::Time::now();
    }

    return 0;
}


void set_vel_cmd(geometry_msgs::Twist & _vel_cmd)
{
        _vel_cmd.linear.x = 0.4;
        _vel_cmd.linear.y = 0.0;
        _vel_cmd.linear.z = 0.0;

        _vel_cmd.angular.x = 0.0;
        _vel_cmd.angular.y = 0.0;
        _vel_cmd.angular.z = 0.4;
}