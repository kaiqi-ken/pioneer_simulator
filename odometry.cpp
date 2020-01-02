#include <string.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

//#include <random_numbers.h>
// initial position
double x = 0.0;
double y = 0.0;
double th = 0;

// initial noisy position
double x_n = 0.0;
double y_n = 0.0;
double th_n = 0.0;

// velocity
double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

void cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
    vx = twist_aux.linear.x;
    vy = twist_aux.linear.y;
    vth = twist_aux.angular.z;
    //ROS_INFO("reciving message");
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "odometry");
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
    ros::Subscriber vel_cmd_sub = n.subscribe("cmd_vel", 10, cmd_velCallback);


    ros::Time current_time;
    ros::Time last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(10);

    const double degree = M_PI/180;

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    //random_numbers::RandomNumberGenerator random_generator();
    while(ros::ok())
    {
        current_time = ros::Time::now();

        double dt = (current_time - last_time).toSec();
        double delta_x = (vx * cos(th) - vy * sin(th))*dt;
        double delta_y = (vx * sin(th) + vy * cos(th))*dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        //random_generator.gaussian(0.0, 1.0);
        x_n += delta_x;
        y_n += delta_y;
        th_n += delta_th;

        geometry_msgs::Quaternion odom_quat;
        odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, th);

        // update transform
        odom_trans.header.stamp = current_time;
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);

        // filling the odometry
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";

        // position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        // velocity
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = vth;

        last_time = current_time;

        // publishing the odometry and the new tf
        broadcaster.sendTransform(odom_trans);
        //odom_pub.publish(odom);
        //ROS_INFO("%f", th);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}