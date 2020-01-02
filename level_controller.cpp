#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "level_controller");
    ros::NodeHandle n;

    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    ros::Rate loop_rate(0.25);
    int all_data_count = 0;
    int train_data_count = 0;

    int init_pos_x = 0;
    int init_pos_y = 0;

    geometry_msgs::Twist control;
    while(ros::ok())
    {
        ROS_INFO("%d", all_data_count);
        set_init_pos(init_pos_x, init_pos_y);
        set_control(control);


        all_data_count++;
        train_data_count++;
        loop_rate.sleep();
    }
    
    return 0;
}

void set_init_pos(int& _init_pos_x, int& _init_pos_y)
{
    _init_pos_x = 0;
    _init_pos_y = 0;
}

void set_control(geometry_msgs::Twist _control)
{
        _control.linear.x = 0.50;
        _control.linear.y = 0.0;
        _control.linear.z = 0.0;

        _control.angular.x = 0.0;
        _control.angular.y = 0.0;
        _control.angular.z = 0.50;
}