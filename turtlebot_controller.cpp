#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <tf/transform_broadcaster.h>
#include "robot_simulator/LevelController.h"

#define TIME_LENGTH 5

enum ControlMode {Stop, Circle, Line};

ControlMode control_mode = Stop;
int control_mode_int = 0;
//bool record_data = false;
//bool start_state = false;
void set_control_in_gz(geometry_msgs::Twist& _control, ros::ServiceClient& _robot_pos_client, ros::ServiceClient& _robot_pos_client_get);
void send_msg_listener(const bool& _record_data, const ros::Publisher& _record_data_pub);
void set_control(geometry_msgs::Twist& _control, const ControlMode& _control_mode);
void reset_robot_pos(ros::ServiceClient& _robot_pos_client, ros::ServiceClient& _robot_pos_client_get);
// void start_stateCallback(const std_msgs::Bool &_start_state);
void init();
void wait_for_time(double length);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "turtlebot_controller");
    ros::NodeHandle n;

    ros::Publisher cmd_pos_pub = n.advertise<geometry_msgs::Pose>("cmd_pos", 1);
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Publisher record_data_pub = n.advertise<std_msgs::Bool>("record_data", 1);

    // ros::ServiceClient robot_pos_client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    // ros::ServiceClient robot_pos_client_get = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    // ros::Subscriber start_state_sub = n.subscribe("start_state", 10, start_stateCallback);

    ros::Rate loop_rate(30);

    init();

    bool trial_start = false;

    ros::Time start_time;
    ros::Time end_time;


    ros::Rate temp_rate(10);
    bool listener_state=false;
    while(!listener_state)
    {
        ros::param::get("listener_state", listener_state);
        ROS_INFO("waiting for listener start");
        temp_rate.sleep();
    }
    ROS_INFO("listener is ready");

    while(ros::ok())
    {

        int experiment_state = 0;
        ros::param::get("experiment_state", experiment_state);


        if(experiment_state==0) // init state
        {
            ros::param::set("experiment_state", 1);
            experiment_state = 1;

            geometry_msgs::Pose cmd_pos;
            cmd_pos.position.x = 0;
            cmd_pos.position.y = 0;
            cmd_pos.position.z = 0.2;
            cmd_pos.orientation.x = 0;
            cmd_pos.orientation.y = 0;
            cmd_pos.orientation.z = 0;
            
            cmd_pos_pub.publish(cmd_pos);
            //reset_robot_pos(robot_pos_client);
            control_mode = Stop;
            geometry_msgs::Twist cmd_vel;
            set_control(cmd_vel, control_mode);
            // set_control_in_gz(cmd_vel, robot_pos_client, robot_pos_client_get);
            // cmd_pub.publish(cmd_vel);
            wait_for_time(1);

            // ros::ServiceClient pauseGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
            // ros::ServiceClient unpauseGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
            // std_srvs::Empty pauseSrv;
            // std_srvs::Empty unpauseSrv;
            // pauseGazebo.call(pauseSrv);
            // reset_robot_pos(robot_pos_client, robot_pos_client_get);  
            // wait_for_time(1);
            // reset_robot_pos(robot_pos_client, robot_pos_client_get);  
            // wait_for_time(1);
            // unpauseGazebo.call(unpauseSrv);

            // control_mode = Stop;
            // //geometry_msgs::Twist cmd_vel;
            // set_control(cmd_vel, control_mode);
            // cmd_pub.publish(cmd_vel);
            // wait_for_time(1);


            control_mode = Circle;
            start_time = ros::Time::now();
            ROS_INFO("trail start");

            
            //ROS_INFO("start_time:%f", start_time.toSec());
            

        }
        else if(experiment_state==1) // experiment is processing
        {
            end_time = ros::Time::now();
            //ROS_INFO("end_time:%f", end_time.toSec());
            if(end_time.toSec() - start_time.toSec() > TIME_LENGTH)
            {
                    control_mode = Stop;
                    geometry_msgs::Twist cmd_vel;
                    set_control(cmd_vel, control_mode);
                    //set_control_in_gz(cmd_vel, robot_pos_client, robot_pos_client_get);
                    cmd_vel_pub.publish(cmd_vel);

                    experiment_state = 2; // switch to saving files
                    ros::param::set("experiment_state", 2);

                    ROS_INFO("trail is over");
            }
            else
            {
                // during the experiment, sending cmd_vel to turtle_bot
                geometry_msgs::Twist cmd_vel;
                set_control(cmd_vel, control_mode);
                cmd_vel_pub.publish(cmd_vel);
                //set_control_in_gz(cmd_vel, robot_pos_client, robot_pos_client_get);
            }
        }
        else if(experiment_state==2) // saving file
        {
            ROS_INFO("waiting for saving files");
        }
        loop_rate.sleep();
    }
    
    return 0;
}

void send_msg_listener(const bool& _record_data, const ros::Publisher& _record_data_pub)
{   
    std_msgs::Bool _record_data_msg;
    _record_data_msg.data = _record_data;
    //ROS_INFO("%d", _record_data_msg.data);
    _record_data_pub.publish(_record_data_msg);

}

void set_control_in_gz(geometry_msgs::Twist& _control, ros::ServiceClient& _robot_pos_client, ros::ServiceClient& _robot_pos_client_get)
{
    gazebo_msgs::GetModelState srv_get;
    srv_get.request.model_name = (std::string) "turtlebot3_waffle_pi";
    _robot_pos_client_get.call(srv_get);

    gazebo_msgs::ModelState robot_state_gz;
    robot_state_gz.model_name = (std::string) "turtlebot3_waffle_pi";
    robot_state_gz.pose = srv_get.response.pose;
    robot_state_gz.twist = _control;
    //robot_state_gz.pose = srv_get.response.pose;
    //robot_state_gz.twist = srv_get.response.twist;

    gazebo_msgs::SetModelState srv;
    srv.request.model_state = robot_state_gz;
    _robot_pos_client.call(srv);
}

void set_control(geometry_msgs::Twist& _control, const ControlMode& _control_mode)
{
    switch(_control_mode)
    {
        case Circle:
            _control.linear.x = 2.0;
            _control.linear.y = 0.5;
            _control.linear.z = 0.0;

            _control.angular.x = 0.0;
            _control.angular.y = 0.0;
            _control.angular.z = 0.0;
            break;
        case Line:
            _control.linear.x = 2.0;
            _control.linear.y = 2.0;
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

void init()
{
    // experiment_state: 0 init; 1 processing; 2 saving file

    if(!ros::param::has("experiment_state"))
        ros::param::set("experiment_state", 0);
    else
    {
        ros::param::del("experiment_state");
        ros::param::set("experiment_state", 0);
    }

    if(!ros::param::has("listener_state"))
        ros::param::set("listener_state", false);
    else
    {
        ros::param::del("listener_state");
        ros::param::set("listener_state", false);
    }
}

void reset_robot_pos(ros::ServiceClient& _robot_pos_client, ros::ServiceClient& _robot_pos_client_get)
{
    gazebo_msgs::GetModelState srv_get;
    srv_get.request.model_name = (std::string) "turtlebot3_waffle_pi";
    _robot_pos_client_get.call(srv_get);


    geometry_msgs::Pose robot_pos;
    geometry_msgs::Twist robot_vel;
    robot_vel.linear.x = 0.0;
    robot_vel.linear.y = 0.0;
    robot_vel.linear.z = 0.0;
    robot_vel.angular.x = 0.0;
    robot_vel.angular.y = 0.0;
    robot_vel.angular.z = 0.0;

    robot_pos.position.x = 0.0;
    robot_pos.position.y = 0.0;
    robot_pos.position.z = srv_get.response.pose.position.z;

    robot_pos.orientation = tf::createQuaternionMsgFromYaw(0.0);

    //ROS_INFO("%f", th);

    gazebo_msgs::ModelState robot_state_gz;
    robot_state_gz.model_name = (std::string) "turtlebot3_waffle_pi";
    robot_state_gz.pose = robot_pos;
    robot_state_gz.twist = robot_vel;
    //robot_state_gz.pose = srv_get.response.pose;
    //robot_state_gz.twist = srv_get.response.twist;



    gazebo_msgs::SetModelState srv;
    srv.request.model_state = robot_state_gz;
    _robot_pos_client.call(srv);
}

void wait_for_time(double length)
{
    ros::Time start_time = ros::Time::now();
    ros::Time end_time = ros::Time::now();
            //ROS_INFO("end_time:%f", end_time.toSec());
    while(end_time.toSec() - start_time.toSec() < length)
    {
        end_time = ros::Time::now();
    }
}