/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>

mavros_msgs::State current_state;
sensor_msgs::Imu current_imu;

using namespace std;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void imu_cb(const sensor_msgs::Imu::ConstPtr& msg){
    current_imu = *msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
            ("mavros/imu/data", 10, imu_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        cout<<current_state<<endl;
        cout<<current_imu<<endl;
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}