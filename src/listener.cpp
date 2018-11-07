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

double toEulerAngle(double q_w, double q_x, double q_y, double q_z)
{
  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (q_w * q_x + q_y * q_z);
  double cosr_cosp = +1.0 - 2.0 * (q_x * q_x + q_y * q_y);
  double roll = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (q_w * q_y - q_z * q_x);
  if (fabs(sinp) >= 1)
  double   pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
  double   pitch = asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (q_w * q_z + q_x * q_y);
  double cosy_cosp = +1.0 - 2.0 * (q_y * q_y + q_z * q_z);  
  double yaw = atan2(siny_cosp, cosy_cosp);
  return yaw;
}

double frame_conversion_yaw_x(double input_yaw,double input_x, double input_y)
{
  //convert body frame to local frame
  return input_x * cos(input_yaw) - input_y * sin(input_yaw);
}

double frame_conversion_yaw_y(double input_yaw,double input_x, double input_y)
{
  //convert body frame to local frame
  return input_x * sin(input_yaw) + input_y * cos(input_yaw);
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

    double yaw_angle;
    double pos_x, pos_y;
    double posX = 1;
    double posY = 0;

    while(ros::ok()){
//        cout<<current_state<<endl;
//        cout<<current_imu.orientation<<endl<<endl;
//        cout<<current_imu.orientation.x<<endl<<endl;
        
        yaw_angle = toEulerAngle(current_imu.orientation.w, current_imu.orientation.x,current_imu.orientation.y,current_imu.orientation.z);
        printf("yaw:   %lf       ", yaw_angle * 180/ 3.1415926);
        
        pos_x = frame_conversion_yaw_x(yaw_angle, posX, posY);
        pos_y = frame_conversion_yaw_y(yaw_angle, posX, posY);
        printf("x = %lf    y = %lf    \n", pos_x, pos_y);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

