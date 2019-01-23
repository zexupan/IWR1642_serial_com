/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>


//#include <mavros_msgs/State.h>
//#include <sensor_msgs/Imu.h>

using namespace std;

//mavros_msgs::State current_state;
//sensor_msgs::Imu current_imu;

geometry_msgs::PointStamped current_targetposMsg;
geometry_msgs::PoseStamped current_droneposMsg;

geometry_msgs::PointStamped posMsg;
geometry_msgs::PointStamped pos1Msg;


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

//void state_cb(const mavros_msgs::State::ConstPtr& msg){
//    current_state = *msg;
//}

//void imu_cb(const sensor_msgs::Imu::ConstPtr& msg){
//    current_imu = *msg;
//}


void posestamped_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_droneposMsg = *msg;
}

void pointstamped_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
    current_targetposMsg = *msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

//    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
//            ("mavros/state", 10, state_cb);

//    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
//            ("mavros/imu/data", 10, imu_cb);


    ros::Subscriber pointstamped_sub = nh.subscribe<geometry_msgs::PointStamped>
            ("serial_ti_radar_read/radar_info0", 10, pointstamped_cb);


    ros::Subscriber posestamped_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, posestamped_cb);

    ros::Publisher pos0Pub = nh.advertise<geometry_msgs::PointStamped>("drone_local_position", 1000);
    ros::Publisher pos1Pub = nh.advertise<geometry_msgs::PointStamped>("radar_target_position", 1000);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    ros::Time last_request = ros::Time::now();

    double yaw_angle;
    double pos_x, pos_y; //radar target position in loacl enu frame
    double posX, posY; //radar target position in drone body frame
    double drone_posx, drone_posy; //drone local position

    while(ros::ok()){

        yaw_angle = toEulerAngle(current_droneposMsg.pose.orientation.w, current_droneposMsg.pose.orientation.x, current_droneposMsg.pose.orientation.y, current_droneposMsg.pose.orientation.z);//(current_imu.orientation.w, current_imu.orientation.x,current_imu.orientation.y,current_imu.orientation.z);
        printf("yaw:   %lf       ", yaw_angle * 180/ 3.1415926);
        
        //sensor frame to drone body frame transformation
        posX = current_targetposMsg.point.y;
        posY = current_targetposMsg.point.x;

        //drone local position
        drone_posx = current_droneposMsg.pose.position.x;
        drone_posy = current_droneposMsg.pose.position.y;

        //radar target position in loacl enu frame
        pos_x = drone_posx+ frame_conversion_yaw_x(yaw_angle, posX, posY);
        pos_y = drone_posy + frame_conversion_yaw_y(yaw_angle, posX, posY);
 
        printf("\n\nx = %lf    y = %lf    ", drone_posx, drone_posy);
        printf("x = %lf    y = %lf    ", pos_x, pos_y);

        posMsg.header.stamp = ros::Time::now();
        posMsg.header.frame_id = '1';//tlv_data_targetObjectList_trackID;
        posMsg.point.x = drone_posx;
        posMsg.point.y = drone_posy;
        posMsg.point.z = 0.0;
        pos0Pub.publish(posMsg);

        pos1Msg.header.stamp = ros::Time::now();
        pos1Msg.header.frame_id = '1';//tlv_data_targetObjectList_trackID;
        pos1Msg.point.x = pos_x;
        pos1Msg.point.y = pos_y;
        pos1Msg.point.z = 0.0;
        pos1Pub.publish(pos1Msg);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

