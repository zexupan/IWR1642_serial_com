#include "ros/ros.h"
#include "serial/serial.h"
#include "sys/types.h"
#include <math.h>
#include <geometry_msgs/TwistStamped.h>
#include <fstream>
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
//#include "mmw_pplcount_demo_0.cfg"

using namespace std;

int main(int argc, char *argv[])
{
	//creat ros handler to node
	ros::init(argc, argv, "serial_TI_radar");
	ros::NodeHandle serial_radar_nh("~");

	serial::Serial fd;

	string serialPort;

	if(serial_radar_nh.getParam("serialPort", serialPort))
		printf("Retrived Port Name: %s\n", serialPort.data());
	else
	{
		printf("Cannot retrived Port name. Exit\n");
		exit(-1);
	}

	fd.setPort(serialPort.data());
	fd.setBaudrate(115200);
	fd.setTimeout(5, 10, 2, 10, 2);
	fd.open();
	if (fd.isOpen())
	{
		fd.flushInput();
		fd.flushOutput();
		printf("Connection established\n\n");
	}
	else
	{
		printf("serialInit: Failed to open port\n");
		return 0;
	}

	string lineread;
	string readline;
	int counter = 0;
	ifstream myfile("/home/pzx/drones/src/serial_ti_radar/src/mmw_pplcount_demo_0.cfg");

  	if (myfile.is_open()) 
    {
    	fd.flushOutput();

	    while (getline(myfile,lineread))
	    {
	    	fd.write(lineread);
	    	fd.write("\n");
	    	counter++;
	    	int bytes_in_buff = fd.available();
	    	cout<<"bytes in buff = "<<bytes_in_buff<<endl;
	    	fd.read(readline, 300);
	    	fd.flushOutput();
	    	cout<<readline<<endl;
	    	cout<<"counter = "<<counter<<endl;
	    }

	    myfile.close();
    }

    fd.close();
}