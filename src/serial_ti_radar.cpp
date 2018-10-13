#include "ros/ros.h"
#include "serial/serial.h"
#include "sys/types.h"
#include <math.h>
#include <geometry_msgs/TwistStamped.h>
#include <fstream>
#include <iostream>
#include <string>
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
		printf("Connection established\n\n");
	}
	else
	{
		printf("serialInit: Failed to open port\n");
		return 0;
	}

/*//	string lineread;
//	ifstream myfile ("~/drones/src/serial_ti_radar/src/mmw_pplcount_demo_0.cfg");
//	ifstream myfile;
//	myfile.open("mmw_pplcount_demo_0.cfg");
//
//  	if (myfile.is_open()) 
//    {
    	printf("checkpoint2");
	    while (getline(myfile,lineread))
	    {
	    	fd.write(lineread);
	    	printf("checkpoint3");
	    }

	    myfile.close();
    }
*/
	string dfeDataOutputMode = "dfeDataOutputMode 1\n";
	string channelCfg ="channelCfg 15 3 0\n";
	string adcCfg ="adcCfg 2 1\n";
	string adcbufCfg ="adcbufCfg 0 1 1 1\n";
	string profileCfg ="profileCfg 0 77 300 7 62 0 0 60 1 128 2500 0 0 300\n";
	string chirpCfga ="chirpCfg 0 0 0 0 0 0 0 1\n";
	string chirpCfgb ="chirpCfg 1 1 0 0 0 0 0 2\n";
	string frameCfg ="frameCfg 0 1 128 0 50 1 0\n";
	string lowPower ="lowPower 0 1\n";
	string guiMonitor ="guiMonitor 1 1 0 0\n";
	string cfarCfg ="cfarCfg 6 4 4 4 4 16 16 4 4 50 62 0\n";
	string doaCfg ="doaCfg 600 1875 300 1\n";
	string SceneryParam ="SceneryParam -6 2 0.05 3\n";
	string GatingParam ="GatingParam 4 3 2 0\n";
	string StateParam ="StateParam 10 5 10 100 5\n";
	string AllocationParam ="AllocationParam 450 0.01 25 1 2\n";
	string VariationParam ="VariationParam 0.289 0.289 1.0\n";
	string trackingCfg ="trackingCfg 1 2 250 20 200 50 90\n";
	string sensorStart ="sensorStart\n";
	
	string readline;
	uint32_t bytes_in_buff;

	fd.flushOutput();

	fd.write(dfeDataOutputMode);
//	bytes_in_buff = fd.available();
	fd.read(readline, 300);
	cout<<readline<<endl;
	fd.write(channelCfg);
	fd.read(readline, 300);
	cout<<readline<<endl;
	fd.write(adcCfg);
	fd.read(readline, 300);
	cout<<readline<<endl;	
	fd.write(adcbufCfg);
	fd.read(readline, 300);
	cout<<readline<<endl;
	fd.write(profileCfg);
	fd.read(readline, 300);
	cout<<readline<<endl;
	fd.write(chirpCfga);
	fd.read(readline, 300);
	cout<<readline<<endl;
	fd.write(chirpCfgb);
	fd.read(readline, 300);
	cout<<readline<<endl;
	fd.write(frameCfg);
	fd.read(readline, 300);
	cout<<readline<<endl;
	fd.write(lowPower);
	fd.read(readline, 300);
	cout<<readline<<endl;
	fd.write(guiMonitor);
	fd.read(readline, 300);
	cout<<readline<<endl;
	fd.write(cfarCfg);
	fd.read(readline, 300);
	cout<<readline<<endl;
	fd.write(doaCfg);
	fd.read(readline, 300);
	cout<<readline<<endl;
	fd.write(SceneryParam);
	fd.read(readline, 300);
	cout<<readline<<endl;
	fd.write(GatingParam);
	fd.read(readline, 300);
	cout<<readline<<endl;
	fd.write(StateParam);
	fd.read(readline, 300);
	cout<<readline<<endl;
	fd.write(AllocationParam);
	fd.read(readline, 300);
	cout<<readline<<endl;
	fd.write(VariationParam);
	fd.read(readline, 300);
	cout<<readline<<endl;
	fd.write(trackingCfg);
	fd.read(readline, 300);
	cout<<readline<<endl;
	fd.write(sensorStart);
	fd.read(readline, 300);
	cout<<readline<<endl;


    fd.close();
}


