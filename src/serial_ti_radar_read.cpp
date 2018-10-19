#include "ros/ros.h"
#include "serial/serial.h"
#include "sys/types.h"
#include <math.h>

//define 5 different messages and the buffer
uint8_t frame_header[52];
uint8_t frame_header_sync[8];
uint8_t buffer[14];



//define starting and ending bytes
#define HDRBYTES      0xAAAA
#define EDRBYTES      0x5555

//define Message ID of 5 different messages
#define MSGID_SC      0x0002
#define MSGID_SB      0x0004
#define MSGID_SS      0x060A
#define MSGID_TS      0x070B
#define MSGID_TI      0x070C

//define the buffer
#define HEADER        *(uint16_t *)(buffer)
#define MESSAGE_ID    *(uint16_t *)(buffer + 2)
#define MESSAGE_INFO  *(uint64_t *)(buffer + 4)
#define ENDER         *(uint16_t *)(buffer + 12)

//define sensor_status
//define target status
//#define NO_NoOfTarget *(uint8_t *)(target_status)
//#define NO_RollCount  *(uint8_t *)(target_status + 1)

//define target info
#define TI_Index      *(uint8_t *)(target_info)
#define TI_Rcs        *(uint8_t *)(target_info + 1)
#define TI_RangeH     *(uint8_t *)(target_info + 2)
#define TI_RangeL     *(uint8_t *)(target_info + 3)
#define TI_Azimuth    *(uint8_t *)(target_info + 4)
#define TI_VreIH      *(uint8_t *)(target_info + 5)
#define TI_VreIL      *(uint8_t *)(target_info + 6)
#define TI_SNR        *(uint8_t *)(target_info + 7)




using namespace std;

int main(int argc, char *argv[])
{
	//creat ros handler to node
	ros::init(argc, argv, "serial_radar");
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
	fd.setBaudrate(921600);
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

	ros::Rate rate(60);

	lossync = 0;

	while(ros::ok())
	{
		
		lossync = 0;

		while(lostsync)
		{
			n = 1;
			while(n < 8)
			{
				uint8_t temp_byte;
				fd.read(&temp_byte, 1);
				while(temp_byte == testsync)
				{
					testsync++;
				}
			}
			
		}
		rate.sleep();
	}
	
}

