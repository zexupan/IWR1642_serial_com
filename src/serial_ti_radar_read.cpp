#include "ros/ros.h"
#include "serial/serial.h"
#include "sys/types.h"
#include <math.h>

//define 5 different messages and the buffer
uint8_t frame_header[52];
uint8_t tlv_header[8];
uint8_t tlv_data_targetObjectList[68]; 

#define frame_header_package_length         *(uint32_t *)(frame_header + 20)
#define frame_header_frame_number           *(uint32_t *)(frame_header + 24)
#define frame_header_no_tlv                 *(uint16_t *)(frame_header + 48)

#define tlv_header_type                     *(uint32_t *)(tlv_header)
#define tlv_header_length                   *(uint32_t *)(tlv_header + 4)

#define tlv_data_targetObjectList_trackID                 *(uint32_t *)(tlv_data_targetObjectList)
#define tlv_data_targetObjectList_posX                    *(float *)(tlv_data_targetObjectList + 4)
#define tlv_data_targetObjectList_posY                    *(float *)(tlv_data_targetObjectList + 8)
#define tlv_data_targetObjectList_velX                    *(float *)(tlv_data_targetObjectList + 12)
#define tlv_data_targetObjectList_velY                    *(float *)(tlv_data_targetObjectList + 16)
#define tlv_data_targetObjectList_accX                    *(float *)(tlv_data_targetObjectList + 20)
#define tlv_data_targetObjectList_accY                    *(float *)(tlv_data_targetObjectList + 24)
#define tlv_data_targetObjectList_covarianceMatrix[9]     *(float *)(tlv_data_targetObjectList + 28) 
#define tlv_data_targetObjectList_gatingFunctionGain      *(float *)(tlv_data_targetObjectList + 64)



#define tlv_header_type_pointCloud          0x00000006
#define tlv_header_type_targetObjectList    0x00000007
#define tlv_header_type_targetIndex         0x00000008

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

	ros::Rate rate(100);

	int lostsync = 1;
	uint8_t testsync[8];

	testsync[0] = 0x02;
	testsync[1] = 0x01;
	testsync[2] = 0x04;
	testsync[3] = 0x03;
	testsync[4] = 0x06;
	testsync[5] = 0x05;
	testsync[6] = 0x08;
	testsync[7] = 0x07;
	

	int got_frame_header = 0;
	int frame_header_bytes = 52;
	uint32_t target_frame_number = 0x00000000;
	uint32_t frame_dataLength;
	uint32_t tlv_dataLength;
	int no_target;
	int no_input_points;

	while(ros::ok())
	{
		while(lostsync == 0)
		{
/*			if (got_frame_header == 0)
			{
				for (int i = 0; i < 52; i++)
				{
					uint8_t temp_byte;
					fd.read(&temp_byte, 1);
					frame_header[i] = temp_byte;
//					printf("%02x ", frame_header[i]);
				}
//				printf("\n\n");
			}

			//check sync pattern
			for (int i = 0; i < 8; i++)
			{
				if(frame_header[i] != testsync[i])
				{
					lostsync = 1;
					break;
				}
			}


			//check sum..................................................................................

*/
			if (got_frame_header == 1)
			{
				//check for new sync
				for (int i = 0; i < 8; i++)
				{
					if (frame_header[i] != testsync[i])
					{
						lostsync = 1;
						got_frame_header = 0;
					}
				}

				if (frame_header_frame_number > target_frame_number)
				{
					target_frame_number = frame_header_frame_number;
					got_frame_header = 0;
				}
				else
				{
					lostsync = 1; //old frame
					got_frame_header = 0;
				}
			}

			if (lostsync){

				break;
			}


			//we got a valid header
			frame_dataLength = frame_header_package_length - 52;
			target_frame_number = frame_header_frame_number;

			no_input_points = 0;
			no_target = 0;

//			for (int i = 0; i < 52; i++)
//			{
//				printf("%02x ", frame_header[i]);
//			}


//			cout<<"frame_dataLength = "<<frame_dataLength<<endl;

			for (int i = 0; i < frame_header_no_tlv; i++)
			{
				for (int i = 0; i < 8; i++)
				{
					uint8_t temp_byte;
					fd.read(&temp_byte, 1);
					tlv_header[i] = temp_byte;
				}

				tlv_dataLength = tlv_header_length - 8;

				if (tlv_header_type == tlv_header_type_pointCloud)
				{
					for (int i = 0; i < tlv_dataLength; i++)
					{
						uint8_t temp_byte;
						fd.read(&temp_byte, 1);
					}
				}

				else if (tlv_header_type == tlv_header_type_targetObjectList)
				{
					for (int i = 0; i < tlv_dataLength; i++)
					{
						uint8_t temp_byte;
						fd.read(&temp_byte, 1);
						tlv_data_targetObjectList[i] = temp_byte;
					}
					cout<<"frame no.           "<<target_frame_number<<endl<<endl;
					cout<<"target no.          "<<tlv_data_targetObjectList_trackID<<endl;
					cout<<"position     X      "<<tlv_data_targetObjectList_posX<<endl;
					cout<<"position     Y      "<<tlv_data_targetObjectList_posY<<endl;
					cout<<"velocity     X      "<<tlv_data_targetObjectList_velX<<endl;
					cout<<"velocity     Y      "<<tlv_data_targetObjectList_velY<<endl;
					cout<<"acceleration X      "<<tlv_data_targetObjectList_accX<<endl;
					cout<<"acceleration Y      "<<tlv_data_targetObjectList_accY<<endl;
				}

				else if (tlv_header_type == tlv_header_type_targetIndex)
				{
					for (int i = 0; i < tlv_dataLength; i++)
					{
						uint8_t temp_byte;
						fd.read(&temp_byte, 1);
					}
				}
				else
				{
					cout<<"TLV header wrong, lost sync at frame = "<<target_frame_number<<endl;
					lostsync = 1;
				}
			}

//			for (int i = 0; i < frame_dataLength; i++)
//			{
//				uint8_t temp_byte;
//				fd.read(&temp_byte, 1);
//			}
//			cout<<"finished reading data at frame "<<target_frame_number<<endl;
//			double secs =ros::Time::now().toSec();
//			printf("%f\n", secs);
			lostsync = 1;
		}

		while(lostsync)
		{
			//Looking for sync data for the header
			int n = 0;
			while(n < 8)
			{
				uint8_t temp_byte;
				fd.read(&temp_byte, 1);
				if(temp_byte == testsync[n])
				{
					frame_header[n] = temp_byte;
					n++;
				}
				else n = 0;
			}

			//Header is found, sync back
			if (n == 8)
			{
				printf("found frame header, exit lostsync while loop\n");
				lostsync = 0;
				//read header, 52-8 bytes
				for (int i = 8; i < 52; i++)
				{
					uint8_t temp_byte;
					fd.read(&temp_byte, 1);
					frame_header[i] = temp_byte;
				}
				got_frame_header = 1;
			}
		}
		rate.sleep();
	}
	
}

