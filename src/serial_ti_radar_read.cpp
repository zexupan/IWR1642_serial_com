#include "ros/ros.h"
#include "serial/serial.h"
#include "sys/types.h"
#include <math.h>
#include <geometry_msgs/PointStamped.h>

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
//#define tlv_data_targetObjectList_covarianceMatrix[9]     *(float *)(tlv_data_targetObjectList + 28) 
#define tlv_data_targetObjectList_gatingFunctionGain      *(float *)(tlv_data_targetObjectList + 64)



#define tlv_header_type_pointCloud          0x00000006
#define tlv_header_type_targetObjectList    0x00000007
#define tlv_header_type_targetIndex         0x00000008

using namespace std;
geometry_msgs::PointStamped posMsg;
/*geometry_msgs::PointStamped pos1Msg;
geometry_msgs::PointStamped pos2Msg;
geometry_msgs::PointStamped pos3Msg;
geometry_msgs::PointStamped pos4Msg;
geometry_msgs::PointStamped pos5Msg;
geometry_msgs::PointStamped pos6Msg;
geometry_msgs::PointStamped pos7Msg;
geometry_msgs::PointStamped pos8Msg;
geometry_msgs::PointStamped pos9Msg;
geometry_msgs::PointStamped posaMsg;
geometry_msgs::PointStamped posbMsg;
geometry_msgs::PointStamped poscMsg;
geometry_msgs::PointStamped posdMsg;
geometry_msgs::PointStamped poseMsg;
geometry_msgs::PointStamped posfMsg;
geometry_msgs::PointStamped pos10Msg;
geometry_msgs::PointStamped pos11Msg;
geometry_msgs::PointStamped pos12Msg;
geometry_msgs::PointStamped pos13Msg;
geometry_msgs::PointStamped pos14Msg;
geometry_msgs::PointStamped pos15Msg;
geometry_msgs::PointStamped pos16Msg;
geometry_msgs::PointStamped pos17Msg;
geometry_msgs::PointStamped pos18Msg;
geometry_msgs::PointStamped pos19Msg;
geometry_msgs::PointStamped pos1aMsg;
geometry_msgs::PointStamped pos1bMsg;
geometry_msgs::PointStamped pos1cMsg;
geometry_msgs::PointStamped pos1dMsg;
geometry_msgs::PointStamped pos1eMsg;
geometry_msgs::PointStamped pos1fMsg;
*/

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

	ros::Publisher pos0Pub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info0", 1000);
	ros::Publisher pos1Pub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info1", 1000);
	ros::Publisher pos2Pub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info2", 1000);
	ros::Publisher pos3Pub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info3", 1000);
	ros::Publisher pos4Pub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info4", 1000);
	ros::Publisher pos5Pub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info5", 1000);
	ros::Publisher pos6Pub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info6", 1000);
	ros::Publisher pos7Pub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info7", 1000);
	ros::Publisher pos8Pub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info8", 1000);
	ros::Publisher pos9Pub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info9", 1000);
	ros::Publisher posaPub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_infoa", 1000);
	ros::Publisher posbPub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_infob", 1000);
	ros::Publisher poscPub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_infoc", 1000);
	ros::Publisher posdPub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_infod", 1000);
	ros::Publisher posePub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_infoe", 1000);
	ros::Publisher posfPub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_infof", 1000);
	ros::Publisher pos10Pub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info10", 1000);
	ros::Publisher pos11Pub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info11", 1000);
	ros::Publisher pos12Pub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info12", 1000);
	ros::Publisher pos13Pub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info13", 1000);
	ros::Publisher pos14Pub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info14", 1000);
	ros::Publisher pos15Pub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info15", 1000);
	ros::Publisher pos16Pub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info16", 1000);
	ros::Publisher pos17Pub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info17", 1000);
	ros::Publisher pos18Pub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info18", 1000);
	ros::Publisher pos19Pub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info19", 1000);
	ros::Publisher pos1aPub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info1a", 1000);
	ros::Publisher pos1bPub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info1b", 1000);
	ros::Publisher pos1cPub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info1c", 1000);
	ros::Publisher pos1dPub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info1d", 1000);
	ros::Publisher pos1ePub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info1e", 1000);
	ros::Publisher pos1fPub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info1f", 1000);

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

			cout<<"\n\n\n\nframe no.           "<<target_frame_number<<endl<<endl;
			cout<<"\nTLV    no.           "<<frame_header_no_tlv<<endl<<endl;

			for (int i = 0; i < frame_header_no_tlv; i++)
			{
				for (int i = 0; i < 8; i++)
				{
					uint8_t temp_byte;
					fd.read(&temp_byte, 1);
					tlv_header[i] = temp_byte;
					printf("%02x ", tlv_header[i]);
				}
				

				if (tlv_header_type != tlv_header_type_pointCloud && tlv_header_type != tlv_header_type_targetObjectList && tlv_header_type != tlv_header_type_targetIndex)
					{
//						printf("  Header is wrong, try adjust once\n");
						for (int i = 0; i < 7; i++)
						{
							tlv_header[i] = tlv_header[i+1];
						}
						uint8_t temp_byte;
						fd.read(&temp_byte, 1);
						tlv_header[7] = temp_byte;
						for (int i = 0; i < 8; i++)
						{
//							printf("%02x ", tlv_header[i]);
						}
//						printf("\n\n");
					}


				if (tlv_header_type == tlv_header_type_pointCloud)
				{
					printf("TLV header point cloud found\n\n");
					tlv_dataLength = tlv_header_length - 8;
					cout<<endl<<tlv_dataLength<<endl;
					int no_of_pc = tlv_dataLength/16;
					printf("%d\n", no_of_pc);

					for (int i = 0; i < tlv_dataLength; i++)
					{
						uint8_t temp_byte;
						fd.read(&temp_byte, 1);
					}
				}

				else if (tlv_header_type == tlv_header_type_targetObjectList)
				{
					printf("TLV header object list found\n\n");
					tlv_dataLength = tlv_header_length - 8;

					int no_of_objects = tlv_dataLength/68;

					if(no_of_objects > 100)
						continue;

					printf("%d\n", no_of_objects);

					for (int i = 0; i < no_of_objects; i++)
					{
						for (int i = 0; i < 68; i++)
						{
							uint8_t temp_byte;
							fd.read(&temp_byte, 1);
							tlv_data_targetObjectList[i] = temp_byte;
						}
						
//						cout<<"target no.          "<<tlv_data_targetObjectList_trackID<<endl;
						printf("target no.   %02x \n", tlv_data_targetObjectList_trackID);
						cout<<"position     X      "<<tlv_data_targetObjectList_posX<<endl;
						cout<<"position     Y      "<<tlv_data_targetObjectList_posY<<endl;
						cout<<"velocity     X      "<<tlv_data_targetObjectList_velX<<endl;
						cout<<"velocity     Y      "<<tlv_data_targetObjectList_velY<<endl;
						cout<<"acceleration X      "<<tlv_data_targetObjectList_accX<<endl;
						cout<<"acceleration Y      "<<tlv_data_targetObjectList_accY<<endl;

						posMsg.header.stamp = ros::Time::now();
						posMsg.header.frame_id = '1';//tlv_data_targetObjectList_trackID;
						posMsg.point.x = tlv_data_targetObjectList_posX;
						posMsg.point.y = tlv_data_targetObjectList_posY;
						posMsg.point.z = 0.0;

						if (tlv_data_targetObjectList_trackID == 0x00)
						{
							pos0Pub.publish(posMsg);
						}
						if (tlv_data_targetObjectList_trackID == 0x01)
						{
							pos1Pub.publish(posMsg);
						}
						if (tlv_data_targetObjectList_trackID == 0x02)
						{
							pos2Pub.publish(posMsg);
						}
						if (tlv_data_targetObjectList_trackID == 0x03)
						{
							pos3Pub.publish(posMsg);
						}
						if (tlv_data_targetObjectList_trackID == 0x04)
						{
							pos4Pub.publish(posMsg);
						}
						if (tlv_data_targetObjectList_trackID == 0x05)
						{
							pos5Pub.publish(posMsg);
						}
						if (tlv_data_targetObjectList_trackID == 0x06)
						{
							pos6Pub.publish(posMsg);
						}
						if (tlv_data_targetObjectList_trackID == 0x07)
						{
							pos7Pub.publish(posMsg);
						}
						if (tlv_data_targetObjectList_trackID == 0x08)
						{
							pos8Pub.publish(posMsg);
						}
						if (tlv_data_targetObjectList_trackID == 0x09)
						{
							pos9Pub.publish(posMsg);
						}
						if (tlv_data_targetObjectList_trackID == 0x0a)
						{
							posaPub.publish(posMsg);
						}
						if (tlv_data_targetObjectList_trackID == 0x0b)
						{
							posbPub.publish(posMsg);
						}
						if (tlv_data_targetObjectList_trackID == 0x0c)
						{
							poscPub.publish(posMsg);
						}
						if (tlv_data_targetObjectList_trackID == 0x0d)
						{
							posdPub.publish(posMsg);
						}
						if (tlv_data_targetObjectList_trackID == 0x0e)
						{
							posePub.publish(posMsg);
						}
						if (tlv_data_targetObjectList_trackID == 0x0f)
						{
							posfPub.publish(posMsg);
						}
						if (tlv_data_targetObjectList_trackID == 0x10)
						{
							pos10Pub.publish(posMsg);
						}
						if (tlv_data_targetObjectList_trackID == 0x11)
						{
							pos11Pub.publish(posMsg);
						}
						if (tlv_data_targetObjectList_trackID == 0x12)
						{
							pos12Pub.publish(posMsg);
						}
						if (tlv_data_targetObjectList_trackID == 0x13)
						{
							pos13Pub.publish(posMsg);
						}
						if (tlv_data_targetObjectList_trackID == 0x14)
						{
							pos14Pub.publish(posMsg);
						}
						if (tlv_data_targetObjectList_trackID == 0x15)
						{
							pos15Pub.publish(posMsg);
						}
						if (tlv_data_targetObjectList_trackID == 0x16)
						{
							pos16Pub.publish(posMsg);
						}
						if (tlv_data_targetObjectList_trackID == 0x17)
						{
							pos17Pub.publish(posMsg);
						}
						if (tlv_data_targetObjectList_trackID == 0x18)
						{
							pos18Pub.publish(posMsg);
						}
						if (tlv_data_targetObjectList_trackID == 0x19)
						{
							pos19Pub.publish(posMsg);
						}
						if (tlv_data_targetObjectList_trackID == 0x1a)
						{
							pos1aPub.publish(posMsg);
						}
						if (tlv_data_targetObjectList_trackID == 0x1b)
						{
							pos1bPub.publish(posMsg);
						}
						if (tlv_data_targetObjectList_trackID == 0x1c)
						{
							pos1cPub.publish(posMsg);
						}
						if (tlv_data_targetObjectList_trackID == 0x1d)
						{
							pos1dPub.publish(posMsg);
						}
						if (tlv_data_targetObjectList_trackID == 0x1e)
						{
							pos1ePub.publish(posMsg);
						}
						if (tlv_data_targetObjectList_trackID == 0x1f)
						{
							pos1fPub.publish(posMsg);
						}		

					}

					
				}

				else if (tlv_header_type == tlv_header_type_targetIndex)
				{
					printf("TLV header target index found\n\n");
					tlv_dataLength = tlv_header_length - 8;

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
//				printf("found frame header, exit lostsync while loop\n");
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

