#include "ros/ros.h"
#include "serial/serial.h"
#include "sys/types.h"
#include <math.h>
#include <geometry_msgs/TwistStamped.h>

//define 5 different messages and the buffer
uint8_t sensor_configuration[8];
uint8_t sensor_back[8];
uint8_t sensor_status[8];
uint8_t target_status[8];
uint8_t target_info[8];
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



geometry_msgs::TwistStamped velDMsg;
ros::Publisher velDPub;

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

	ros::Rate rate(60);
	velDPub = serial_radar_nh.advertise<geometry_msgs::TwistStamped>("/radarBAG", 10);

	float Rcs_min = 0;
	float Range_min = 1000;
	float Vrel_min = 0;
	float Azimuth_min = 0;
	float SNR_min = 0;
	float Rcs_1;
	float Range_1;
	float Azimuth_1;
	float Vrel_1;
	float SNR_1;
	float Rcs_2;
	float Range_2;
	float Azimuth_2;
	float Vrel_2;
	float SNR_2;
	float Rcs_3;
	float Range_3;
	float Azimuth_3;
	float Vrel_3;
	float SNR_3;
	float Rcs_4;
	float Range_4;
	float Azimuth_4;
	float Vrel_4;
	float SNR_4;
	float Rcs_5;
	float Range_5;
	float Azimuth_5;
	float Vrel_5;
	float SNR_5;
	float Rcs_extra;
	float Range_extra;
	float Azimuth_extra;
	float Vrel_extra;
	float SNR_extra;
	uint8_t No_of_targets;
	float range_list[10];
    uint8_t range_key[10];	


	while(ros::ok())
	{
		uint32_t bytes_in_buff = fd.available();
		uint32_t read_bytes = 0;



		if (bytes_in_buff > 0)
		{
			while(read_bytes < bytes_in_buff)
			{
				uint8_t temp_byte;
				fd.read(&temp_byte, 1);

				for (int i = 0; i < 13; i++)
					buffer[i] = buffer[i+1];
				
				buffer[13] = temp_byte;
				read_bytes++;
//				printf("0x%x\n", temp_byte);

				if (HEADER == HDRBYTES && ENDER == EDRBYTES)
				{
//					printf("Message ID : 0x%x\n", MESSAGE_ID);

//					if(MESSAGE_ID == MSGID_SS)
//					{
//
//					}

					if(MESSAGE_ID == MSGID_TS)
					{
						
						ros::Time listeningtime = ros::Time::now();
						printf("\n\n\n\n\n\n\n");
						cout<<"Message received time: "<<listeningtime<<endl;
						printf("Target status:");
						printf("            No of targets:   %x\n", buffer[4]);
						printf("                          Rollcount:     %x\n\n", buffer[5]);

						No_of_targets = buffer[4];
						if(No_of_targets == 0)
						{
							Rcs_min = 0;
							Range_min = 100;
							Vrel_min = 0;
							Azimuth_min = 0;
							SNR_min = 0;

						}
							
					}
						

					else if(MESSAGE_ID == MSGID_TI)
					{
						for(int k = 0; k < 8; k++)
							target_info[k] = buffer[k+4];

						
						if(TI_Index == 1)
						{
							Rcs_1 = TI_Rcs*0.5 - 50;
							Range_1 = (TI_RangeH*256 + TI_RangeL)*0.01;
							Azimuth_1 = TI_Azimuth*2 - 90;
							Vrel_1 = (TI_VreIH*256 + TI_VreIL)*0.05 - 35;
							SNR_1 = TI_SNR - 127;
							printf("Target %x info:", TI_Index);
							printf("            Reflected area_1:      %3.2f\n", Rcs_1);
							printf("                          Distance_1:            %3.2f\n", Range_1);
							printf("                          Angle_1:               %3.2f\n", Azimuth_1);
							printf("                          Relative speed_1:      %3.2f\n", Vrel_1);
							printf("                          Signal noise ratio_1:  %3.2f\n\n", SNR_1);
							
							range_list[TI_Index-1]= Range_1;
							range_key[TI_Index-1] = TI_Index;
							if(No_of_targets == TI_Index)
							{
								Rcs_min = Rcs_1;
								Range_min = Range_1;
								Azimuth_min = Azimuth_1;
								Vrel_min = Vrel_1;
								SNR_min = SNR_1;
								printf("Target_min info: ");
								printf("         Reflected area_min:      %3.2f\n", Rcs_min);
								printf("                          Distance_min:            %3.2f\n", Range_min);
								printf("                          Angle_min:               %3.2f\n", Azimuth_min);
								printf("                          Relative speed_min:      %3.2f\n", Vrel_min);
								printf("                          Signal noise ratio:  %3.2f\n\n", SNR_min);
								velDMsg.header.stamp = ros::Time::now();
							    velDMsg.twist.linear.x = Range_min;
							    velDMsg.twist.linear.y = Azimuth_min;
							    velDMsg.twist.linear.z = 0.0;
							    velDPub.publish(velDMsg);
							}
						}
						else if(TI_Index == 2)
						{
							Rcs_2 = TI_Rcs*0.5 - 50;
							Range_2 = (TI_RangeH*256 + TI_RangeL)*0.01;
							Azimuth_2 = TI_Azimuth*2 - 90;
							Vrel_2 = (TI_VreIH*256 + TI_VreIL)*0.05 - 35;
							SNR_2 = TI_SNR - 127;
							printf("Target %x info:", TI_Index);
							printf("            Reflected area_2:      %3.2f\n", Rcs_2);
							printf("                          Distance_2:            %3.2f\n", Range_2);
							printf("                          Angle_2:               %3.2f\n", Azimuth_2);
							printf("                          Relative speed_2:      %3.2f\n", Vrel_2);
							printf("                          Signal noise ratio_2:  %3.2f\n\n", SNR_2);
							
							range_list[TI_Index-1]= Range_2;
							range_key[TI_Index-1] = TI_Index;
							if(No_of_targets == TI_Index)
							{
								if(Range_1 < Range_2)
								{
									Rcs_min = Rcs_1;
									Range_min = Range_1;
									Azimuth_min = Azimuth_1;
									Vrel_min = Vrel_1;
									SNR_min = SNR_1;
								}
								else
								{
									Rcs_min = Rcs_2;
									Range_min = Range_2;
									Azimuth_min = Azimuth_2;
									Vrel_min = Vrel_2;
									SNR_min = SNR_2;
								}
								printf("Target_min info: ");
								printf("         Reflected area_min:      %3.2f\n", Rcs_min);
								printf("                          Distance_min:            %3.2f\n", Range_min);
								printf("                          Angle_min:               %3.2f\n", Azimuth_min);
								printf("                          Relative speed_min:      %3.2f\n", Vrel_min);
								printf("                          Signal noise ratio:  %3.2f\n\n", SNR_min);
								velDMsg.header.stamp = ros::Time::now();
							    velDMsg.twist.linear.x = Range_min;
							    velDMsg.twist.linear.y = Azimuth_min;
							    velDMsg.twist.linear.z = 0.0;
							    velDPub.publish(velDMsg);
							}
						}
						else if(TI_Index == 3)
						{
							Rcs_3 = TI_Rcs*0.5 - 50;
							Range_3 = (TI_RangeH*256 + TI_RangeL)*0.01;
							Azimuth_3 = TI_Azimuth*2 - 90;
							Vrel_3 = (TI_VreIH*256 + TI_VreIL)*0.05 - 35;
							SNR_3 = TI_SNR - 127;
							printf("Target %x info:", TI_Index);
							printf("            Reflected area_3:      %3.2f\n", Rcs_3);
							printf("                          Distance_3:            %3.2f\n", Range_3);
							printf("                          Angle_3:               %3.2f\n", Azimuth_3);
							printf("                          Relative speed_3:      %3.2f\n", Vrel_3);
							printf("                          Signal noise ratio_3:  %3.2f\n\n", SNR_3);
							
							range_list[TI_Index-1]= Range_3;
							range_key[TI_Index-1] = TI_Index;
							if(No_of_targets == TI_Index)
							{
								for(int i = 0; i<No_of_targets-1;i++)
								{
									for(int j = 0; j<No_of_targets-i-1; j++)
									{
										if(range_list[j] > range_list[j+1])
										{
											float temp_range = range_list[j];
											range_list[j] = range_list[j+1];
											range_list[j+1] = temp_range;
											float temp_key = range_key[j];
											range_key[j] = range_key[j+1];
											range_key[j+1] = temp_key;
										}
									}
								}

								switch(range_key[0])
								{
									case 1: Rcs_min = Rcs_1;
											Range_min = Range_1;
											Azimuth_min = Azimuth_1;
											Vrel_min = Vrel_1;
											SNR_min = SNR_1;
											break;

									case 2: Rcs_min = Rcs_2;
											Range_min = Range_2;
											Azimuth_min = Azimuth_2;
											Vrel_min = Vrel_2;
											SNR_min = SNR_2;
											break;

									case 3: Rcs_min = Rcs_3;
											Range_min = Range_3;
											Azimuth_min = Azimuth_3;
											Vrel_min = Vrel_3;
											SNR_min = SNR_3;
											break;

									case 4: Rcs_min = Rcs_4;
											Range_min = Range_4;
											Azimuth_min = Azimuth_4;
											Vrel_min = Vrel_4;
											SNR_min = SNR_4;
											break;

									case 5: Rcs_min = Rcs_5;
											Range_min = Range_5;
											Azimuth_min = Azimuth_5;
											Vrel_min = Vrel_5;
											SNR_min = SNR_5;
											break;

								}
								printf("Target_min info: ");
								printf("         Reflected area_min:      %3.2f\n", Rcs_min);
								printf("                          Distance_min:            %3.2f\n", Range_min);
								printf("                          Angle_min:               %3.2f\n", Azimuth_min);
								printf("                          Relative speed_min:      %3.2f\n", Vrel_min);
								printf("                          Signal noise ratio:  %3.2f\n\n", SNR_min);
								velDMsg.header.stamp = ros::Time::now();
							    velDMsg.twist.linear.x = Range_min;
							    velDMsg.twist.linear.y = Azimuth_min;
							    velDMsg.twist.linear.z = 0.0;
							    velDPub.publish(velDMsg);

							}
							
						}
						else if(TI_Index == 4)
						{
							Rcs_4 = TI_Rcs*0.5 - 50;
							Range_4 = (TI_RangeH*256 + TI_RangeL)*0.01;
							Azimuth_4 = TI_Azimuth*2 - 90;
							Vrel_4 = (TI_VreIH*256 + TI_VreIL)*0.05 - 35;
							SNR_4 = TI_SNR - 127;
							printf("Target %x info:", TI_Index);
							printf("            Reflected area_4:      %3.2f\n", Rcs_4);
							printf("                          Distance_4:            %3.2f\n", Range_4);
							printf("                          Angle_4:               %3.2f\n", Azimuth_4);
							printf("                          Relative speed_4:      %3.2f\n", Vrel_4);
							printf("                          Signal noise ratio_4:  %3.2f\n\n", SNR_4);
							
							range_list[TI_Index-1]= Range_4;
							range_key[TI_Index-1] = TI_Index;
							if(No_of_targets == TI_Index)
							{
								for(int i = 0; i<No_of_targets-1;i++)
								{
									for(int j = 0; j<No_of_targets-i-1; j++)
									{
										if(range_list[j] > range_list[j+1])											{
											float temp_range = range_list[j];
											range_list[j] = range_list[j+1];
											range_list[j+1] = temp_range;
											float temp_key = range_key[j];
											range_key[j] = range_key[j+1];
											range_key[j+1] = temp_key;
										}
									}
								}
								switch(range_key[0])
								{
									case 1: Rcs_min = Rcs_1;
											Range_min = Range_1;
											Azimuth_min = Azimuth_1;
											Vrel_min = Vrel_1;
											SNR_min = SNR_1;
											break;

									case 2: Rcs_min = Rcs_2;
											Range_min = Range_2;
											Azimuth_min = Azimuth_2;
											Vrel_min = Vrel_2;
											SNR_min = SNR_2;
											break;

									case 3: Rcs_min = Rcs_3;
											Range_min = Range_3;
											Azimuth_min = Azimuth_3;
											Vrel_min = Vrel_3;
											SNR_min = SNR_3;
											break;

									case 4: Rcs_min = Rcs_4;
											Range_min = Range_4;
											Azimuth_min = Azimuth_4;
											Vrel_min = Vrel_4;
											SNR_min = SNR_4;
											break;

									case 5: Rcs_min = Rcs_5;
											Range_min = Range_5;
											Azimuth_min = Azimuth_5;
											Vrel_min = Vrel_5;
											SNR_min = SNR_5;
											break;

								}

								printf("Target_min info: ");
								printf("         Reflected area_min:      %3.2f\n", Rcs_min);
								printf("                          Distance_min:            %3.2f\n", Range_min);
								printf("                          Angle_min:               %3.2f\n", Azimuth_min);
								printf("                          Relative speed_min:      %3.2f\n", Vrel_min);
								printf("                          Signal noise ratio:  %3.2f\n\n", SNR_min);
								velDMsg.header.stamp = ros::Time::now();
							    velDMsg.twist.linear.x = Range_min;
							    velDMsg.twist.linear.y = Azimuth_min;
							    velDMsg.twist.linear.z = 0.0;
							    velDPub.publish(velDMsg);	
							}
							
						}
						else if(TI_Index == 5)
						{
							Rcs_5 = TI_Rcs*0.5 - 50;
							Range_5 = (TI_RangeH*256 + TI_RangeL)*0.01;
							Azimuth_5 = TI_Azimuth*2 - 90;
							Vrel_5 = (TI_VreIH*256 + TI_VreIL)*0.05 - 35;
							SNR_5 = TI_SNR - 127;
							printf("Target %x info:", TI_Index);
							printf("            Reflected area_5:      %3.2f\n", Rcs_5);
							printf("                          Distance_5:            %3.2f\n", Range_5);
							printf("                          Angle_5:               %3.2f\n", Azimuth_5);
							printf("                          Relative speed_5:      %3.2f\n", Vrel_5);
							printf("                          Signal noise ratio:  %3.2f\n\n", SNR_5);

							range_list[TI_Index-1]= Range_5;
							range_key[TI_Index-1] = TI_Index;
							if(TI_Index == 5)
							{
								for(int i = 0; i<No_of_targets-1;i++)
								{
									for(int j = 0; j<No_of_targets-i-1; j++)
									{
										if(range_list[j] > range_list[j+1])											{
											float temp_range = range_list[j];
											range_list[j] = range_list[j+1];
											range_list[j+1] = temp_range;
											float temp_key = range_key[j];
											range_key[j] = range_key[j+1];
											range_key[j+1] = temp_key;
										}
									}
								}
								switch(range_key[0])
								{
									case 1: Rcs_min = Rcs_1;
											Range_min = Range_1;
											Azimuth_min = Azimuth_1;
											Vrel_min = Vrel_1;
											SNR_min = SNR_1;
											break;

									case 2: Rcs_min = Rcs_2;
											Range_min = Range_2;
											Azimuth_min = Azimuth_2;
											Vrel_min = Vrel_2;
											SNR_min = SNR_2;
											break;

									case 3: Rcs_min = Rcs_3;
											Range_min = Range_3;
											Azimuth_min = Azimuth_3;
											Vrel_min = Vrel_3;
											SNR_min = SNR_3;
											break;

									case 4: Rcs_min = Rcs_4;
											Range_min = Range_4;
											Azimuth_min = Azimuth_4;
											Vrel_min = Vrel_4;
											SNR_min = SNR_4;
											break;

									case 5: Rcs_min = Rcs_5;
											Range_min = Range_5;
											Azimuth_min = Azimuth_5;
											Vrel_min = Vrel_5;
											SNR_min = SNR_5;
											break;

								}

								printf("Target_min info: ");
								printf("         Reflected area_min:      %3.2f\n", Rcs_min);
								printf("                          Distance_min:            %3.2f\n", Range_min);
								printf("                          Angle_min:               %3.2f\n", Azimuth_min);
								printf("                          Relative speed_min:      %3.2f\n", Vrel_min);
								printf("                          Signal noise ratio:  %3.2f\n\n", SNR_min);
								velDMsg.header.stamp = ros::Time::now();
							    velDMsg.twist.linear.x = Range_min;
							    velDMsg.twist.linear.y = Azimuth_min;
							    velDMsg.twist.linear.z = 0.0;
							    velDPub.publish(velDMsg);	
							}
							
						}
						else
						{
							Rcs_extra = TI_Rcs*0.5 - 50;
							Range_extra = (TI_RangeH*256 + TI_RangeL)*0.01;
							Azimuth_extra = TI_Azimuth*2 - 90;
							Vrel_extra = (TI_VreIH*256 + TI_VreIL)*0.05 - 35;
							SNR_extra = TI_SNR - 127;
							printf("Target %x info:", TI_Index);
							printf("            Reflected area:      %3.2f\n", Rcs_extra);
							printf("                          Distance:            %3.2f\n", Range_extra);
							printf("                          Angle:               %3.2f\n", Azimuth_extra);
							printf("                          Relative speed:      %3.2f\n", Vrel_extra);
							printf("                          Signal noise ratio:  %3.2f\n\n", SNR_extra);
							if(Range_min > Range_extra)
							{
								Rcs_min = Rcs_extra;
								Range_min = Range_extra;
								Azimuth_min = Azimuth_extra;
								Vrel_min = Vrel_extra;
								SNR_min = SNR_extra;
								printf("Target_min info: ");
								printf("         Reflected area_min:      %3.2f\n", Rcs_min);
								printf("                          Distance_min:            %3.2f\n", Range_min);
								printf("                          Angle_min:               %3.2f\n", Azimuth_min);
								printf("                          Relative speed_min:      %3.2f\n", Vrel_min);
								printf("                          Signal noise ratio:  %3.2f\n\n", SNR_min);
								velDMsg.header.stamp = ros::Time::now();
							    velDMsg.twist.linear.x = Range_min;
							    velDMsg.twist.linear.y = Azimuth_min;
							    velDMsg.twist.linear.z = 0.0;
							    velDPub.publish(velDMsg);
							}
						}

						
					}					
				}
			}
		}
		rate.sleep();
	}
}

