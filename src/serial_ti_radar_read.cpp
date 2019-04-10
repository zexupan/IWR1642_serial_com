#include "ros/ros.h"
#include "serial/serial.h"
#include "sys/types.h"
#include <math.h>
#include <geometry_msgs/PointStamped.h>

//define 5 different messages and the buffer
uint8_t frame_header[52];
uint8_t tlv_header[8];
uint8_t tlv_data_targetObjectList[68];
double object_list[32][6];
int object_list_index[32];
double object_list_range[32];

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
#define tlv_data_targetObjectList_gatingFunctionGain      *(float *)(tlv_data_targetObjectList + 64)



#define tlv_header_type_pointCloud          0x00000006
#define tlv_header_type_targetObjectList    0x00000007
#define tlv_header_type_targetIndex         0x00000008

using namespace std;
geometry_msgs::PointStamped posMsg;


bool inverse4x4(float m[],float invOut[])
{
    float inv[16], det;
    int i;


    inv[0] = m[5]  * m[10] * m[15] -
    m[5]  * m[11] * m[14] -
    m[9]  * m[6]  * m[15] +
    m[9]  * m[7]  * m[14] +
    m[13] * m[6]  * m[11] -
    m[13] * m[7]  * m[10];

    inv[4] = -m[4]  * m[10] * m[15] +
    m[4]  * m[11] * m[14] +
    m[8]  * m[6]  * m[15] -
    m[8]  * m[7]  * m[14] -
    m[12] * m[6]  * m[11] +
    m[12] * m[7]  * m[10];

    inv[8] = m[4]  * m[9] * m[15] -
    m[4]  * m[11] * m[13] -
    m[8]  * m[5] * m[15] +
    m[8]  * m[7] * m[13] +
    m[12] * m[5] * m[11] -
    m[12] * m[7] * m[9];

    inv[12] = -m[4]  * m[9] * m[14] +
    m[4]  * m[10] * m[13] +
    m[8]  * m[5] * m[14] -
    m[8]  * m[6] * m[13] -
    m[12] * m[5] * m[10] +
    m[12] * m[6] * m[9];

    inv[1] = -m[1]  * m[10] * m[15] +
    m[1]  * m[11] * m[14] +
    m[9]  * m[2] * m[15] -
    m[9]  * m[3] * m[14] -
    m[13] * m[2] * m[11] +
    m[13] * m[3] * m[10];

    inv[5] = m[0]  * m[10] * m[15] -
    m[0]  * m[11] * m[14] -
    m[8]  * m[2] * m[15] +
    m[8]  * m[3] * m[14] +
    m[12] * m[2] * m[11] -
    m[12] * m[3] * m[10];

    inv[9] = -m[0]  * m[9] * m[15] +
    m[0]  * m[11] * m[13] +
    m[8]  * m[1] * m[15] -
    m[8]  * m[3] * m[13] -
    m[12] * m[1] * m[11] +
    m[12] * m[3] * m[9];

    inv[13] = m[0]  * m[9] * m[14] -
    m[0]  * m[10] * m[13] -
    m[8]  * m[1] * m[14] +
    m[8]  * m[2] * m[13] +
    m[12] * m[1] * m[10] -
    m[12] * m[2] * m[9];

    inv[2] = m[1]  * m[6] * m[15] -
    m[1]  * m[7] * m[14] -
    m[5]  * m[2] * m[15] +
    m[5]  * m[3] * m[14] +
    m[13] * m[2] * m[7] -
    m[13] * m[3] * m[6];

    inv[6] = -m[0]  * m[6] * m[15] +
    m[0]  * m[7] * m[14] +
    m[4]  * m[2] * m[15] -
    m[4]  * m[3] * m[14] -
    m[12] * m[2] * m[7] +
    m[12] * m[3] * m[6];

    inv[10] = m[0]  * m[5] * m[15] -
    m[0]  * m[7] * m[13] -
    m[4]  * m[1] * m[15] +
    m[4]  * m[3] * m[13] +
    m[12] * m[1] * m[7] -
    m[12] * m[3] * m[5];

    inv[14] = -m[0]  * m[5] * m[14] +
    m[0]  * m[6] * m[13] +
    m[4]  * m[1] * m[14] -
    m[4]  * m[2] * m[13] -
    m[12] * m[1] * m[6] +
    m[12] * m[2] * m[5];

    inv[3] = -m[1] * m[6] * m[11] +
    m[1] * m[7] * m[10] +
    m[5] * m[2] * m[11] -
    m[5] * m[3] * m[10] -
    m[9] * m[2] * m[7] +
    m[9] * m[3] * m[6];

    inv[7] = m[0] * m[6] * m[11] -
    m[0] * m[7] * m[10] -
    m[4] * m[2] * m[11] +
    m[4] * m[3] * m[10] +
    m[8] * m[2] * m[7] -
    m[8] * m[3] * m[6];

    inv[11] = -m[0] * m[5] * m[11] +
    m[0] * m[7] * m[9] +
    m[4] * m[1] * m[11] -
    m[4] * m[3] * m[9] -
    m[8] * m[1] * m[7] +
    m[8] * m[3] * m[5];

    inv[15] = m[0] * m[5] * m[10] -
    m[0] * m[6] * m[9] -
    m[4] * m[1] * m[10] +
    m[4] * m[2] * m[9] +
    m[8] * m[1] * m[6] -
    m[8] * m[2] * m[5];

    det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

    if (det == 0) {
            printf("matrix inversion failed `");
            return false;
        }

    det = 1.0f / det;

    for (i = 0; i < 16; i++)
        invOut[i] = inv[i] * det;
    return true;
}

double Est_Target_Position_x;
double Est_Target_Position_y;
double Est_Target_Velocity_x;
double Est_Target_Velocity_y;
float dt_est = 0.05;
float Mea_Pos_x_err = 0.5;
float Mea_Pos_y_err = 0.5;
float Mea_Vel_x_err = 0.5;
float Mea_Vel_y_err = 0.5;
float Pro_Pos_x_err = 5;
float Pro_Pos_y_err = 5;
float sum;
float Four_Four_temp_00[16];
float Four_Four_temp_11[16];



float X_now_1[4][1]=
{
    {0},
    {0},
    {0},
    {0}

};

float X_now[4][1]=
{
    {0},
    {0},
    {0},
    {0}
};

float X_now_kp[4][1]=
{
    {0},
    {0},
    {0},
    {0}
};


float Y_mea[4][1]=
{
    {0},
    {0},
    {0},
    {0}
};

float Matrix_A[4][4]=
{
    {1, 0, dt_est, 0},
    {0, 1, 0, dt_est},
    {0, 0, 1, 0},
    {0, 0, 0, 1}
};

float Matrix_A_T[4][4]=
{
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {dt_est, 0, 1, 0},
    {0, dt_est, 0, 1}
};

//Measured Covariance Matrix
float R_now[4][4]=
{
    {Mea_Pos_x_err*Mea_Pos_x_err, 0, 0, 0},
    {0, Mea_Pos_y_err*Mea_Pos_y_err, 0, 0},
    {0, 0,Mea_Vel_x_err*Mea_Vel_x_err, 0},
    {0, 0, 0, Mea_Vel_y_err*Mea_Vel_y_err}
};

//Predicted Covariance matrix
float Q_now[4][4]=
{
    {Pro_Pos_x_err*Pro_Pos_x_err*dt_est*dt_est*dt_est*dt_est/4,0,Pro_Pos_x_err*Pro_Pos_x_err*dt_est*dt_est*dt_est/2,0},
    {0, Pro_Pos_y_err*Pro_Pos_y_err*dt_est*dt_est*dt_est*dt_est/4, 0, Pro_Pos_y_err*Pro_Pos_y_err*dt_est*dt_est*dt_est/2},
    {Pro_Pos_x_err*Pro_Pos_x_err*dt_est*dt_est/2, 0, Pro_Pos_x_err*Pro_Pos_x_err*dt_est, 0},
    {0, Pro_Pos_y_err*Pro_Pos_y_err*dt_est*dt_est/2, 0, Pro_Pos_y_err*Pro_Pos_y_err*dt_est}
};

float P_now_kp[4][4]=
{
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0}
};

float P_now_1[4][4]=
{
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0}
};

float Kalman_Gain[4][4]=
{
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0}
};

float Kalman_Gain_T[4][4]=
{
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0}
};

float Four_Four_temp[4][4]=
{
    {0,0,0,0},
    {0,0,0,0},
    {0,0,0,0},
    {0,0,0,0}
};

float Four_Four_temp_1[4][4]=
{
    {0,0,0,0},
    {0,0,0,0},
    {0,0,0,0},
    {0,0,0,0}
};

float Four_One_temp[4][1]=
{
    {0},
    {0},
    {0},
    {0}
};

float Four_Four_I[4][4]=
{
    {1,0,0,0},
    {0,1,0,0},
    {0,0,1,0},
    {0,0,0,1}
};


int radar_estimator(double Pos_x, double Pos_y, double Vel_x, double Vel_y)
{

    //Compute the processed state
    sum = 0;
    for(int c=0; c<4;c++){
         for(int d=0; d<1; d++){
            for(int k=0; k<4; k++){
                sum= sum + Matrix_A[c][k] * X_now_1[k][d];
            }
            X_now_kp[c][d]= sum;
            sum = 0;
        }

    }

    //Compute Pkp Covariance Matrix
    sum = 0;
    for(int c=0; c<4; c++){
        for(int d=0; d<4; d++){
            for(int k=0; k<4; k++){
                sum= sum + Matrix_A[c][k] * P_now_1[k][d];
            }
            Four_Four_temp[c][d]= sum;
            sum = 0;
        }

    }

    sum = 0;
    for(int c=0; c<4; c++){
        for(int d=0; d<4; d++){
            for(int k=0; k<4; k++){
                sum= sum + Four_Four_temp[c][k] * Matrix_A_T[k][d];
            }
            P_now_kp[c][d]= sum;
            sum = 0;
        }
    }

    for(int c=0; c<4; c++){
        for(int d=0; d<4; d++){
            P_now_kp[c][d] = P_now_kp[c][d] + Q_now[c][d];
        }
    }


    //Compute the Kalman Gain
    for(int c=0; c<4; c++){
        for(int d=0; d<4; d++){
            Four_Four_temp[c][d] = R_now[c][d] + P_now_kp[c][d];
        }
    }

    Four_Four_temp_00[0] = Four_Four_temp[0][0];
    Four_Four_temp_00[1] = Four_Four_temp[0][1];
    Four_Four_temp_00[2] = Four_Four_temp[0][2];
    Four_Four_temp_00[3] = Four_Four_temp[0][3];
    Four_Four_temp_00[4] = Four_Four_temp[1][0];
    Four_Four_temp_00[5] = Four_Four_temp[1][1];
    Four_Four_temp_00[6] = Four_Four_temp[1][2];
    Four_Four_temp_00[7] = Four_Four_temp[1][3];
    Four_Four_temp_00[8] = Four_Four_temp[2][02];
    Four_Four_temp_00[9] = Four_Four_temp[2][1];
    Four_Four_temp_00[10] = Four_Four_temp[2][2];
    Four_Four_temp_00[11] = Four_Four_temp[2][3];
    Four_Four_temp_00[12] = Four_Four_temp[3][0];
    Four_Four_temp_00[13] = Four_Four_temp[3][1];
    Four_Four_temp_00[14] = Four_Four_temp[3][2];
    Four_Four_temp_00[15] = Four_Four_temp[3][3];

    if(inverse4x4(Four_Four_temp_00, Four_Four_temp_11) == false)
        {
            for(int i=0; i<4; i++)
                X_now[i][0] = X_now_kp[i][0];
            printf("matrix inverse failed\n");
            return false;
        }

    Four_Four_temp[0][0] = Four_Four_temp_11[0];
    Four_Four_temp[0][1] = Four_Four_temp_11[1];
    Four_Four_temp[0][2] = Four_Four_temp_11[2];
    Four_Four_temp[0][3] = Four_Four_temp_11[3];
    Four_Four_temp[1][0] = Four_Four_temp_11[4];
    Four_Four_temp[1][1] = Four_Four_temp_11[5];
    Four_Four_temp[1][2] = Four_Four_temp_11[6];
    Four_Four_temp[1][3] = Four_Four_temp_11[7];
    Four_Four_temp[2][0] = Four_Four_temp_11[8];
    Four_Four_temp[2][1] = Four_Four_temp_11[9];
    Four_Four_temp[2][2] = Four_Four_temp_11[10];
    Four_Four_temp[2][3] = Four_Four_temp_11[11];
    Four_Four_temp[3][0] = Four_Four_temp_11[12];
    Four_Four_temp[3][1] = Four_Four_temp_11[13];
    Four_Four_temp[3][2] = Four_Four_temp_11[14];
    Four_Four_temp[3][3] = Four_Four_temp_11[15];


    sum = 0;
    for(int c=0; c<4;c++){
        for(int d=0; d<4; d++){
            for(int k=0; k<4; k++){
                sum= sum + P_now_kp[c][k] * Four_Four_temp[k][d];
            }
            Kalman_Gain[c][d]= sum;
            sum = 0;
        }
    }



    //compute the filtered state
    Y_mea[0][0] = Pos_x;
    Y_mea[1][0] = Pos_y;
    Y_mea[2][0] = Vel_x;
    Y_mea[3][0] = Vel_y;

    for(int c=0; c<4; c++){
        for(int d=0; d<1; d++){
            Four_One_temp[c][d] = Y_mea[c][d] - X_now_kp[c][d];
        }
    }

    sum = 0;
    for(int c=0; c<4;c++){
        for(int d=0; d<1; d++){
            for(int k=0; k<4; k++){
                sum= sum + Kalman_Gain[c][k] * Four_One_temp[k][d];
            }
            X_now[c][d]= sum + X_now_kp[c][d];
            sum = 0;
        }
    }



    Est_Target_Position_x = X_now[0][0];
    Est_Target_Position_y = X_now[1][0];
    Est_Target_Velocity_x = X_now[2][0];
    Est_Target_Velocity_y = X_now[3][0];

    //update covariance matrix
    for(int c=0; c<4; c++){
        for(int d=0; d<4; d++){
            Four_Four_temp[c][d] = Four_Four_I[c][d] -  Kalman_Gain[c][d];
        }
    }

    sum = 0;
    for(int c=0; c<4;c++){
        for(int d=0; d<4; d++){
            for(int k=0; k<4; k++){
                sum= sum + Four_Four_temp[c][k] * P_now_kp[k][d];
            }
            P_now_1[c][d]= sum;
            sum = 0;
        }
    }

    for(int c=0; c<4; c++){
        for(int d=0; d<c; d++){
            float temp = Four_Four_temp[d][c];
            Four_Four_temp[d][c] = Four_Four_temp[c][d];
            Four_Four_temp[c][d] = temp;
        }
    }

    sum = 0;
    for(int c=0; c<4;c++){
        for(int d=0; d<4; d++){
            for(int k=0; k<4; k++){
                sum= sum + P_now_1[c][k] * Four_Four_temp[k][d];
            }
            Four_Four_temp_1[c][d]= sum;
            sum = 0;
        }
    }

    sum = 0;
    for(int c=0; c<4;c++){
        for(int d=0; d<4; d++){
            for(int k=0; k<4; k++){
                sum= sum + Kalman_Gain[c][k] * R_now[k][d];
            }
            Four_Four_temp[c][d]= sum;
            sum = 0;
        }
    }

    for(int c=0; c<4; c++){
        for(int d=0; d<4; d++){
            Kalman_Gain_T[c][d] = Kalman_Gain[d][c];
        }
    }

    sum = 0;
    for(int c=0; c<4;c++){
        for(int d=0; d<4; d++){
            for(int k=0; k<4; k++){
                sum= sum + Four_Four_temp[c][k] * Kalman_Gain_T[k][d];
            }
            P_now_1[c][d]= sum + Four_Four_temp_1[c][d];
            sum = 0;
        }
    }

    //update the state matrix
    for(int c=0; c<4; c++){
        for(int d=0; d<1; d++){
            X_now_1[c][d] = X_now[c][d];
        }
    }

    return 1;
}





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

	ros::Rate rate(20);

    ros::Publisher posPub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info", 1000);
	ros::Publisher pos0Pub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info0", 1000);
	ros::Publisher pos1Pub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info1", 1000);
    ros::Publisher pos2Pub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info2", 1000);
    ros::Publisher pos3Pub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info3", 1000);
    ros::Publisher pos4Pub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info4", 1000);
    ros::Publisher pos5Pub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info5", 1000);
    ros::Publisher pos6Pub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info6", 1000);
    ros::Publisher pos7Pub = serial_radar_nh.advertise<geometry_msgs::PointStamped>("radar_info7", 1000);


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
	int last_track_ID = 100;
    float angle = 0;
    float distance = 0;
    float speed = 0;
	

	while(ros::ok())
	{	
		while(lostsync)
		{
			//Looking for sync data for the header
			int n = 0;
			while(n < 8)
			{
				while(fd.available() <= 0);
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
				for (int i = 8; i < 52; i++ )
				{
					while(fd.available() <= 0);
					uint8_t temp_byte;
					fd.read(&temp_byte, 1);
					frame_header[i] = temp_byte;
				}
			}
		}

		while(lostsync == 0)
		{
			
			if (frame_header_frame_number > target_frame_number)
			{
				target_frame_number = frame_header_frame_number;

			}
			else
			{
				lostsync = 1; //old frame
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

			cout<<"\n\nframe no.           "<<target_frame_number<<endl<<endl;
//			cout<<"\nTLV    no.           "<<frame_header_no_tlv<<endl<<endl;

			for (int i = 0; i < frame_header_no_tlv; i++)
			{
				for (int i = 0; i < 8; i++)
				{
					while(fd.available() <= 0);
					uint8_t temp_byte;
					fd.read(&temp_byte, 1);
					tlv_header[i] = temp_byte;
//					printf("%02x ", tlv_header[i]);
				}
				

				if (tlv_header_type != tlv_header_type_pointCloud && tlv_header_type != tlv_header_type_targetObjectList && tlv_header_type != tlv_header_type_targetIndex)
					{
						printf("Header is wrong...............................!!!!!!!!!!!!!!!!!!!!!");
//						for (int i = 0; i < 7; i++)
//						{
//							tlv_header[i] = tlv_header[i+1];
//						}
//						uint8_t temp_byte;
//						fd.read(&temp_byte, 1);
//						tlv_header[7] = temp_byte;

						for (int i = 0; i < 8; i++)
						{
							printf("%02x ", tlv_header[i]);
						}
						printf("\n");
					}


				if (tlv_header_type == tlv_header_type_pointCloud)
				{
//					printf("TLV header point cloud found\n\n");
					tlv_dataLength = tlv_header_length - 8;
//					cout<<endl<<tlv_dataLength<<endl;
					int no_of_pc = tlv_dataLength/16;
//					printf("%d\n", no_of_pc);

					for (int i = 0; i < tlv_dataLength; i++)
					{
						while(fd.available() <= 0);
						uint8_t temp_byte;
						fd.read(&temp_byte, 1);
					}
				}

				else if (tlv_header_type == tlv_header_type_targetObjectList)
				{
//					printf("TLV header object list found\n\n");
					tlv_dataLength = tlv_header_length - 8;

					int no_of_objects = tlv_dataLength/68;

					if(no_of_objects > 10)
						continue;

//					printf("%d\n", no_of_objects);

					

					for (int i = 0; i < no_of_objects; i++)
					{
						for (int j = 0; j < 68; j++)
						{
							while(fd.available() <= 0);
							uint8_t temp_byte;
							fd.read(&temp_byte, 1);
							tlv_data_targetObjectList[j] = temp_byte;
						}
						

						object_list_index[i] = tlv_data_targetObjectList_trackID;
						object_list[i][0] = tlv_data_targetObjectList_posX;
						object_list[i][1] = tlv_data_targetObjectList_posY;
						object_list[i][2] = tlv_data_targetObjectList_velX;
						object_list[i][3] = tlv_data_targetObjectList_velY;
						object_list[i][4] = tlv_data_targetObjectList_accX;
						object_list[i][5] = tlv_data_targetObjectList_accY;
						
//						cout<<"target no.          "<<tlv_data_targetObjectList_trackID<<endl;
//						printf("target no.   %02x \n", tlv_data_targetObjectList_trackID);
//						cout<<"position     X      "<<tlv_data_targetObjectList_posX<<endl;
//						cout<<"position     Y      "<<tlv_data_targetObjectList_posY<<endl;
//						cout<<"velocity     X      "<<tlv_data_targetObjectList_velX<<endl;
//						cout<<"velocity     Y      "<<tlv_data_targetObjectList_velY<<endl;
//						cout<<"acceleration X      "<<tlv_data_targetObjectList_accX<<endl;
//						cout<<"acceleration Y      "<<tlv_data_targetObjectList_accY<<endl<<endl;

						
					}

					int found_last_track_ID = 0;
					int last_track_ID_index;

					for (int i = 0; i < no_of_objects; i++)
					{
						if (last_track_ID == object_list_index[i])
						{
							found_last_track_ID = 1;
							last_track_ID_index = i;
							break;
						}
					}

					if (found_last_track_ID == 0)
					{
						for (int i = 0; i < no_of_objects; i++)
						{
							object_list_range[i] = object_list[i][0] * object_list[i][0] + object_list[i][1]*object_list[i][1];
						}

						//sorting of object_list_range

						int object_list_index_counter[no_of_objects];
						for (int i = 0; i < no_of_objects; i++)
						{
							object_list_index_counter[i] = i;
						}

						for(int i = 0; i<no_of_objects-1;i++)
						{
							for(int j = 0; j<no_of_objects-i-1; j++)
							{
								if(object_list_range[j] > object_list_range[j+1])
								{
									double temp_range = object_list_range[j];
									object_list_range[j] = object_list_range[j+1];
									object_list_range[j+1] = temp_range;
									int temp_key = object_list_index_counter[j];
									object_list_index_counter[j] = object_list_index_counter[j+1];
									object_list_index_counter[j+1] = temp_key;
								}
							}
						}
						//set track id to be the nearest target
						last_track_ID = object_list_index[object_list_index_counter[0]];
						last_track_ID_index = object_list_index_counter[0];
					}


                    angle = atan(object_list[last_track_ID_index][0]/object_list[last_track_ID_index][1]);
                    distance = sqrt(object_list[last_track_ID_index][0]*object_list[last_track_ID_index][0]+object_list[last_track_ID_index][1]*object_list[last_track_ID_index][1]);
                    speed = object_list[last_track_ID_index][2]*sin(angle) + object_list[last_track_ID_index][3]*cos(angle);

                    printf("x = %lf ", object_list[last_track_ID_index][0]);
                    printf("   y = %lf \n", object_list[last_track_ID_index][1]);
					posMsg.header.stamp = ros::Time::now();
					posMsg.header.frame_id = '1';//tlv_data_targetObjectList_trackID;
					posMsg.point.x = object_list[last_track_ID_index][0];
					posMsg.point.y = object_list[last_track_ID_index][1];
					posMsg.point.z = object_list_index[last_track_ID_index];
					posPub.publish(posMsg);


                    //kalman filter
					//radar_estimator(object_list[last_track_ID_index][0],object_list[last_track_ID_index][1] , object_list[last_track_ID_index][2], object_list[last_track_ID_index][3]);
                    //radar_estimator(distance, angle*180/3.1415926 , speed, 0);



                    for (int i = 0; i < 8;; i++)
                    {
                        posMsg.header.stamp = ros::Time::now();
                        posMsg.header.frame_id = '1';//tlv_data_targetObjectList_trackID;
                        posMsg.point.x = object_list[i][0];
                        posMsg.point.y = object_list[i][1];
                        posMsg.point.z = object_list_index[i];

                        switch(i){
                            case 0: pos0Pub.publish(posMsg);

                            case 1: pos1Pub.publish(posMsg);

                            case 2: pos2Pub.publish(posMsg);

                            case 3: pos3Pub.publish(posMsg);

                            case 4: pos4Pub.publish(posMsg);

                            case 5: pos5Pub.publish(posMsg);

                            case 6: pos6Pub.publish(posMsg);

                            case 7: pos7Pub.publish(posMsg);
                        }
                        
                    }


                    posMsg.header.stamp = ros::Time::now();
                    posMsg.header.frame_id = '1';//tlv_data_targetObjectList_trackID;
                    posMsg.point.x = 0;
                    posMsg.point.y = 0;
                    posMsg.point.z = 0;

                    switch(last_track_ID_index){
                        case 0: pos0Pub.publish(posMsg);

                        case 1: pos1Pub.publish(posMsg);

                        case 2: pos2Pub.publish(posMsg);

                        case 3: pos3Pub.publish(posMsg);

                        case 4: pos4Pub.publish(posMsg);

                        case 5: pos5Pub.publish(posMsg);

                        case 6: pos6Pub.publish(posMsg);

                        case 7: pos7Pub.publish(posMsg);
                    }

                    if (no_of_objects < 8)
                    {
                        for (int i = no_of_objects; i < 8;; i++)
                        {
                            switch(i){
                                case 0: pos0Pub.publish(posMsg);

                                case 1: pos1Pub.publish(posMsg);

                                case 2: pos2Pub.publish(posMsg);

                                case 3: pos3Pub.publish(posMsg);

                                case 4: pos4Pub.publish(posMsg);

                                case 5: pos5Pub.publish(posMsg);

                                case 6: pos6Pub.publish(posMsg);

                                case 7: pos7Pub.publish(posMsg);
                            }
                            
                        }
                    }
				
				}

				else if (tlv_header_type == tlv_header_type_targetIndex)
				{
//					printf("TLV header target index found\n\n");
					tlv_dataLength = tlv_header_length - 8;

					for (int i = 0; i < tlv_dataLength; i++)
					{
						while(fd.available() <= 0);
						uint8_t temp_byte;
						fd.read(&temp_byte, 1);
					}
				}
				else
				{
					cout<<"TLV header wrong, lost sync at frame = "<<target_frame_number<<endl<<endl;
					lostsync = 1;				
				}
			}

			lostsync = 1;
		}

		

		rate.sleep();
	}
}
