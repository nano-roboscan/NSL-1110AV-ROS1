#include <stdio.h>   
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/types.hpp>

#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <time.h>
#include <vector>


#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

using namespace std;
using namespace cv;

// Lidar Parameter
#define MAX_DISTANCE_MM_12Mhz 		12500
#define MAX_DISTANCE_MM_24Mhz 		6250
#define MAX_PASE  					30000
#define IMAGE_WIDTH 				320
#define IMAGE_HEIGHT				240
#define IMAGE_SIZE					76800 	// 320x240
#define DISTANCE_OFFSET				0 		// 거리정보의 OFFSET 만큼 보정한다.
#define AMPLITUDE_THRESHOLD			20  	// 경계값이하의 약한 신호는 사용하지 않음.
#define AMPLITUDE_OVERFLOW			60000  	// 6XXXX이상은 overflow 인 픽셀을 나타냄

#define DISTANCE_MIN_THRESHOLD		300  	// 가까운 거리는 사용하지 않음.
#define DISTANCE_MAX_THRESHOLD		12000  	// 먼거리 데이터를 제한함.
#define BUFF_SIZE   				153600
static uint8_t *distanceBuff;
static uint8_t *amplutudeBuff;
static uint8_t *response;
static int *dist;
/////////////////////////////////////////////////
clock_t start = clock();


// Lidar Function
void lidar_init(void)
{
	response = (uint8_t *)malloc(sizeof(char)*BUFF_SIZE*2);
	distanceBuff = (uint8_t *)malloc(sizeof(char)*BUFF_SIZE);
	amplutudeBuff = (uint8_t *)malloc(sizeof(char)*BUFF_SIZE);	
	dist = (int *)malloc(sizeof(char)*BUFF_SIZE);	
}

void lidar_close(void)
{
	free(response);
	free(distanceBuff);
	free(amplutudeBuff);
	free(dist);
}

inline int lidar_calcDistance(int phase)
{
	int data = MAX_DISTANCE_MM_12Mhz * phase/MAX_PASE;

	if(phase < 0)
		return -1;

	if(data > DISTANCE_OFFSET) data -= DISTANCE_OFFSET;
	else data = 0;

	return data;
}

int* lidar_getDistanceAndAmplitudeSorted()
{
	int client_socket;
	struct sockaddr_in   server_addr;
	uint16_t *pMemDistance = 0;
	uint16_t *pMemAmplitude = 0;
	int sum = 0;
	int nbyte = 256;
	static unsigned char  buff[1024];


	memset(&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(50660);
	server_addr.sin_addr.s_addr = inet_addr("192.168.7.2");

	client_socket = socket(PF_INET, SOCK_STREAM, 0);
	if(-1 == client_socket)
	{
		printf("Can not open socket-192.168.7.2\n");		
		exit( 1);
	}
   
	if(-1 == connect(client_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)))
	{
		printf("Socket not connect\n");
		exit(1);
   	}

	send(client_socket, "getDistanceAndAmplitudeSorted", strlen("getDistanceAndAmplitudeSorted")+1, 0); // +1: NULL까지 포함해서 전송
   
	while(nbyte!=0)
	{
		nbyte = recv( client_socket, buff, 1024, 0);
		memcpy(response+sum , buff , sizeof(buff));
		sum += nbyte;
	}

	memcpy(distanceBuff, response, BUFF_SIZE);
	memcpy(amplutudeBuff, response+BUFF_SIZE, BUFF_SIZE);

	close(client_socket);

	pMemDistance = (uint16_t *)distanceBuff;
	pMemAmplitude = (uint16_t *)amplutudeBuff;


	for(int y = 0; y < IMAGE_HEIGHT; y++)
	{
		for(int x = 0; x < IMAGE_WIDTH; x++)
		{
			// bin error pixel
			if(pMemAmplitude[y*IMAGE_WIDTH + x] < AMPLITUDE_THRESHOLD)
				dist[y*IMAGE_WIDTH + x] = 0;
			else if(pMemAmplitude[y*IMAGE_WIDTH + x] > AMPLITUDE_OVERFLOW)
				dist[y*IMAGE_WIDTH + x] = 0;

			// calculate distance
			if(pMemDistance[y*IMAGE_WIDTH + x] != 0)				
				dist[y*IMAGE_WIDTH + x] = (MAX_DISTANCE_MM_12Mhz * pMemDistance[y*IMAGE_WIDTH + x])/MAX_PASE;
				// dist[y*IMAGE_WIDTH + x] = lidar_calcDistance(pMemDistance[y*IMAGE_WIDTH + x]);


			if(dist[y*IMAGE_WIDTH + x] < DISTANCE_MIN_THRESHOLD)
				dist[y*IMAGE_WIDTH + x] = 0;
			else if(dist[y*IMAGE_WIDTH + x] > DISTANCE_MAX_THRESHOLD)
				dist[y*IMAGE_WIDTH + x] = 0;
				

			if(x == 160 && y == 120)
				printf("(160, 120) Distance : %.2f mm--***--\n", dist[320*y + x]);
		}
	}


	return dist;
}

typedef struct _RGB888Pixel
{
    unsigned char r;
    unsigned char g;
    unsigned char b;
} RGB888Pixel;

int Convert_To_RGB24_2( float fValue, RGB888Pixel *nRGBData, float fMinValue, float fMaxValue)
{
    if(fValue == 0) //Invalide Pixel
    {
        nRGBData->r = 0;//R
        nRGBData->g = 0;//G
        nRGBData->b = 0;//B
    }
    else if(fValue < fMinValue)
    {
        nRGBData->r = 255;//R
        nRGBData->g = 0;//G
        nRGBData->b = 0;//B
    }
    else if(fValue > fMaxValue)
    {
        nRGBData->r = 255;//R
        nRGBData->g = 0;//G
        nRGBData->b = 255;//B
    }
    else
    {
        float fColorWeight;
        fColorWeight = (fValue-fMinValue) / (fMaxValue-fMinValue);

        if( (fColorWeight <= 1.0f) && (fColorWeight > 0.8f) )
        {
            nRGBData->r = (unsigned char)(255 * ((fColorWeight - 0.8f) / 0.2f));//값에 따라 증가
            nRGBData->g = 0;
            nRGBData->b = 255;
        }
        else if( (fColorWeight <= 0.8f) && (fColorWeight > 0.6f) )
        {
            nRGBData->r = 0;
            nRGBData->g = (unsigned char)(255 * (1.0f - (fColorWeight - 0.6f) / 0.2f));//값에 따라 감소
            nRGBData->b = 255;
        }
        else if( (fColorWeight <= 0.6f) && (fColorWeight > 0.4f) )
        {
            nRGBData->r = 0;
            nRGBData->g = 255;
            nRGBData->b = (unsigned char)(255 * ((fColorWeight - 0.4f) / 0.2f));//값에 따라 증가
        }
        else if( (fColorWeight <= 0.4f) && (fColorWeight > 0.2f) )
        {
            nRGBData->r = (unsigned char)(255 * (1.0f - (fColorWeight - 0.2f) / 0.2f));//값에 따라 감소
            nRGBData->g = 255;
            nRGBData->b = 0;
        }
        else if( (fColorWeight <= 0.2f) && (fColorWeight >= 0.0f) )
        {
            nRGBData->r = 255;
            nRGBData->g = (unsigned char)(255 * ((fColorWeight - 0.0f) / 0.2f));//값에 따라 증가
            nRGBData->b = 0;
        }
        else
        {
            nRGBData->r = 0;
            nRGBData->g = 0;
            nRGBData->b = 0;
        }
    }

    return true;
}


int main(int argc, char** argv)
{
	// Initialize
	lidar_init();
	ros::init(argc, argv, "roboscan_publish_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("roboscan", 1);
	cv::Mat imageLidar(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, Scalar(255,255,255));	
	RGB888Pixel* pTex = new RGB888Pixel[1];

	// Convert the lidar data to pcl/PointCloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);	
	cloud->width  = 320;
	cloud->height = 240;
	cloud->points.resize (cloud->width * cloud->height);


    
    while (nh.ok())
    {
		int *arr = lidar_getDistanceAndAmplitudeSorted();
		int i = 0;
		for(int y = 0; y < IMAGE_HEIGHT; y++)
		{
			for(int x = 0; x < IMAGE_WIDTH; x++)
			{				
				// i = 320*y + (320-x);
				i = 320*y + x;
				Convert_To_RGB24_2((double)arr[i], pTex, 0.0f, 15000.0f);
				imageLidar.at<Vec3b>(y, x)[0] = pTex->b; // 해당 픽셀을 파란색으로 변경
				imageLidar.at<Vec3b>(y, x)[1] = pTex->g; // BGR 순서 
				imageLidar.at<Vec3b>(y, x)[2] = pTex->r; 

				if(x == 160 && y == 120)
					printf("(160, 120) ---Distance : %.2f cm\n", arr[320*y + (320-x)]*0.1);

				// Generate the data
				if(arr[i] == 0)
				{
					cloud->points[i].x = 0;
					cloud->points[i].z = 0;
					cloud->points[i].y = 0;
					cloud->points[i].b = 0;
					cloud->points[i].g = 0;
					cloud->points[i].r = 0;
				}
				else
				{
					cloud->points[i].x = (x-160)/50.0;
					cloud->points[i].z = (240-y)/50.0;
					cloud->points[i].y = arr[i]/500.0;
					cloud->points[i].b = pTex->b;
					cloud->points[i].g = pTex->g;
					cloud->points[i].r = pTex->r;
				}
			}
		}
		// Show OpenCV Window
		namedWindow("roboscan", WINDOW_NORMAL);
		resizeWindow("roboscan", 320*2, 240*2);
		imshow("roboscan", imageLidar);

		// Publish the pclcloud2 msg for rviz pointcloud2
		sensor_msgs::PointCloud2 outputMsg;
		pcl::toROSMsg(*cloud, outputMsg);
		outputMsg.header.stamp = ros::Time::now();
		outputMsg.header.frame_id = "/map";
		pub.publish (outputMsg);		

		//wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
		if(cv::waitKey(1) == 27) 
		{
			std::cout << "esc key is pressed by user" << std::endl;
			break; 
		}

        // clock_t end = clock();
        // ROS_INFO("%0.4f sec..", (double)(end-start) / 100000); 
        // start = end;
        
        ros::spinOnce();
    }

	delete pTex;

	lidar_close();
	return 0;    
}
