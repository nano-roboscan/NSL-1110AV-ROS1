#include <ros/ros.h>
#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/types.hpp>

#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Float32.h>
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
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>


//control + C event
#include <cstdio>
#include <sys/stat.h>
#include <cstdlib>
#include <unistd.h>
#include "std_msgs/String.h"


using namespace std;
using namespace cv;

// Lidar Parameter
#define MAX_DISTANCE_MM_12Mhz 		12500
#define MAX_DISTANCE_MM_24Mhz	 	6250
#define MAX_PASE  					30000
#define IMAGE_WIDTH 				320
#define IMAGE_HEIGHT				240
#define IMAGE_SIZE					76800 	// 320x240
#define DISTANCE_OFFSET				0 		// 거리정보의 OFFSET 만큼 보정한다.
#define AMPLITUDE_THRESHOLD			20  	// 경계값이하의 약한 신호는 사용하지 않음. 20 100
#define DISTANCE_THRESHOLD			200  	// 경계값이하의 약한 신호는 사용하지 않음.
#define BUFF_SIZE   				153600

#define LOW_AMPLITUDE				65300
#define ADC_OVERFLOW				65500
#define SATURATION					65400


static uint8_t *distanceBuff;
static uint8_t *amplutudeBuff;
static uint8_t *response;
static int *dist;
//static int *ampl;

clock_t start = clock();

uint16_t *pMemDistance = 0;
uint16_t *pMemAmplitude = 0;

void (*breakCapture)(int);

static int amplitudeData[320*240] = {0, };

int distortionTableSize;
int numCols;
int numRows;
int angleH;
double angle[101];
double rp[101];

double fixAngle;
double angleR;
double sin_angle;
double cos_angle;   

double xUA[320][240];
double yUA[320][240];
double zUA[320][240];

void initLensDistortionTable();
double interpolate(double x_in, double x0, double y0, double x1, double y1);
double getAngle(double x, double y, double sensorPointSizeMM);

//////////////////////////////////////////////////
void signalingHandler(int signo) {
  printf("'Ctrl + C' processing...");

  exit(1);
}

void initLensDistortionTable()
{
    // 110 agree
    distortionTableSize = 46;
    angleH = 110;

    angle[	0	] = 	0	;
    angle[	1	] = 	1.677055033	;
    angle[	2	] = 	3.354110067	;
    angle[	3	] = 	5.0311651	;
    angle[	4	] = 	6.708220134	;
    angle[	5	] = 	8.385275167	;
    angle[	6	] = 	10.0623302	;
    angle[	7	] = 	11.73938523	;
    angle[	8	] = 	13.41644027	;
    angle[	9	] = 	15.0934953	;
    angle[	10	] = 	16.77055033	;
    angle[	11	] = 	18.44760537	;
    angle[	12	] = 	20.1246604	;
    angle[	13	] = 	21.80171543	;
    angle[	14	] = 	23.47877047	;
    angle[	15	] = 	25.1558255	;
    angle[	16	] = 	26.83288053	;
    angle[	17	] = 	28.50993557	;
    angle[	18	] = 	30.1869906	;
    angle[	19	] = 	31.86404563	;
    angle[	20	] = 	33.54110067	;
    angle[	21	] = 	35.2181557	;
    angle[	22	] = 	36.89521074	;
    angle[	23	] = 	38.57226577	;
    angle[	24	] = 	40.2493208	;
    angle[	25	] = 	41.92637584	;
    angle[	26	] = 	43.60343087	;
    angle[	27	] = 	45.2804859	;
    angle[	28	] = 	46.95754094	;
    angle[	29	] = 	48.63459597	;
    angle[	30	] = 	50.311651	;
    angle[	31	] = 	51.98870604	;
    angle[	32	] = 	53.66576107	;
    angle[	33	] = 	55.3428161	;
    angle[	34	] = 	57.01987114	;
    angle[	35	] = 	58.69692617	;
    angle[	36	] = 	60.3739812	;
    angle[	37	] = 	62.05103624	;
    angle[	38	] = 	63.72809127	;
    angle[	39	] = 	65.4051463	;
    angle[	40	] = 	67.08220134	;
    angle[	41	] = 	68.75925637	;
    angle[	42	] = 	70.4363114	;
    angle[	43	] = 	72.11336644	;
    angle[	44	] = 	73.79042147	;
    angle[	45	] = 	75.4674765	;


    //size mm
    rp[	0	] = 	0	;
    rp[	1	] = 	0.1	;
    rp[	2	] = 	0.2	;
    rp[	3	] = 	0.3	;
    rp[	4	] = 	0.4	;
    rp[	5	] = 	0.5	;
    rp[	6	] = 	0.6	;
    rp[	7	] = 	0.7	;
    rp[	8	] = 	0.8	;
    rp[	9	] = 	0.9	;
    rp[	10	] = 	1	;
    rp[	11	] = 	1.1	;
    rp[	12	] = 	1.2	;
    rp[	13	] = 	1.3	;
    rp[	14	] = 	1.4	;
    rp[	15	] = 	1.5	;
    rp[	16	] = 	1.6	;
    rp[	17	] = 	1.7	;
    rp[	18	] = 	1.8	;
    rp[	19	] = 	1.9	;
    rp[	20	] = 	2	;
    rp[	21	] = 	2.1	;
    rp[	22	] = 	2.2	;
    rp[	23	] = 	2.3	;
    rp[	24	] = 	2.4	;
    rp[	25	] = 	2.5	;
    rp[	26	] = 	2.6	;
    rp[	27	] = 	2.7	;
    rp[	28	] = 	2.8	;
    rp[	29	] = 	2.9	;
    rp[	30	] = 	3	;
    rp[	31	] = 	3.1	;
    rp[	32	] = 	3.2	;
    rp[	33	] = 	3.3	;
    rp[	34	] = 	3.4	;
    rp[	35	] = 	3.5	;
    rp[	36	] = 	3.6	;
    rp[	37	] = 	3.7	;
    rp[	38	] = 	3.8	;
    rp[	39	] = 	3.9	;
    rp[	40	] = 	4	;
    rp[	41	] = 	4.1	;
    rp[	42	] = 	4.2	;
    rp[	43	] = 	4.3	;
    rp[	44	] = 	4.4	;
    rp[	45	] = 	4.5	;
}

double interpolate(double x_in, double x0, double y0, double x1, double y1)
{
	return ((x_in-x0)*(y1-y0)/(x1-x0) + y0);
}

double getAngle(double x, double y, double sensorPointSizeMM)
{
    double radius = sensorPointSizeMM * sqrt(x*x + y*y);
    double alfaGrad = 0;

    for(int i=1; i<distortionTableSize; i++)
    {
        if(radius >= rp[i-1] && radius <= rp[i]){

            alfaGrad = interpolate(radius, rp[i-1], angle[i-1], rp[i], angle[i]);
        }
    }

    return alfaGrad;
}

void transformPixel(unsigned int srcX, unsigned int srcY, double srcZ, double *destX, double *destY, double *destZ, double sin_angle, double cos_angle)
{   
	double y = srcZ * yUA[srcX][srcY];
    double z = srcZ * zUA[srcX][srcY];

    *destX   = srcZ * xUA[srcX][srcY];
    *destY = z * sin_angle + y * cos_angle;
    *destZ = z * cos_angle - y * sin_angle;
}

double getLensAngleH()
{
    return angleH;
}

void initLensTransform(double sensorPointSizeMM, int width, int height, int offsetX, int offsetY)
{
    int x, y, row, col;
    numCols = width;
    numRows = height;

    initLensDistortionTable();

    int r0 = 1 - numRows/2 + offsetY;
    int c0 = 1 - numCols/2 + offsetX;

    for(y=0, row = r0; y < numRows; row++, y++){
        for(x=0, col = c0; x < numCols; col++, x++){

            double c = col - 0.5;
            double r = row - 0.5;

            double angleGrad = getAngle(c, r, sensorPointSizeMM);
            double angleRad =  angleGrad * 3.14159265 / 180.0;

            double rp = sqrt(c * c + r * r);
            double rUA = sin(angleRad);

            xUA[x][y] = c * rUA / rp;
            yUA[x][y] = r * rUA / rp;
            zUA[x][y] = cos(angleRad);
        }
    }
}


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

	int sum = 0;
	int nbyte = 256;
	static unsigned char  buff[1024];

#if 1
	memset(&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(50660);
	server_addr.sin_addr.s_addr = inet_addr("192.168.7.2");

	client_socket = socket(PF_INET, SOCK_STREAM, 0);
	if(-1 == client_socket)
	{
		printf("Can not open socket\n");
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
			if(pMemDistance[y*IMAGE_WIDTH + x] < AMPLITUDE_THRESHOLD)
				dist[y*IMAGE_WIDTH + x] = 0;
			else if(pMemDistance[y*IMAGE_WIDTH + x] < DISTANCE_THRESHOLD)
				dist[y*IMAGE_WIDTH + x] = 0;
			else if(pMemDistance[y*IMAGE_WIDTH + x] > MAX_DISTANCE_MM_12Mhz && pMemDistance[y*IMAGE_WIDTH + x] < 65000)
				dist[y*IMAGE_WIDTH + x] = 0;
			else if(pMemDistance[y*IMAGE_WIDTH + x] == LOW_AMPLITUDE)
			 	dist[y*IMAGE_WIDTH + x] = 0;
			else if(pMemDistance[y*IMAGE_WIDTH + x] == ADC_OVERFLOW)
			 	dist[y*IMAGE_WIDTH + x] = ADC_OVERFLOW;
			else if(pMemDistance[y*IMAGE_WIDTH + x] == SATURATION)
			 	dist[y*IMAGE_WIDTH + x] = SATURATION;
			else
			 	dist[y*IMAGE_WIDTH + x] = lidar_calcDistance(pMemDistance[y*IMAGE_WIDTH + x]);
			
			amplitudeData[y*IMAGE_WIDTH + x] = pMemAmplitude[y*IMAGE_WIDTH + x];


			if(x == 160 && y == 120)
			{
				if(dist[320*y + x] > 60000)
					printf("(160, 120) Distance : invalid\n");
				else
					printf("(160, 120) Distance : %d mm\n", dist[320*y + x]);
			}
		}
	}
#endif

	return dist;
}

typedef struct _RGB888Pixel
{
    unsigned char r;
    unsigned char g;
    unsigned char b;
} RGB888Pixel;

int Convert_To_RGB24( float fValue, RGB888Pixel *nRGBData, float fMinValue, float fMaxValue)
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
            nRGBData->r = (unsigned char)(255 * ((fColorWeight - 0.8f) / 0.2f));
            nRGBData->g = 0;
            nRGBData->b = 255;
        }
        else if( (fColorWeight <= 0.8f) && (fColorWeight > 0.6f) )
        {
            nRGBData->r = 0;
            nRGBData->g = (unsigned char)(255 * (1.0f - (fColorWeight - 0.6f) / 0.2f));
            nRGBData->b = 255;
        }
        else if( (fColorWeight <= 0.6f) && (fColorWeight > 0.4f) )
        {
            nRGBData->r = 0;
            nRGBData->g = 255;
            nRGBData->b = (unsigned char)(255 * ((fColorWeight - 0.4f) / 0.2f));
        }
        else if( (fColorWeight <= 0.4f) && (fColorWeight > 0.2f) )
        {
            nRGBData->r = (unsigned char)(255 * (1.0f - (fColorWeight - 0.2f) / 0.2f));
            nRGBData->g = 255;
            nRGBData->b = 0;
        }
        else if( (fColorWeight <= 0.2f) && (fColorWeight >= 0.0f) )
        {
            nRGBData->r = 255;
            nRGBData->g = (unsigned char)(255 * ((fColorWeight - 0.0f) / 0.2f));
            nRGBData->b = 0;
        }
        else
        {
            nRGBData->r = 0;
            nRGBData->g = 0;
            nRGBData->b = 0;
        }
    }

    if(fValue == SATURATION)
    {
    	nRGBData->r = 255;
		nRGBData->g = 0;
		nRGBData->b = 128;
    }
    if(fValue == ADC_OVERFLOW)
    {
    	nRGBData->r = 169;
		nRGBData->g = 14;
		nRGBData->b = 255;
    }

    return true;
}

void getGrayscaleColor(cv::Mat &imageGray, int x, int y, int value, double end_range )
{
	if (value == SATURATION)
	{
		imageGray.at<Vec3b>(y, x)[0] = 128;
		imageGray.at<Vec3b>(y, x)[1] = 0;
		imageGray.at<Vec3b>(y, x)[2] = 255; 
	}
	else if (value == ADC_OVERFLOW)
	{
		imageGray.at<Vec3b>(y, x)[0] = 255;
		imageGray.at<Vec3b>(y, x)[1] = 14;
		imageGray.at<Vec3b>(y, x)[2] = 169; 
	}
	else if (value > end_range)
	{
		imageGray.at<Vec3b>(y, x)[0] = 255;
		imageGray.at<Vec3b>(y, x)[1] = 255;
		imageGray.at<Vec3b>(y, x)[2] = 255; 
	}
	else if (value < 0)
	{
		imageGray.at<Vec3b>(y, x)[0] = 0;
		imageGray.at<Vec3b>(y, x)[1] = 0;
		imageGray.at<Vec3b>(y, x)[2] = 0; 
	}
	else
	{
		int color = value * (255/end_range);

		//printf("color index = %d\n", color);

		imageGray.at<Vec3b>(y, x)[0] = color;
		imageGray.at<Vec3b>(y, x)[1] = color;
		imageGray.at<Vec3b>(y, x)[2] = color; 
	}
}


int main(int argc, char** argv)
{
	// Initialize
	lidar_init();
	ros::init(argc, argv, "roboscan_publish_node");
    
	//image publisher
	ros::NodeHandle nh;
	
	ros::Publisher pointcloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("points", 1);

	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("video_source/raw", 1);

    
	cv::Mat imageLidar(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, Scalar(255,255,255));
	cv::Mat passGray(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, Scalar(255,255,255));

	
	RGB888Pixel* pTex = new RGB888Pixel[1];

	double pX = 0;
	double pY = 0;
	double pDistance[320*240] = {9999.9f, };
	double transAngle = 0;
	initLensTransform(0.02, 320, 240, 0, 0);
    while (nh.ok())
    {
		ros::Time scan_time = ros::Time::now();
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);	
		cloud->width  = 320;
		cloud->height = 240;
		cloud->points.resize (cloud->width * cloud->height);

		int *arr = lidar_getDistanceAndAmplitudeSorted();
		
		int i = 0;
		int j = 0;
		int p = 0;
		for(int y = 0; y < IMAGE_HEIGHT; y++)
		{
			for(int x = 0; x < IMAGE_WIDTH; x++)
			{				
				i = 320*y + x;
				j = 320*y + (319-x);
			
				transformPixel(x, y, (double)arr[320*y + x], &pX, &pY, &pDistance[320 * y + x], 0, 1);

				Convert_To_RGB24((double)arr[320*y + (319-x)], pTex, 0.0f, 12500.0f);
				imageLidar.at<Vec3b>(y, x)[0] = pTex->b; // 해당 픽셀을 파란색으로 변경
				imageLidar.at<Vec3b>(y, x)[1] = pTex->g; // BGR 순서 
				imageLidar.at<Vec3b>(y, x)[2] = pTex->r; 
				
				
				getGrayscaleColor(passGray, x, y, amplitudeData[320*y + (319-x)], 1000.0);
				
				// Generate the data
				if(arr[i] == 0 || arr[i] > 60000)
				{
					cloud->points[i].x = 0;//std::numeric_limits<float>::quiet_NaN();
                    cloud->points[i].y = 0;//std::numeric_limits<float>::quiet_NaN();
                    cloud->points[i].z = 0;//std::numeric_limits<float>::quiet_NaN();
					cloud->points[i].b = 0;
                    cloud->points[i].g = 0;
                    cloud->points[i].r = 0;
				}
				else
				{
					cloud->points[i].x = (pDistance[320 * y + x])/1000.0;
					cloud->points[i].y = (pX/1000.0);
					cloud->points[i].z = (-pY/1000.0);
					
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
		//imshow("amplitude Gray", passGray);


		//sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", passGray).toImageMsg();
		
		// Publish the pclcloud2 msg for rviz pointcloud2
		sensor_msgs::PointCloud2 pointcloudMsg;
		pcl::toROSMsg(*cloud, pointcloudMsg);
		pointcloudMsg.header.stamp = ros::Time::now();
		pointcloudMsg.header.frame_id = "/roboscan_frame";
		pointcloudPublisher.publish(pointcloudMsg);


		//wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
		if(cv::waitKey(1) == 27) 
		{
			std::cout << "esc key is pressed by user" << std::endl; 
			exit(1); 
		}

        clock_t end = clock();
        //ROS_INFO("%0.4f sec..", 1/((double)(end-start) / 100000)); 

        
        ros::spinOnce();
    }

	delete pTex;

	lidar_close();
	return 0;    
}
