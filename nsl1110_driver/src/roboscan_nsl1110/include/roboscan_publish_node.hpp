#include <cstdio>
#include "cartesian_transform.hpp"
#include "interface.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <pcl/conversions.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/CameraInfo.h>
//#include <sensor_msgs/srv/set_camera_info.hpp>
#include <roboscan_nsl1110/roboscan_nsl1110Config.h>
#include <cstdio>
#include <sys/stat.h>
#include <cstdlib>
#include <unistd.h>
#include <atomic>

namespace nanosys {

	typedef std::vector<uint8_t> Packet;

	struct SetParameter {
        int imageType;
        int lensType;
        int old_lensType;

		std::string	ipAddr;
        
		bool startStream;
		bool publishPointCloud;
		bool cartesian;
		uint16_t channel;
		int frequencyModulation;
		int int0, int1, int2, intGr; //integration times
		int hdr_mode; //0 - hdr off, 1 - hdr spatial, 2 - hdr temporal
		int minAmplitude;
		uint16_t lensCenterOffsetX = 0;
		uint16_t lensCenterOffsetY = 0;
		uint16_t old_lensCenterOffsetX = 0;
		uint16_t old_lensCenterOffsetY = 0;
		uint16_t modIndex = 0;

		int roi_leftX = 4;
		int roi_topY = 6;
		int roi_rightX = 323;
		int roi_bottomY = 125;

		uint8_t grayscaleIlluminationMode = 0;
		uint8_t bAdcOverflow = 1;
		uint8_t bSaturation = 1;
		double transformAngle;
		int cutPixels;

		bool medianFilter;
        bool averageFilter;
       	double temporalFilterFactor ;
        uint16_t temporalFilterThreshold;        
        uint16_t edgeFilterThreshold;
        uint16_t temporalEdgeThresholdLow;
		uint16_t temporalEdgeThresholdHigh;
		uint16_t interferenceDetectionLimit;
		bool useLastValue;

		uint32_t frameSeq;
		bool cvShow;
    };

   	typedef struct _RGB888Pixel
	{
		unsigned char r;
		unsigned char g;
		unsigned char b;
	} RGB888Pixel;

	int x_start;
	int y_start;
	
	class roboscanPublisher { 

		const int LensWidth   = 320;
		const int LensHeight  = 240;
		const double sensorPixelSizeMM = 0.02; //camera sensor pixel size 20x20 um

	public:
		roboscanPublisher();
		~roboscanPublisher();

		void thread_callback();
		void setReconfigure();
		void publishFrame(Frame *frame);
		void setGrayscaleColor(cv::Mat &imageLidar, int x, int y, int value, double end_range );
		void setAmplitudeColor(cv::Mat &imageLidar, int x, int y, int value, double end_range );
		bool startStreaming(Packet &databuf);
		double interpolate( double x, double x0, double y0, double x1, double y1);
		void createColorMapPixel(int numSteps, int indx, unsigned char &red, unsigned char &green, unsigned char &blue);
		void updateConfig(roboscan_nsl1110::roboscan_nsl1110Config &config, uint32_t level);
		boost::signals2::connection connectionCameraInfo;
		

		Interface interface;
		CartesianTransform cartesianTransform;
		sensor_msgs::CameraInfo cameraInfo;


		//static ros::Time timeNow;

		ros::Publisher imgDistancePub;
		ros::Publisher imgAmplPub;
		ros::Publisher imgGrayPub;
		ros::Publisher imgDCSPub;	
		ros::Publisher pointcloudPub;

		int Convert_To_RGB24( float fValue, RGB888Pixel *nRGBData, float fMinValue, float fMaxValue);

	    SetParameter lidarParam;
		std::vector<cv::Vec3b> colorVector;
		float maxDistance;
		boost::scoped_ptr<boost::thread> publisherThread;
		bool runThread;
		bool bSetReconfigure;
		bool bInitCmd;

	private:
		void initialise();
		void setParameters();
		void getMouseEvent( int &mouse_xpos, int &mouse_ypos );
		cv::Mat addDistanceInfo(cv::Mat distMat, std::vector<uint16_t> dist2BData, int width);
		int mouseXpos, mouseYpos;

		//OnSetParametersCallbackHandle::SharedPtr callback_handle_;		
	};


} //end namespace nanosys
