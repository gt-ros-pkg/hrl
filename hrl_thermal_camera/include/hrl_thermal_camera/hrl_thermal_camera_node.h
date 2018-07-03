#ifndef HRL_THERMAL_CAMERA_H
#define HRL_THERMAL_CAMERA_H

//Pleora SDK Headers
#include <PvSampleUtils.h>
#include <PvDevice.h>
#include <PvDeviceGEV.h>
#include <PvDeviceU3V.h>
#include <PvStream.h>
#include <PvStreamGEV.h>
#include <PvStreamU3V.h>
#include <PvPipeline.h>
#include <PvBuffer.h>
#include <PvSystem.h>

//OpenCV Headers
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

//ROS Headers
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#define BUFFER_COUNT ( 16 )


class HRLThermalCamera
{
	const PvDeviceInfo *lDeviceInfo = NULL;
	PvDevice *lDevice = NULL;
	PvStream *lStream;
	PvSystem *lPvSystem;
	PvPipeline *lPipeline = NULL;
	PvGenParameterArray *lDeviceParams = NULL;
	PvGenParameterArray *lStreamParams = NULL;
	PvGenCommand *lStart;
	PvGenCommand *lStop;
	struct
	{
	  int height,width;
	  double frameRate,bandwidth;
	}camInfo;

public:

	HRLThermalCamera();
	~HRLThermalCamera();

	void stop_capturing(void);
  void start_capturing(void);
  bool is_capturing();

	void ConnectToDevice();
	void OpenStream();
	void ConfigureStream();
	void CreatePipeline();
	void CacheParams();
	void SelectDevice();
	bool GrabImage(sensor_msgs::Image &,sensor_msgs::CameraInfo &);
	void Connect();
	void Disconnect();
	void StartAcquisition();
	void StopAcquisition();

};

#endif
