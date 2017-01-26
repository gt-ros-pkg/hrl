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
#include <sensor_msgs/Image.h>

#define BUFFER_COUNT ( 16 )

namespace hrl_thermal_cam {

class HRLThermalCamera
{
  public:
  HRLThermalCamera();
	~HRLThermalCamera();

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





    typedef enum
    {
    IO_METHOD_READ, IO_METHOD_MMAP, IO_METHOD_USERPTR, IO_METHOD_UNKNOWN,
    } io_method;

    typedef enum
    {
    PIXEL_FORMAT_YUYV, PIXEL_FORMAT_UYVY, PIXEL_FORMAT_MJPEG, PIXEL_FORMAT_YUVMONO10, PIXEL_FORMAT_RGB24, PIXEL_FORMAT_GREY, PIXEL_FORMAT_UNKNOWN
    } pixel_format;

    // start camera
    void start(int image_width, int image_height, int framerate);
    // shutdown camera
    void shutdown(void);

    // grabs a new image from the camera
    void grab_image(sensor_msgs::Image& image);

    void stop_capturing(void);
    void start_capturing(void);
    bool is_capturing();

	void ConnectToDevice();
	void OpenStream();
	void ConfigureStream();
	void CreatePipeline();
	void CacheParams();
	void SelectDevice();
	bool grab_image(sensor_msgs::Image &,sensor_msgs::CameraInfo &);
	void Connect();
	void Disconnect();

private:
    typedef struct
    {
    int width;
    int height;
    int bytes_per_pixel;
    int image_size;
    char *image;
    int is_new;
    } camera_image_t;

    struct buffer
    {
    void * start;
    size_t length;
    };

    void init_device(int image_width, int image_height, int framerate);
    void close_device(void);
    void open_device(void);
//    void grab_image();
    bool is_capturing_;

    unsigned int pixelformat_;
    bool monochrome_;
    int fd_;
    unsigned int n_buffers_;

    camera_image_t *image_;

};
}
#endif
