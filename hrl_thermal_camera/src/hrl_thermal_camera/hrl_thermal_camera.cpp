#define __STDC_CONSTANT_MACROS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <cxxabi.h>
#include <iostream>
//#include <opencv2/imgproc.hpp>
//#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

using namespace cv;
using namespace std;


#include "hrl_thermal_camera.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/fill_image.h>

#define CLEAR(x) memset (&(x), 0, sizeof (x))

PV_INIT_SIGNAL_HANDLER();

namespace hrl_thermal_cam {

HRLThermalCamera::HRLThermalCamera()
  : image_(NULL), is_capturing_(false)
{
}
HRLThermalCamera::~HRLThermalCamera()
{
  shutdown();
}

void HRLThermalCamera::start(int image_width, int image_height, int framerate)
{
  lPvSystem = new PvSystem;
	SelectDevice();
  Connect();
  start_capturing();
}

void HRLThermalCamera::shutdown(void)
{
  stop_capturing();
  Disconnect();
  delete lPvSystem;
}

void HRLThermalCamera::SelectDevice( )
{
  PvResult lResult;
  if (NULL != lPvSystem) 
  {
    lDeviceInfo = PvSelectDevice(*lPvSystem );
  }
}

bool HRLThermalCamera::is_capturing() {
  return is_capturing_;
}

void HRLThermalCamera::stop_capturing(void)
{
  if(!is_capturing_) return;

  if(!lDeviceParams)
    return;
  lStop->Execute();
  lDevice->StreamDisable();
  lPipeline->Stop();
  is_capturing_ = true;
}

void HRLThermalCamera::start_capturing(void)
{
  if(is_capturing_) return;

  lPipeline->Start();
  lDevice->StreamEnable();
  lStart->Execute();
  CacheParams();
  is_capturing_ = true;
}

void HRLThermalCamera::Connect()
{
  ConnectToDevice();
  OpenStream();
  ConfigureStream();
  CreatePipeline();
  lStart = dynamic_cast<PvGenCommand *>( lDeviceParams->Get( "AcquisitionStart" ) );
  lStop = dynamic_cast<PvGenCommand *>( lDeviceParams->Get( "AcquisitionStop" ) );
}

void HRLThermalCamera::Disconnect()
{
  delete lPipeline;
  lStream->Close();
  PvStream::Free(lStream );
  lDevice->Disconnect();
  PvDevice::Free(lDevice);
  lDeviceParams = NULL;
  lStreamParams = NULL;
}

void HRLThermalCamera::ConnectToDevice( )
{
  PvResult lResult;
  cout << "Connecting to " << lDeviceInfo->GetDisplayID().GetAscii() << "." << endl;
  lDevice = PvDevice::CreateAndConnect(lDeviceInfo, &lResult );
  if ( lDevice == NULL )
  {
    cout << "Unable to connect to " << lDeviceInfo->GetDisplayID().GetAscii() << "." << endl;
  }
  lDeviceParams = lDevice->GetParameters();
}

void HRLThermalCamera::OpenStream( )
{
  PvResult lResult;
  cout << "Opening stream to device." << endl;
  lStream = PvStream::CreateAndOpen(lDeviceInfo->GetConnectionID(),&lResult);
  if ( lStream == NULL )
  {
    cout << "Unable to stream from " << lDeviceInfo->GetDisplayID().GetAscii() << "." << endl;
  }
}

void HRLThermalCamera::ConfigureStream( )
{
  PvDeviceGEV* lDeviceGEV = dynamic_cast<PvDeviceGEV *>( lDevice );
  if ( lDeviceGEV != NULL )
  {
    PvStreamGEV *lStreamGEV = static_cast<PvStreamGEV *>( lStream );
    // Negotiate packet size
    lDeviceGEV->NegotiatePacketSize();
    // Configure device streaming destination
    lDeviceGEV->SetStreamDestination( lStreamGEV->GetLocalIPAddress(), lStreamGEV->GetLocalPort() );
  }
}

void HRLThermalCamera::CreatePipeline()
{
	// Create the PvPipeline object
  lPipeline = new PvPipeline( lStream );

  if ( lPipeline != NULL )
    {
      // Reading payload size from device
      uint32_t lSize = lDevice->GetPayloadSize();
      // Set the Buffer count and the Buffer size
      lPipeline->SetBufferCount( BUFFER_COUNT );
      lPipeline->SetBufferSize( lSize );
    }
}

void HRLThermalCamera::CacheParams()
{
  int64_t width, height;
  lStreamParams = lStream->GetParameters();
  PvGenFloat *lFrameRate = dynamic_cast<PvGenFloat *>( lStreamParams->Get( "AcquisitionRate" ) );
  PvGenFloat *lBandwidth = dynamic_cast<PvGenFloat *>( lStreamParams->Get( "Bandwidth" ) );
  double lFrameRateVal = 0.0;
  double lBandwidthVal = 0.0;
  lFrameRate->GetValue( lFrameRateVal );
  lBandwidth->GetValue( lBandwidthVal );
  lStreamParams->GetIntegerValue("Width",width);
  lStreamParams->GetIntegerValue("Height",height);
  camInfo.height = height;
  camInfo.width = width;
  camInfo.frameRate = lFrameRateVal;
  camInfo.bandwidth = lBandwidthVal;
}

bool HRLThermalCamera::grab_image(sensor_msgs::Image& image_msg, sensor_msgs::CameraInfo &cinfo_msg)
{
  static bool skip_next_frame = false;

  // Start loop for acquisition
  PvBuffer *buffer;
  PvResult op_result;

  // Skip next frame when operation is not ok
  if (skip_next_frame)
    {
      skip_next_frame = false;
      sleep(1);
    }

  // Retrieve next buffer
  PvResult result = lPipeline->RetrieveNextBuffer(&buffer, 1000, &op_result);

  // Failed to retrieve buffer
  if (result.IsFailure())
  {
     return false;
  }

  // Operation not ok, need to return buffer back to pipeline
  if (op_result.IsFailure())
  {
      skip_next_frame = true;
      // Release the buffer back to the pipeline
      lPipeline->ReleaseBuffer(buffer);
      return false;
  }

  // Buffer is not an image
  if ((buffer->GetPayloadType()) != PvPayloadTypeImage)
    {
      lPipeline->ReleaseBuffer(buffer);
      return false;
    }

  // Get image specific buffer interface
  PvImage *lImage = buffer->GetImage();





  unsigned int width_prev = lImage->GetWidth(); //this is for duplicate cols, 648x256
  unsigned int height = lImage->GetHeight(); 
  unsigned int width = lImage->GetWidth()/2; //this is for number of cols we want, 324x256

  // Assemble image msg
  image_msg.header.stamp = ros::Time::now();
  cinfo_msg.header = image_msg.header;
  image_msg.height = height;
  image_msg.width = width;
  image_msg.encoding = sensor_msgs::image_encodings::MONO16;
  image_msg.step = lImage->GetBitsPerPixel()/8*width + lImage->GetPaddingX();
  unsigned char* dataBuffer = (unsigned char*)lImage;
  const size_t data_size = lImage->GetImageSize()/2;
  

  if (image_msg.data.size() != data_size)
  {
    image_msg.data.resize(data_size);
  }


  unsigned char *img = lImage->GetDataPointer();


  // Copy/convert Pleora Vision image pointer to cv::Mat container


  //cut out the duplicate columns
  cv::Mat lframe(height, width_prev, CV_16U, img, cv::Mat::AUTO_STEP); 
  cv::Mat lframe_mod = cv::Mat(0, width_prev, CV_16U);
  cv::Mat lframe_t = lframe.t();  
  for (int i=1; i<lframe.cols; i=i+2) {
    lframe_mod.push_back(lframe_t.row(i));
  }
  lframe_mod = lframe_mod.t();

  unsigned char* imgMod = lframe_mod.data;

  memcpy(&image_msg.data[0], imgMod, data_size);

  lPipeline->ReleaseBuffer(buffer);
  return true;
  // stamp the image
  // fill the info
//  fillImage(*msg, "mono16", height, image_->width, image_->width, image_->image);
}

}
