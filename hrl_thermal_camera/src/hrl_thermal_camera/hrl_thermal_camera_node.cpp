
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "hrl_thermal_camera_node.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <iostream>
#include <sstream>
#include <std_srvs/Empty.h>

using namespace std;

using namespace cv;

namespace hrl_thermal_cam {

class HRLThermalCameraNode
{
public:
    // private ROS node handle
    ros::NodeHandle node_;

    // shared image message
    sensor_msgs::Image img_;
    image_transport::CameraPublisher image_pub_;

    // parameters
    std::string video_device_name_, io_method_name_, pixel_format_name_, camera_name_, camera_info_url_;
    //std::string start_service_name_, start_service_name_;
    bool streaming_status_;
    int image_width_, image_height_, framerate_, exposure_, brightness_, contrast_, saturation_, sharpness_, focus_,
        white_balance_, gain_;
    bool autofocus_, autoexposure_, auto_white_balance_;
    boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

    HRLThermalCam cam_;

    ros::ServiceServer service_start_, service_stop_;

    bool service_start_cap(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
    {
      cam_.start_capturing();
      return true;
    }

    bool service_stop_cap( std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
    {
      cam_.stop_capturing();
      return true;
    }




    HRLThermalCameraNode() :
        node_("~")
    {
        PV_INIT_SIGNAL_HANDLER();
	    lPvSystem = new PvSystem;
	    SelectDevice();
    }

HRL_Thermal_Camera::~HRL_Thermal_Camera()
{
	delete lPvSystem;
}

void HRL_Thermal_Camera::SelectDevice( )
{
	PvResult lResult;

	if (NULL != lPvSystem)
	{
		lDeviceInfo = PvSelectDevice(*lPvSystem );
	}
}

void HRL_Thermal_Camera::ConnectToDevice( )
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

void HRL_Thermal_Camera::OpenStream( )
{
	PvResult lResult;
	cout << "Opening stream to device." << endl;
	lStream = PvStream::CreateAndOpen(lDeviceInfo->GetConnectionID(),&lResult);
	if ( lStream == NULL )
	{
		cout << "Unable to stream from " << lDeviceInfo->GetDisplayID().GetAscii() << "." << endl;
	}
}

void HRL_Thermal_Camera::ConfigureStream( )
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

void HRL_Thermal_Camera::CreatePipeline()
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

void HRL_Thermal_Camera::Connect()
{
  ConnectToDevice();
  OpenStream();
  ConfigureStream();
  CreatePipeline();
  lStart = dynamic_cast<PvGenCommand *>( lDeviceParams->Get( "AcquisitionStart" ) );
  lStop = dynamic_cast<PvGenCommand *>( lDeviceParams->Get( "AcquisitionStop" ) );
}

void HRL_Thermal_Camera::Disconnect()
{
  delete lPipeline;
  lStream->Close();
  PvStream::Free(lStream );
  lDevice->Disconnect();
  PvDevice::Free(lDevice);
  lDeviceParams = NULL;
  lStreamParams = NULL;
}

void HRL_Thermal_Camera::StartAcquisition()
{
  lPipeline->Start();
  lDevice->StreamEnable();
  lStart->Execute();
  CacheParams();
}

void HRL_Thermal_Camera::StopAcquisition()
{
  if(!lDeviceParams)
    return;
  lStop->Execute();
  lDevice->StreamDisable();
  lPipeline->Stop();
}

void HRL_Thermal_Camera::CacheParams()
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

bool HRL_Thermal_Camera::GrabImage(sensor_msgs::Image &image_msg,sensor_msgs::CameraInfo &cinfo_msg)
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
  unsigned int width = lImage->GetWidth();
  unsigned int height = lImage->GetHeight();

  // Assemble image msg
  image_msg.height = height;
  image_msg.width = width;
  image_msg.encoding = sensor_msgs::image_encodings::MONO16;
  image_msg.step = lImage->GetBitsPerPixel()/8*width + lImage->GetPaddingX();
  unsigned char* dataBuffer = (unsigned char*)lImage;
  const size_t data_size = lImage->GetImageSize();
  if (image_msg.data.size() != data_size)
  {
      image_msg.data.resize(data_size);
  }
  memcpy(&image_msg.data[0], lImage->GetDataPointer(), lImage->GetImageSize());
  cout<<"Width: "<<width<<" Height: "<<height<<endl;

  lPipeline->ReleaseBuffer(buffer);
  return true;
}


    virtual ~UsbCamNode()
    {
      cam_.shutdown();
    }

    bool take_and_send_image()
    {
      // grab the image
      cam_.grab_image(&img_);

      // grab the camera info
      sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
      ci->header.frame_id = img_.header.frame_id;
      ci->header.stamp = img_.header.stamp;

      // publish the image
      image_pub_.publish(img_, *ci);

      return true;
    }

    bool spin()
    {
      ros::Rate loop_rate(this->framerate_);
      while (node_.ok())
      {
        if (cam_.is_capturing()) {
          if (!take_and_send_image()) ROS_WARN("USB camera did not respond in time.");
        }
        ros::spinOnce();
        loop_rate.sleep();

      }
      return true;
    }

};
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hrl_thermal_cam");
  hrl_thermal_cam::HRLThermalCameraNode a;
  a.spin();
  return EXIT_SUCCESS;
}


