#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>

static const std::string OPENCV_WINDOW = "Image window";

using namespace std;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  uint16_t temp_data[324*256];
  uint16_t cur_max;
  uint16_t cur_min;
  uint16_t cutoff;
  //int root_package_;

public:
  ImageConverter() :
      it_(nh_)
  {
    ros::NodeHandle nodeIC_("~");
    std::string root_package_;
    nodeIC_.param<std::string>("root_package",root_package_,"");
    std::cout<<"the root package is "+root_package_<<std::endl; 


    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe(root_package_+"/thermal_camera/image_raw", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("thermal_camera/bw_converted_image", 1);

//    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
//    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
    }
    catch (cv_bridge::Exception& e)
    {
      std::cout<<"ERROR FROM CV BRIDGE FAILING TO CONVERT!!!!!\n";
      std::cout<<"ERROR FROM CV BRIDGE FAILING TO CONVERT!!!!!\n";
      std::cout<<"ERROR FROM CV BRIDGE FAILING TO CONVERT!!!!!\n";
      std::cout<<"ERROR FROM CV BRIDGE FAILING TO CONVERT!!!!!\n";
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

//  std::cout << "Data size:" << img_msg->data.size();
//  union
//  {
//    uint32_t value;
//    char bytes[sizeof(uint32_t)];
//  } color;
//  float temp_data[(msg->height)*(msg->width)] = {};
//  uint8_t bw_data [324] = {};
//  double *cur_max;// = 0;
//  double *cur_min;// = 65535;
//  std::cout<< "data size: " << msg->data.size() <<"\n";
//  std::cout<< "temp data size: " << temp_data.size() <<"\n";

//  for (uint i=0; i<  (msg->height)*(msg->width); ++i)
//  {
//
//    for (uint j=0; j< (msg->height)*(msg->width); ++i)
//    {
//    ind = 2*(int(uv.y)*thermal_image_msg->width + int(uv.x));
//    int ind = 2*i; // + j*bw_msg.width*2;
//    std::cout<<"ind: "<<ind<<" ";
//    std::cout<<"data: "<<msg->data[ind]<<" "<<msg->data[ind+1]<<" ";
//    color.bytes[0]=msg->data[ind];
//    color.bytes[1]=msg->data[ind+1];
//    temp_data[i] = static_cast<float>(color.value);
//    std::cout<<"temp: "<<temp_data[i]<<"\n";
//    if(temp_data[i] > cur_max) cur_max = temp_data[i];
//    if(temp_data[i] < cur_min) cur_min = temp_data[i];
//  }
//  float cutoff = (cur_max - cur_min)/2.;
//
//  for (uint i=0; i< bw_msg.width; ++i)
//  {
////    ind = 2*(int(uv.y)*thermal_image_msg->width + int(uv.x));
//    temp_data[i] = static_cast<float>(color.value);
//    if(temp_data[i] > cutoff)
//    {
//      bw_data[i] = 255;
//    }
//    else
//    {
//      bw_data[i] = 0;
//    }
//  }
//
//  *cur_min = 65535.;
//  *cur_max = 0.;

//  cv::minMaxLoc(cv_ptr->image, cur_min, cur_max);
//  cutoff = (*cur_max - *cur_min)/2.;
//  std::cout<<"Min: "<< *cur_min<< " Max: "<<*cur_max<<"\n";
//  std::cout<<"Cutoff: "<< cutoff<<"\n";
  cur_min = 65535;
  cur_max = 0;
  for(int i = 0; i < cv_ptr->image.rows; i++){
   for(int j = 0; j < cv_ptr->image.cols; j++){

     if (cv_ptr->image.at<uint16_t>(i,j) > cur_max) cur_max = cv_ptr->image.at<uint16_t>(i,j);
     if (cv_ptr->image.at<uint16_t>(i,j) < cur_min) cur_min = cv_ptr->image.at<uint16_t>(i,j);
   }
  }
//  std::cout<<"Min: "<< cur_min<< " Max: "<<cur_max<<"\n";
//  cutoff = cur_min + (cur_max - cur_min)/2;
//  cutoff = 4000;
//  std::cout<<"Cutoff: "<< cutoff<<"\n";
  for(int i = 0; i < cv_ptr->image.rows; i++){
   for(int j = 0; j < cv_ptr->image.cols; j++){
     cv_ptr->image.at<uint16_t>(i,j)= uint16_t(255.0*(cur_max - cv_ptr->image.at<uint16_t>(i,j))/((cur_max - cur_min)));
//     if (cv_ptr->image.at<uint16_t>(i,j) > 3.*255./4.)
//     {
//       cv_ptr->image.at<uint16_t>(i,j) = 255;
//     }
//     if (cv_ptr->image.at<uint16_t>(i,j) < 255./4.)
//     {
//       cv_ptr->image.at<uint16_t>(i,j) = 0;
//     }
   }
  }
//      int h = i;
//     std::cout<<"value: "
      //apply condition here
//      if (temp_data[i+(cv_ptr->image.rows)*j] > cur_max)
//      {
//        cv_ptr->image.at<int>(i,j) = 255;
//      }
//      else
//      {
//        cv_ptr->image.at<int>(i,j) = 0;
//      }
//      std::cout<< "value: " << cv_ptr->image.at<uint16_t>(i,j)<<"\n";
//      if(image.at<uint16_t>(i,j) > cur_max) cur_max = temp_data[i];
//      if(image.at<uint16_t>(i,j) < cur_min) cur_min = temp_data[i];
//      if(image.at<uint16_t>(i,j) == threshold){
//         image.at<uint16_t>(i,j) = 255;
//
//   }
//  }

    // Draw an example circle on the video stream
//    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
//      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
//    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
//    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
    //cout << cv_ptr->toImageMsg() << endl;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
