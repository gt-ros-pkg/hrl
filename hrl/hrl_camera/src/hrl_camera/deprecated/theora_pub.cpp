#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
  

class Theora_obj
{
  public:
    image_transport::Publisher pub;
    void callback(const sensor_msgs::ImageConstPtr& msg);
    void set_pub(image_transport::ImageTransport& it);
      
};

void Theora_obj::set_pub(image_transport::ImageTransport& it)
{
    pub = it.advertise("hrl_compressed_img/image", 1);
}

void Theora_obj::callback(const sensor_msgs::ImageConstPtr& msg)
{
    pub.publish(msg);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "theora_publish");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  Theora_obj t_obj;
  t_obj.set_pub(it);
  ros::Subscriber cam_image = nh.subscribe(argv[1], 1, &Theora_obj::callback, &t_obj);

  ros::spin();
}


