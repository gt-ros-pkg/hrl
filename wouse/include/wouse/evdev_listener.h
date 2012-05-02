#ifndef WOUSE__WOUSE_INPUT__H
#define WOUSE__WOUSE_INPUT__H

#include <ros/ros.h>

namespace evdev_listener
{


class EventInput
{
public:
  EventInput();
  bool init(const char *input_device, ros::NodeHandle nh);
  bool run();

  static const int SWIFTPOINT_USB_VENDOR_ID = 0x214E;
  static const int SWIFTPOINT_USB_PRODUCT_ID = 1;

protected:
  void publishWince();
  void publishMovement(double delta_x, double delta_y, ros::Time time);

protected:
  int fd_; //!< file decriptor for mouse event input
  bool initialized_;
  ros::Publisher movement_pub_;
  ros::Publisher wince_pub_;

  ros::Time last_wince_publish_time_;
  int wince_count_;
};



}; //end namespace evdev_listener 


#endif //WOUSE__WOUSE_INPUT__H

