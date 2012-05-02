
// Using the Input Subsystem, Part II

// http://www.linuxquestions.org/questions/linux-desktop-74/hal-vs-xserver-dev-input-mice-convincing-x-to-*not*-use-a-given-input-device-697802/

// Wouse : Wince mouse.  Input device that detects a user wincing to produce some type of event.
//  In this case the wouse is used as to produce a runstop event for a PR2.

// This node uses events from Swiftpoint mouse to provide runstop signal as message on ROS topic.
// Node is meant to be used as safety mechanism, extra considuration is required:
//  1. What happeds if node crashes or wouse runs out of batteries or gets unplugged?
//  2. If networking buffers/delays messages.  IE, target system ROS messages but they are delayed by 1 or more seconds.
    
// Currently it is possible, to make a ROS service call that will halt motors on a PR2.  
// This is not a reliable runstop method b 

// Pulish data every <pub_timeout> seconds, even if button state has not changed
// this should allow rostopic message to work as a runstop keep-alive
// if signal does

#include <linux/input.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <wouse/evdev_listener.h>

#include <sstream>

namespace evdev_listener
{

EventInput::EventInput() 
  : fd_(-1)
  , initialized_(false)
{
  wince_count_ = 0;
};

bool EventInput::init(const char *input_device, ros::NodeHandle nh)
{
  ROS_INFO("Opening device '%s'", input_device);
  fd_ = open(input_device, O_RDONLY);
  if (fd_ == -1)
  {
    int error = errno;
    ROS_ERROR("Opening device %s : %s", input_device, strerror(error));
    return false;
  }

  /* ioctl() accesses the underlying driver */
  uint32_t version;
  if (ioctl(fd_, EVIOCGVERSION, &version))
  {    
    int error = errno;
    if (error == ENOTTY)
    {
      ROS_ERROR("'%s' does not seem to be an input event device file. "
                "Input event devices are often named '/dev/input/eventX' where X is an integer.", 
                input_device);
    }
    else
    {
      ROS_ERROR("EVIOCGVERSION ioctl failed : %s", strerror(error));
    }
    return false;
  }

  /* the EVIOCGVERSION ioctl() returns an int */
  /* so we unpack it and display it */
  ROS_INFO("Event driver version is %d.%d.%d",
           version >> 16, (version >> 8) & 0xff, version & 0xff);
  
  if (version != EV_VERSION)
  {
    version = EV_VERSION;
    ROS_WARN("This code was compiled deferent input device header version : %d.%d.%d",
             version >> 16, (version >> 8) & 0xff, version & 0xff);
  }


  // Get id : Id should help to determine is device actually a the Swiftpoint Mouse
  struct input_id id;
  if (ioctl(fd_, EVIOCGID, &id)) 
  {
    int error = errno;
    ROS_ERROR("EVIOCGID ioctl : %s", strerror(error));
    return false;
  }

  if (id.bustype != BUS_USB)
  {
    ROS_WARN("Input device does not seem to be a USB device. "
             "Expected bustype of %d, instead of %d.", 
             BUS_USB, id.bustype);
  }
  else if ((id.vendor != SWIFTPOINT_USB_VENDOR_ID) || (id.product != SWIFTPOINT_USB_PRODUCT_ID))
  {
    ROS_WARN("Input device vendor and product ID of %04X:%04X does not match expected value of %04X:%04X",
             int(id.vendor), int(id.product), SWIFTPOINT_USB_VENDOR_ID, SWIFTPOINT_USB_PRODUCT_ID);
  }           
           
  ROS_DEBUG("Input device id bustype:%u, vendor:%04X, product:%04X, version:%04X\n",
            (unsigned)id.bustype,
            (unsigned)id.vendor,
            (unsigned)id.product,
            (unsigned)id.version);


  // Get string description of device
  char buf[256];
  int buf_len;


  buf_len = ioctl(fd_, EVIOCGNAME((sizeof(buf)-1)), buf);
  if (buf_len < 0) 
  {
    int error = errno;
    ROS_WARN("EVIOCGNAME ioctl : %s", strerror(error));
  }
  else 
  {
    buf[buf_len] = '\0';  
    if (buf_len > 0)
    {
      ROS_INFO("Device name is '%s'", buf);
    }
    else
    {
      ROS_INFO("Device has no name (string length = 0)");
    }
  }


  buf_len = ioctl(fd_, EVIOCGPHYS(sizeof(buf)-1), buf);
  if (buf_len < 0) 
  {
    int error = errno;
    ROS_WARN("EVIOCGPHYS ioctl : %s", strerror(error));
    return false;
  }
  else 
  {
    buf[buf_len] = '\0';
    if (buf_len>0)
    {
      ROS_INFO("Device physical location is '%s'", buf);
    }
  }


  buf_len = ioctl(fd_, EVIOCGUNIQ(sizeof(buf)-1), buf);
  if (buf_len < 0) 
  {
    int error = errno;
    ROS_WARN("EVIOCGUNIQ ioctl : %s", strerror(error));
  }
  else 
  {
    buf[buf_len] = '\0';
    if (buf_len>0)
    {
      ROS_INFO("Unique identifier for device is '%s'", buf);
    }
    else 
    {
      ROS_INFO("No unique identifier for device");
    }
  }


  uint32_t event_bitflags = 0;
  if (ioctl(fd_, EVIOCGBIT(0, sizeof(event_bitflags)), &event_bitflags) < 0) 
  {
    int error = errno;
    ROS_ERROR("EVIOCGBIT ioctl : %s", strerror(error));    
    return false;
  }
  
  unsigned ev_max = EV_MAX;
  if (ev_max > (8*sizeof(event_bitflags)) )
  {
    ROS_WARN("ev_max > %u", unsigned(8*sizeof(event_bitflags)) );
    ev_max = 8*sizeof(event_bitflags);
  }
  
  std::ostringstream os;
  bool first = true;
  os << "Event types provided by device : ";
  for (unsigned ev_type_index = 0; ev_type_index < ev_max; ++ev_type_index) 
  {
    if ( (event_bitflags>>ev_type_index)&1 ) 
    {
      if (!first)
      {
        os << ", ";
      }
      first = false;
      /* the bit is set in the event types list */
      switch (ev_type_index)
      {
      case EV_SYN :
        os << "Synchronization";
        break;
      case EV_KEY :
        os << "Keys/Buttons";
        break;
      case EV_REL :
        os << "Relative Axes";
        break;
      case EV_ABS :
        os << "Absolute Axes";
        break;
      case EV_MSC :
        os << "Miscellaneous";
        break;
      case EV_LED :
        os << "LEDs";
        break;
      case EV_SND :
        os << "Sounds";
        break;
      case EV_REP :
        os << "Repeat";
        break;
      case EV_FF :
      case EV_FF_STATUS:
        os << "Force Feedback";
        break;
      case EV_PWR:
        os << "Power Management";
        break;
      default:
        os << "Unknown(" << ev_type_index << ")";
      }
    }
  }
  
  if ( (event_bitflags & (1<<EV_SYN))  &&  (event_bitflags & (1<<EV_REL)) )
  {
    ROS_INFO_STREAM(os.str());
  }
  else
  {
    ROS_ERROR("Device must provide 'Synchronization' and 'Relative Axes' events for driver to work. %s", 
              os.str().c_str());
    return false;
  }


  // make fd non-blocking
  int flags = fcntl(fd_, F_GETFL);
  if (flags == -1)
  {
    int error = errno;
    ROS_ERROR("Getting file descriptor flags : %s", strerror(error));
    return false;
  }  
  if (fcntl(fd_, F_SETFL, flags|O_NONBLOCK) == -1)
  {
    int error = errno;
    ROS_ERROR("Setting file descriptor flags : %s", strerror(error));
    return false;
  }

  // Steal mouse away from operating system so it does cause mouse movements
  int grab = 1;
  if (ioctl(fd_, EVIOCGRAB, &grab) < 0)
  {
    int error = errno;
    ROS_ERROR("EVIOCGRAB ioctl : %s", strerror(error));
    return false;
  } 

  movement_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>("wouse_movement", 100);
  wince_pub_ = nh.advertise<std_msgs::Int32>("wince_count", 1, true);

  initialized_ = true;
  return true;
}



bool EventInput::run()
{
  if (!initialized_)
  {
    return false;
  }

  int num_event_slots = 64;
  struct input_event *event_buf = new input_event[num_event_slots];
  fd_set read_set, except_set;
  struct timeval timeout;
  double delta_x=0, delta_y=0;  // keep track of delta positions between event.
  bool first = true;

  ros::Duration wince_publish_duration(0.1);
 
  bool is_error = false;
  
  publishWince();

  while (!is_error && ros::ok())
  {
    FD_ZERO(&read_set);    
    FD_ZERO(&except_set);    
    FD_SET(fd_, &read_set);
    FD_SET(fd_, &except_set);
    timeout.tv_sec = 0; 
    timeout.tv_usec = 10 * 1000; // 100ms = 1/10th second
    int result = select(fd_+1, &read_set, NULL, &except_set, &timeout);
    if (result < 0)
    {
      int error = errno;
      ROS_ERROR("Select : %s", strerror(error)); 
      is_error = true;
    }
    else if (result == 0)
    {
      //timeout -- no new data      
      // ROS_WARN("Timeout");
    }
    else 
    {
      // either read set of 
      if (FD_ISSET(fd_, &except_set))
      {
        ROS_WARN("Exception set");
      }
      if (!FD_ISSET(fd_, &read_set))
      {
        ROS_WARN("Read bit not set");
      }
     
      size_t read_len = read(fd_,event_buf,sizeof(struct input_event)*num_event_slots);
      if (int(read_len) == -1)
      {
        int error = errno;
        ROS_ERROR("Read : %s", strerror(error));
        is_error = true;
      }
      else if ( (read_len%sizeof(struct input_event) ) != 0)
      {
        ROS_ERROR("%d leftover bytes after read",
                  int(read_len%sizeof(struct input_event)));
        is_error = true;
      }
      else 
      {
        int num_events = read_len / sizeof(struct input_event);
        for (int ii = 0; ii<num_events; ++ii)
        {
          const struct input_event &event(event_buf[ii]);          
          // swiftpoint mouse seems to support 4 event types:
          // EV_SYN, EV_KEY, EV_REL, and EV_MSC
          // For tracking mouse movements, we track REL movements for both X, and Y
          // And publish the data once we get a SYN (sync).

          // Use  event time since is will probabbly produce a more acurrate
          // duration and movement velocities than ros::Time::now()
          ros::Time current_event_time;
          current_event_time.sec = event.time.tv_sec;
          current_event_time.nsec = event.time.tv_usec * 1000;
          
          if (event.type == EV_SYN)
          {// Event Sync. Publish the previous delta_x, delta_y values

            publishMovement(delta_x, delta_y, current_event_time);

            first = false;
            delta_x = 0; delta_y=0;
          }
          else if ((event.type == EV_REL) && (event.code == REL_X))
          {
            delta_x = event.value;
          }
          else if ((event.type == EV_REL) && (event.code == REL_Y))
          {
            delta_y = event.value;
          }
          else
          { //debugging
             printf("%ld.%06ld ",
                    event.time.tv_sec,
                    event.time.tv_usec);
             printf(" type %d, code %d value %x\n",
                    event.type,event.code, event.value);   
          }
        } // end for
      } // end if(read_result)
    } //end select
      
    if ( (ros::Time::now() - last_wince_publish_time_) > wince_publish_duration)
    {
      //ROS_INFO("wince republish");
      publishWince();
    }

  } //end while (ros::ok())

  // as node is exiting send a runstop message
  return is_error;
}

void EventInput::publishWince()
{
  last_wince_publish_time_ = ros::Time::now();
  std_msgs::Int32 wince_msg;  
  wince_msg.data = wince_count_;
  wince_pub_.publish(wince_msg);
}

void EventInput::publishMovement(double delta_x, double delta_y, ros::Time time)
{
  geometry_msgs::Vector3Stamped movement_msg;
  movement_msg.header.stamp = time;
  movement_msg.vector.x = delta_x;
  movement_msg.vector.y = delta_y;
  movement_msg.vector.z = 0;
  movement_pub_.publish(movement_msg);
}

}; //end namespace event_listener

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "wouse_evdev_listener");
  ros::NodeHandle nh;

  if (argc < 2)
  {
    fprintf(stderr, "Please provide device path : ie /dev/wouse \n");
    exit(EXIT_FAILURE);
  }

  evdev_listener::EventInput evdev_listener;
  
  if (!evdev_listener.init(argv[1], nh))
  {
    return 1;
  }

  if (!evdev_listener.run())
  {
    return 1;
  }

  return 0;
}
