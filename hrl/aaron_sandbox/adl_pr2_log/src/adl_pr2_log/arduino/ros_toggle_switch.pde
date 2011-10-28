/* 
 * Electric Shaver Switch for Rosserial
 */

#include <ros.h>
#include <std_msgs/Bool.h>
//#include <std_msgs/String.h>

const int ledPin = 50;
bool cur_state = false;

ros::NodeHandle nh;
std_msgs::Bool state_msg;
ros::Publisher switch_state("/ros_switch_state", &state_msg);

//std_msgs::String str_msg;
//ros::Publisher switch_status("/shaver_state", &str_msg);
//unsigned char message[19] = "Pressed the switch";

ROS_CALLBACK(msg_cb, std_msgs::Bool, msg)
  if (msg.data ^ cur_state){
    digitalWrite(ledPin,HIGH);
    delay(400);
    digitalWrite(ledPin,LOW);
  };
  cur_state = msg.data;
}

std_msgs::Bool switch_msg;
ros::Subscriber sub_switch("/ros_switch", &switch_msg, &msg_cb);


void setup()
{
  nh.initNode();
  nh.advertise(switch_state);
  nh.subscribe(sub_switch);
  pinMode(ledPin, OUTPUT);
 
}

void loop()
{
  state_msg.data = cur_state;
  switch_state.publish( &state_msg );
  nh.spinOnce();
  delay(10);
}
