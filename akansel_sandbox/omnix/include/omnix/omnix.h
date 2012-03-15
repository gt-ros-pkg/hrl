#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <omnix/buttons.h>
#include <omnix/EngageBrake.h>
#include <omnix/ReleaseBrake.h>
#include <omnix/DisableJoystick.h>
#include <omnix/EnableJoystick.h>
//#include <omnix/WheelPos.h>

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

class OmnixNode{
 private:
  ros::Time last_vel_cmd_recd;
  ros::Time last_vel_cmd_sent;
  ros::Time last_estop_recd;

  geometry_msgs::Twist* last_vel_cmd;

  ros::ServiceServer engage_brake_service_;
  ros::ServiceServer disengage_brake_service_;
  ros::ServiceServer get_joint_position_service_;
  ros::ServiceServer disable_joystick_service_;

  std::string hostname;
  int portno;
  int sock;
  struct hostent *host;
  struct sockaddr_in server_addr;
  int server_addr_len;

  int portno_rec;
  int sock_rec;
  struct hostent *host_rec;
  struct sockaddr_in server_addr_rec;
  int server_addr_rec_len;
  struct sockaddr_in server_addr_rec_other;
  int server_addr_rec_other_len;

  int counter;

  bool emergency_stop;
  bool first_write;
  bool brake_engaged;
  bool joystick_controlled;
  bool recv_odom;
  bool verbose;
  bool open_rec_socket;
  char rec_buf[1024];
  int rec_buf_size;

  //odom info
  double odom_x, odom_y, odom_yaw;
  double prev_j1, prev_j2, prev_j3, prev_j4;
  double vx, vy, vt;
  ros::Time prev_time;

  //Commands
  std::string DisableJoystickCommand;
  std::string EnableJoystickCommand;
  std::string EnableRobotControlCommand;//TODO: NYI (not needed)
  std::string DisableRobotControlCommand;
  std::string SetVelocityCommand;
  std::string SetJointVelocitiesCommand;
  std::string GetJointPositionCommand;
  std::string EngageBrakeCommand;
  std::string ReleaseBrakeCommand;
  std::string SetBlinkPatternCommand;//TODO: NYI

 public:
  ros::NodeHandle n_;
  tf::TransformBroadcaster tf_;
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber joy_sub_;
  ros::Timer estop_timer_;
  ros::Timer read_timer_;
  ros::Duration estop_timeout_;
  ros::Publisher odom_pub_;
  ros::Publisher wheel_pos_pub_;

  OmnixNode();
  ~OmnixNode();
  void openOmnixSocket();
  void openOmnixSocketRec();
  void closeOmnixSocket();
  bool sendCommand(const std::string& cmd);
  void sendGetJointPosition();
  void cmdvelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel);
  bool EngageBrakeCallback(omnix::EngageBrake::Request &req, omnix::EngageBrake::Response &res);
  bool ReleaseBrakeCallback(omnix::ReleaseBrake::Request &req, omnix::ReleaseBrake::Response &res);
  bool DisableJoystickCallback(omnix::DisableJoystick::Request &req, omnix::DisableJoystick::Response &res);
  bool EnableJoystickCallback(omnix::EnableJoystick::Request &req, omnix::EnableJoystickResponse &res);
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void timerCallback(const ros::TimerEvent& e);
  void readTimerCallback(const ros::TimerEvent& e);
  bool recvResponse(bool blocking=true);
  bool peekForNewline();

  void initCommands()
  {
    DisableJoystickCommand = "DJS";
    EnableJoystickCommand = "EJS";
    EnableRobotControlCommand = "ERC";
    DisableRobotControlCommand = "DRC";
    SetVelocityCommand = "SOV";
    SetJointVelocitiesCommand = "SJV";
    GetJointPositionCommand = "GJP";
    EngageBrakeCommand = "EBK";
    ReleaseBrakeCommand = "RBK";
    SetBlinkPatternCommand = "SBP";
  }

};
