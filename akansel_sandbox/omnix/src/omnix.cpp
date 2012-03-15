#include <omnix/omnix.h>


OmnixNode::OmnixNode():
n_("~")
{
  n_.param("omnix_ip_addr",hostname,std::string("130.207.32.211"));
  //n_.param("omnix_ip_addr",hostname,std::string("127.0.0.1"));
  n_.param("omnix_port",portno,5555);
  n_.param("omnix_port_rec",portno_rec,5556);
  n_.param("open_rec_socket",open_rec_socket,true);

  initCommands();
  emergency_stop = true;
  brake_engaged = true;
  joystick_controlled = true;
  first_write = true;
  recv_odom = false;
  counter = 1;
  rec_buf_size = 1024;
  verbose = false;
  

  estop_timer_ = n_.createTimer(ros::Duration(0.1),&OmnixNode::timerCallback,this);
  estop_timeout_ = ros::Duration(0.1);

  read_timer_ = n_.createTimer(ros::Duration(0.02),&OmnixNode::readTimerCallback,this);

  engage_brake_service_ = n_.advertiseService("engage_brake",&OmnixNode::EngageBrakeCallback,this);
  disengage_brake_service_ = n_.advertiseService("disengage_brake",&OmnixNode::ReleaseBrakeCallback,this);
  disable_joystick_service_ = n_.advertiseService("disable_joystick",&OmnixNode::DisableJoystickCallback,this);

  odom_pub_ = n_.advertise<nav_msgs::Odometry>("/odom", 1);
  //wheel_pos_pub_ = n_.advertise<omnix::WheelPos>("/wheel_pos",1);

  //get_joint_position_service_ = n_.advertiseService("/get_joint_position",&OmnixNode::

  last_vel_cmd_sent = ros::Time::now();
  last_vel_cmd = new geometry_msgs::Twist();
  last_vel_cmd->linear.x = 0;
  last_vel_cmd->linear.y = 0;
  last_vel_cmd->angular.z = 0;

  odom_x = odom_y = odom_yaw = 0.0;
  prev_j1 = prev_j2 = prev_j3 = prev_j4 = 0.0;
  vx = vy = vt = 0.0;
  prev_time = ros::Time::now();

  joy_sub_ = n_.subscribe<sensor_msgs::Joy>("/joy", 1, &OmnixNode::joyCallback, this);

  cmd_vel_sub_ = n_.subscribe<geometry_msgs::Twist>("/cmd_vel",1,boost::bind(&OmnixNode::cmdvelCallback, this, _1));

  openOmnixSocket();
  ROS_INFO("Opening receiving socket...");
  if(open_rec_socket){
    openOmnixSocketRec();
    ROS_INFO("Receive socket online, ready to stream odometry.");
  }
}

OmnixNode::~OmnixNode()
{
  closeOmnixSocket();
}

void OmnixNode::openOmnixSocket()
{
  ROS_INFO("Attempting to open socket to Omnix at %s : %d",hostname.c_str(),portno);
  //host = gethostbyname(hostname.c_str());
  gethostbyname(hostname.c_str());
  ROS_INFO("Got host by name");

  if((sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1){
      ROS_ERROR("OmnixNode: Error opening UDP socket.  Exiting!");
      exit(1);
  }

  ROS_INFO("Socket opened.");
  memset((char *) &server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(portno);
  if(inet_aton(hostname.c_str(), &server_addr.sin_addr)==0){
      ROS_ERROR("OmnixNode: inet_aton() failed.  Exiting!");
      exit(1);
  }
  server_addr_len = sizeof(server_addr);
  ROS_INFO("Ready for communication!");
}

void OmnixNode::openOmnixSocketRec()
{
  ROS_INFO("Attempting to open recieving socket to Omnix at port %d",portno_rec);
  //host = gethostbyname(hostname.c_str());
  //gethostbyname(hostname.c_str());
  //ROS_INFO("Got host by name");

  if((sock_rec = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1){
      ROS_ERROR("OmnixNode: Error opening UDP receiving socket.  Exiting!");
      exit(1);
  }

  ROS_INFO("Recv Socket opened.");
  memset((char *) &server_addr_rec, 0, sizeof(server_addr_rec));
  server_addr_rec.sin_family = AF_INET;
  server_addr_rec.sin_port = htons(portno_rec);
  server_addr_rec.sin_addr.s_addr = htonl(INADDR_ANY);
//if(inet_aton(hostname.c_str(), &server_addr_rec.sin_addr)==0){
  //    ROS_ERROR("OmnixNode: inet_aton() failed.  Exiting!");
  //    exit(1);
  //}
  server_addr_rec_len = sizeof(server_addr_rec);
  server_addr_rec_other_len = sizeof(server_addr_rec_other);
  if(bind(sock_rec, (struct sockaddr*)&server_addr_rec, server_addr_rec_len)==-1){
    ROS_ERROR("OmnixNode: Error binding recv socket, exiting!");
    exit(1);
  }

  ROS_INFO("Ready for recv communication!");
}

void OmnixNode::closeOmnixSocket()
{
  close(sock);
  close(sock_rec);
}

void OmnixNode::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  if(joy->buttons[PS3_SHOULDER_L2]){
    last_estop_recd = ros::Time::now();
    emergency_stop = false;
  }
}

void OmnixNode::readTimerCallback(const ros::TimerEvent& e)
{
  recvResponse(false);
}

void OmnixNode::timerCallback(const ros::TimerEvent& e)
{
  //Check the emergency stop!
  if((ros::Time::now() - last_estop_recd) > estop_timeout_){
    emergency_stop = true;
  }
  if(!emergency_stop){
    
  }

  //check for data in the pipe
  //recvResponse(false);
  //Update odom info
  if(recv_odom){
  //if(false){
    sendGetJointPosition();

    //Publish odom via tf
    tf::Transform odom = tf::Transform(tf::createQuaternionFromYaw(odom_yaw),tf::Point(odom_x,odom_y,0.0));
    tf_.sendTransform(tf::StampedTransform(odom, ros::Time::now(),"odom","base_link"));
    
    //Publish odom via odom
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "odom";
    odom_msg.pose.pose.position.x = odom_x;
    odom_msg.pose.pose.position.y = odom_y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_yaw);
    
    odom_msg.child_frame_id = "base_link";
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.angular.z = vt;

    for(int i = 0; i < 35; i++){
      odom_msg.pose.covariance[i] = 0.000;
      odom_msg.twist.covariance[i] = 0.000;
    }
    odom_msg.pose.covariance[0] = 0.1;
    odom_msg.pose.covariance[7] = 0.1;
    odom_msg.pose.covariance[14] = 0.1;
    odom_msg.pose.covariance[21] = 0.1;
    odom_msg.pose.covariance[28] = 0.1;
    odom_msg.pose.covariance[35] = 0.1;
    odom_msg.twist.covariance[0] = 0.1;
    odom_msg.twist.covariance[7] = 0.1;
    odom_msg.twist.covariance[14] = 0.1;
    odom_msg.twist.covariance[21] = 0.1;
    odom_msg.twist.covariance[28] = 0.1;
    odom_msg.twist.covariance[35] = 0.1;


    odom_pub_.publish(odom_msg);
  }
}

bool OmnixNode::sendCommand(const std::string& cmd)
{
  char buf[2048];
  int c = counter % 1000;
  sprintf(buf,"%d %s\n",c,cmd.c_str());
  int cmdlen = strlen(buf)+1;
  if(sendto(sock,buf,cmdlen,0,(struct sockaddr*)&server_addr,server_addr_len) != cmdlen){
      ROS_ERROR("OmnixNode: Error sending on socket to Omnix!");
      return false;
  }
  counter++;

  //recvResponse();
  return true;
}

bool OmnixNode::peekForNewline()
{
  int recd_size;
  int flags = 0 | MSG_PEEK | MSG_DONTWAIT;
  bool done = false;
  while(!done){
    ROS_INFO("Peeking for newline...");
    recd_size = recvfrom(sock_rec, rec_buf, rec_buf_size, flags, (struct sockaddr*)&server_addr_rec_other, (socklen_t*)&server_addr_rec_other_len);
    ROS_INFO("Got %d bytes",recd_size);
    size_t found;
    std::string recd_str = std::string(rec_buf);
    found = recd_str.find("\n");
    if(found==std::string::npos){
      ros::Duration(0.1).sleep();
    } else {
      ROS_INFO("Found a newline!");
      done = true;
    }
  }
}

bool OmnixNode::recvResponse(bool blocking)
{
  if(!open_rec_socket){
    return false;
  }
  //peekForNewline();

  int recd_size;
  int flags = 0;
  if(!blocking){
    flags |= MSG_DONTWAIT;
  }

  //ROS_INFO("Waiting for response from platform.");
  recd_size = recvfrom(sock_rec, rec_buf, rec_buf_size, flags, (struct sockaddr*)&server_addr_rec_other, (socklen_t*)&server_addr_rec_other_len);
  //ROS_INFO("Got %d bytes",recd_size);
  if(recd_size==-1){
    if(blocking){
      ROS_ERROR("OmnixNode: Error receiving from Omnix socket!");
    }
    return false;
  } else {
    if(verbose){
      ROS_INFO("Got string: %s \n",rec_buf);
    }
  }

  char* tok_ptr;
  tok_ptr = strtok(rec_buf," ");
  if(tok_ptr != NULL){
    if(strcmp(tok_ptr,"OK")==0){
      if(verbose){
	ROS_INFO("OmnixNode OK: %s\n",rec_buf);
      }
      tok_ptr = strtok(NULL," ");
      
      if(tok_ptr == NULL){
	ROS_INFO("Error parsing repsonse!");
	return false;
      }
      if(strcmp(tok_ptr,"SOV")==0){
	return true;
      }
      if(strcmp(tok_ptr,"GJP")==0){
	double j1,j2,j3,j4 = 0;
	tok_ptr = strtok(NULL," ");
	if(tok_ptr == NULL){
	  ROS_INFO("Error parsing repsonse!");
	  return false;
	}
	sscanf(tok_ptr,"%lf",&j1);
	tok_ptr = strtok(NULL," ");
	if(tok_ptr == NULL){
	  ROS_INFO("Error parsing repsonse!");
	  return false;
	}
	sscanf(tok_ptr,"%lf",&j2);
	tok_ptr = strtok(NULL," ");
	if(tok_ptr == NULL){
	  ROS_INFO("Error parsing repsonse!");
	  return false;
	}
	sscanf(tok_ptr,"%lf",&j3);
	tok_ptr = strtok(NULL," ");
	if(tok_ptr == NULL){
	  ROS_INFO("Error parsing repsonse!");
	  return false;
	}
	sscanf(tok_ptr,"%lf",&j4);	
	
	//ROS_INFO("Wheel pos: %lf %lf %lf %lf",j1,j2,j3,j4);
	//Calc delta joint positions
	double dj1 = j1 - prev_j1;
	double dj2 = j2 - prev_j2;
	double dj3 = j3 - prev_j3;
	double dj4 = j4 - prev_j4;
	
	//get delta platform pose from joint positions
	double dx = 0.0;
	double dy = 0.0;
	double dt = 0.0;
	
	bool rotating = false;
	if(((dj2 > 0.0) && (dj4 < 0.0)) || ((dj2 < 0.0) && (dj4 > 0.0))){
	  rotating = true;
	  //ROS_INFO("Rotating!");
	  dj3 = -dj1;
	} else {
	  //ROS_INFO("Not rotating!");
	  dj3 = dj1;
	  }
 
	if(true){
	  dx = (dj1 + dj2 + dj3 + dj4) / 4.0;
	  dy = (dj1 - dj2 + dj3 - dj4) / 4.0;
	  //dt = (dj1 + dj2 - dj3 - dj4) / 4.0;
	  dt = -(dj2 - dj4) / 2.0; 
	} else {
	  //dx = ((dj1*2.0) + dj2 + dj4) / 3.0;
	  //dy = ((dj1*2.0) - dj2 - dj4) / 3.0;
	  dx = (dj1 + ((dj2 + dj4)/2.0)) / 2.0;
	  dy = (dj1 + ((-dj2 - dj4)/2.0)) / 2.0;
	  dt = -(dj2 - dj4) / 2.0;
	}

	//ROS_INFO("dx: %lf dy: %lf dt: %lf",dx,dy,dt);
	
	if((dx > 0.01) || (dy > 0.01)){
	  ROS_INFO("Jump detected: %lf %lf %lf",dx,dy,dt);
	  ROS_INFO("Joint Jumps: %lf %lf %lf %lf",dj1,dj2,dj3,dj4);
	  ROS_INFO("Curr joints: %lf %lf %lf %lf", j1, j2, j3, j4);
	  ROS_INFO("Prev joints: %lf %lf %lf %lf", prev_j1, prev_j2, prev_j3, prev_j4);
	}

	//Calc platform velocities
	ros::Duration dtime = ros::Time::now() - prev_time;
	vx = dx / dtime.toSec();
	vy = dy / dtime.toSec();
	vt = dt / dtime.toSec();
	
	//integrate it with previous pose info
	odom_x += dx;
	odom_y += dy;
	odom_yaw += dt;
	//ROS_INFO("Platform pos: %lf %lf %lf",odom_x,odom_y,odom_yaw);
	//ROS_INFO("Platform vel: %lf %lf %lf",vx,vy,vt);
	
	//Store previous joint info
	prev_j1 = j1;
	prev_j2 = j2;
	prev_j3 = j3;
	prev_j4 = j4;
	prev_time = ros::Time::now();
	
      }

    } else if (strcmp(tok_ptr,"ERR")==0) {
      ROS_INFO("OmnixNode ERR: %s\n",rec_buf);
    }
  
  } else {
    if(blocking){
      ROS_INFO("Got NULL str!");
    }
   }
  
}

void OmnixNode::sendGetJointPosition()
{
  char buf[2048];
  int c = counter % 1000;
  sprintf(buf,"%d GJP",c);
  //ROS_INFO("Sending GJP command!");
  if(sendto(sock,buf,strlen(buf)+1,0,(struct sockaddr*)&server_addr,sizeof(server_addr)) != strlen(buf)+1){
      ROS_ERROR("OmnixNode: Error sending on socket to Omnix!");
  }
  counter++;

  //recvResponse();
  /*
  ROS_INFO("Receiving response from platform!");
  
  //if(recvfrom(sock, rec_buf, rec_buf_size, 0, (struct sockaddr*)&server_addr, sizeof(server_addr)==-1)){
  int recd_size;
  //int flags = 0 | MSG_DONTWAIT;
  int flags = 0;
  recd_size = recvfrom(sock_rec, rec_buf, strlen(rec_buf)+1, flags, (struct sockaddr*)&server_addr_rec_other, (socklen_t*)&server_addr_rec_other_len);
  ROS_INFO("Got %d bytes",recd_size);
  if(recd_size==-1){
      ROS_ERROR("OmnixNode: Error receiving from Omnix socket!");
  }
  ROS_INFO("Got string: %s \n",rec_buf);
  double j1,j2,j3,j4 = 0;
  ROS_INFO("calling sscanf!");
  sscanf(rec_buf,"OK GJP %lf %lf %lf %lf",&j1,&j2,&j3,&j4);
  ROS_INFO("Wheel pos: %lf %lf %lf %lf",j1,j2,j3,j4);
  omnix::WheelPos wmsg;
  wmsg.j1 = j1;
  wmsg.j2 = j2;
  wmsg.j3 = j3;
  wmsg.j4 = j4;
  wheel_pos_pub_.publish(wmsg);

  //Calc delta joint positions
  double dj1 = j1 - prev_j1;
  double dj2 = j2 - prev_j2;
  double dj3 = j3 - prev_j3;
  double dj4 = j4 - prev_j4;

  //get delta platform pose from joint positions
  double dx = (dj1 + dj2 + dj3 + dj4) / 4.0;
  double dy = (dj1 - dj2 + dj3 - dj4) / 4.0;
  double dt = (dj1 + dj2 - dj3 - dj4) / 4.0;
  ROS_INFO("dx: %lf dy: %lf dt: %lf",dx,dy,dt);

  //Calc platform velocities
  ros::Duration dtime = ros::Time::now() - prev_time;
  vx = dx / dtime.toSec();
  vy = dy / dtime.toSec();
  vt = dt / dtime.toSec();

  //integrate it with previous pose info
  odom_x += dx;
  odom_y += dy;
  odom_yaw += dt;
  ROS_INFO("Platform pos: %lf %lf %lf",odom_x,odom_y,odom_yaw);
  ROS_INFO("Platform vel: %lf %lf %lf",vx,vy,vt);

  //Store previous joint info
  prev_j1 = j1;
  prev_j2 = j2;
  prev_j3 = j3;
  prev_j4 = j4;
  prev_time = ros::Time::now();
  */
  //Note: publication happens in the timer callback.

}



void OmnixNode::cmdvelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
  if(joystick_controlled){
      ROS_ERROR("OmnixNode: Movement requested while ODV is in hardware joystick-controlled mode.  Ignoring!");
      return;
  }
  if(brake_engaged){
      ROS_ERROR("OmnixNode: Movement requested while brakes are on.  Ignoring!");
      return;
  }
  if(emergency_stop){
    if(verbose){
      ROS_ERROR("OmnixNode: Velocity Requested without deadman switch signal");
    }
    return;
  }

  if(this->first_write){
      last_vel_cmd = new geometry_msgs::Twist(*cmd_vel);
      this->first_write = false;
  }
  last_vel_cmd_recd = ros::Time::now();

  //  if((last_vel_cmd->linear.x != cmd_vel->linear.x) || (last_vel_cmd->linear.y != cmd_vel->linear.y) || (last_vel_cmd->angular.z != cmd_vel->angular.z)){
      last_vel_cmd = new geometry_msgs::Twist(*cmd_vel);

      char tempbuf[1024];
      sprintf(tempbuf," %lf %lf %lf", last_vel_cmd->linear.x, last_vel_cmd->linear.y, last_vel_cmd->angular.z);

      std::string vel_string = SetVelocityCommand + std::string(tempbuf);
      //build up a command!
      if(verbose){
	ROS_INFO("Would publish: %s",vel_string.c_str());
      }
      if(ros::Time::now() - last_vel_cmd_sent > ros::Duration(0.1)){
	sendCommand(vel_string);
	last_vel_cmd_sent = ros::Time::now();
      }
      //}

      if(verbose){
	ROS_INFO("OmnixNode: Got vel command: (X: %lf Y: %lf Theta: %lf",cmd_vel->linear.x,cmd_vel->linear.y,cmd_vel->angular.z);
      }
}

bool OmnixNode::EngageBrakeCallback(omnix::EngageBrake::Request &req, omnix::EngageBrake::Response &res)
{
  if(joystick_controlled){
      ROS_ERROR("OmnixNode: ODV currently in hardware joystick controlled mode.  Switch this off to first to enable UDP control.");
      return false;
  }
  return sendCommand(EngageBrakeCommand);
}

bool OmnixNode::ReleaseBrakeCallback(omnix::ReleaseBrake::Request &req, omnix::ReleaseBrake::Response &res)
{
  if(joystick_controlled){
      ROS_ERROR("OmnixNode: ODV currently in hardware joystick controlled mode.  Switch this off to first to enable UDP control.");
      return false;
  }
  bool result = sendCommand(ReleaseBrakeCommand);
  if(result){
    ROS_INFO("OmnixNode: Successfully released brake.");
    brake_engaged = false;
    recv_odom = true;
  } else {
    ROS_ERROR("OmnixNode: Error sending RBK command!");
  }
}

bool OmnixNode::DisableJoystickCallback(omnix::DisableJoystick::Request &req, omnix::DisableJoystick::Response &res)
{
  ROS_INFO("OmnixNode: Disabling joystick control, moving to UDP control.");
  bool result = sendCommand(DisableJoystickCommand);
  if(result){
      ROS_INFO("OmnixNode: Successfully disabled joystick control.");
      joystick_controlled = false;
  } else {
      ROS_ERROR("OmnixNode: Error sending DJS command!");
  }
  return true;
}

//TODO: Implement this.
bool OmnixNode::EnableJoystickCallback(omnix::EnableJoystick::Request &req, omnix::EnableJoystickResponse &res)
{
  ROS_INFO("OmnixNode: EJS is NYI.");
  return false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "omnix");
  OmnixNode omnix;
  ros::spin();
  return 0;
}

