#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Wrench.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>

#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>

struct OmniState
{
    hduVector3Dd position; //3x1 vector of position
    hduVector3Dd rot;
	hduVector3Dd joint;
	hduMatrix omni_mx;
    hduVector3Dd force; //3 element double vector force[0], force[1], force[2]
};

class PhantomROS {

	public:
	ros::NodeHandle n;
	ros::Publisher pose_publisher;
	ros::Subscriber haptic_sub;
    std::string frame_id;
    OmniState *state;

    void init(OmniState *s) 
    {
        pose_publisher = n.advertise<geometry_msgs::PoseStamped>("omni_pose", 100);
        ros::param::param(std::string("publish_frame"), frame_id, std::string("/omni_frame"));
        haptic_sub = n.subscribe("force_feedback", 100, &PhantomROS::force_callback, this);
        state = s;
    }

    /*******************************************************************************
     ROS node callback.  
    *******************************************************************************/
    void force_callback(const geometry_msgs::WrenchConstPtr& wrench)
    {
        state->force[0] = wrench->force.x;
        state->force[1] = wrench->force.y;
        state->force[2] = wrench->force.z;
    }

    void publish_pose_stamped()
    {
        geometry_msgs::PoseStamped pose_stamped;
        //btScalar *glmat = new btScalar[16];

        pose_stamped.header.frame_id = frame_id;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.pose.position.x = state->position[0] / 1000.;
        pose_stamped.pose.position.y = state->position[1] / 1000.;
        pose_stamped.pose.position.z = state->position[2] / 1000.;

        printf("=================== Transform ===================\n");
        for (int row = 0; row < 4; row++)
        {
            for (int col = 0; col < 4; col++)
            {
                printf("%.3f, ", state->omni_mx[col][row]);
            }
            printf("\n");
        }
    
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(state->omni_mx[3][0], state->omni_mx[3][1], state->omni_mx[3][2]));
        transform.getBasis().setValue(  state->omni_mx[0][0], state->omni_mx[1][0], state->omni_mx[2][0],
                                        state->omni_mx[0][1], state->omni_mx[1][1], state->omni_mx[2][1], 
                                        state->omni_mx[0][2], state->omni_mx[1][2], state->omni_mx[2][2]);
        pose_stamped.pose.orientation.x = transform.getRotation()[0];
        pose_stamped.pose.orientation.y = transform.getRotation()[1];
        pose_stamped.pose.orientation.z = transform.getRotation()[2];
        pose_stamped.pose.orientation.w = transform.getRotation()[3];
        pose_publisher.publish(pose_stamped);
    }
};

HDCallbackCode HDCALLBACK omni_state_callback(void *pUserData)
{
    OmniState *omni_state = static_cast<OmniState *>(pUserData);

	hdBeginFrame(hdGetCurrentDevice());
    //hdGetDoublev(HD_CURRENT_VELOCITY, omni_state>vel);
	hdGetDoublev(HD_CURRENT_POSITION,      omni_state->position);
	hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, omni_state->rot);
	hdGetDoublev(HD_CURRENT_JOINT_ANGLES,  omni_state->joint);
	hdGetDoublev(HD_CURRENT_TRANSFORM,     omni_state->omni_mx);  

	hdSetDoublev(HD_CURRENT_FORCE,         omni_state->force);  
	hdEndFrame(hdGetCurrentDevice());

    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
		hduPrintError(stderr, &error, "Error during main scheduler callback\n");
		if (hduIsSchedulerError(&error))
		{
			return HD_CALLBACK_DONE;
		}        
    }
    return HD_CALLBACK_CONTINUE;
}

/*******************************************************************************
Automatic Calibration of Phantom Device - No character inputs
*******************************************************************************/
void HHD_Auto_Calibration()
{
   int calibrationStyle;
   int supportedCalibrationStyles;
   HDErrorInfo error;

   hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
   if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET)
   {
		calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
		ROS_INFO("HD_CALIBRATION_ENCODER_RESE..\n\n");
   }
   if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL)
   {
		calibrationStyle = HD_CALIBRATION_INKWELL;
		ROS_INFO("HD_CALIBRATION_INKWELL..\n\n");
   }
   if (supportedCalibrationStyles & HD_CALIBRATION_AUTO)
   {
       calibrationStyle = HD_CALIBRATION_AUTO;
       ROS_INFO("HD_CALIBRATION_AUTO..\n\n");
   }

   do 
   {
	   hdUpdateCalibration(calibrationStyle);
	   ROS_INFO("Calibrating.. (put stylus in well)\n");
       if (HD_DEVICE_ERROR(error = hdGetError()))
       {
	       hduPrintError(stderr, &error, "Reset encoders reset failed.");
	       break;           
           }
   }   while (hdCheckCalibration() != HD_CALIBRATION_OK);

   ROS_INFO("\n\nCalibration complete.\n");
}

int main(int argc, char** argv)
{
   ////////////////////////////////////////////////////////////////
   // Init Phantom
   ////////////////////////////////////////////////////////////////
   HDErrorInfo error;

   HHD hHD;
   hHD = hdInitDevice(HD_DEFAULT_DEVICE);
   if (HD_DEVICE_ERROR(error = hdGetError())) 
   {
       //hduPrintError(stderr, &error, "Failed to initialize haptic device");
       ROS_ERROR("Failed to initialize haptic device"); //: %s", &error);
       return -1;
   }

   ROS_INFO("Found %s.\n\n", hdGetString(HD_DEVICE_MODEL_TYPE));
   hdEnable(HD_FORCE_OUTPUT);
   hdStartScheduler(); 
   if (HD_DEVICE_ERROR(error = hdGetError()))
   {
       //hduPrintError(stderr, &error, "Failed to start the scheduler");
       ROS_ERROR("Failed to start the scheduler");//, &error);
       return -1;           
   }
   HHD_Auto_Calibration();

   ////////////////////////////////////////////////////////////////
   // Init ROS 
   ////////////////////////////////////////////////////////////////
   ros::init(argc, argv, "omni_haptic_node");
   OmniState state;
   PhantomROS omni_ros;
   omni_ros.init(&state);

   int publish_rate;
   omni_ros.n.param(std::string("publish_rate"), publish_rate, 50);
   ros::Rate loop_rate(publish_rate);

   //hdScheduleAsynchronous(omni_state_callback, &state, HD_DEFAULT_SCHEDULER_PRIORITY);
   hdScheduleAsynchronous(omni_state_callback, &state, HD_DEFAULT_SCHEDULER_PRIORITY);

   ////////////////////////////////////////////////////////////////
   // Loop and publish 
   ////////////////////////////////////////////////////////////////
   tf::TransformBroadcaster br;
   tf::Transform transform;
   transform.setOrigin(tf::Vector3(0., 0, 1.));
   transform.setRotation(tf::Quaternion(0, 0, 1.57));

   while(ros::ok())
   {
       omni_ros.publish_pose_stamped();
       br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", omni_ros.frame_id.c_str()));
       ros::spinOnce();
       loop_rate.sleep();
   }

   ROS_INFO("Ending Session....\n");
   hdStopScheduler();
   hdDisableDevice(hHD);

   return 0;
}







//glmat[col*4 + row] = state.omni_mx[col][row];
//state.omni_mx[0][0] state.omni_mx[1][0] state.omni_mx[2][0] state.omni_mx[3][0]
//state.omni_mx[0][1] state.omni_mx[1][1] state.omni_mx[2][1] state.omni_mx[3][1]
//state.omni_mx[0][2] state.omni_mx[1][2] state.omni_mx[2][2] state.omni_mx[3][2]
//state.omni_mx[0][3] state.omni_mx[1][3] state.omni_mx[2][3] state.omni_mx[3][3]

//printf("x %.3f y %.3f z %.3f\n",
//        transform.getOrigin()[0],
//        transform.getOrigin()[1],
//        transform.getOrigin()[2]);
//printf("%.3f %.3f %.3f\n",
//        transform.getBasis().getRow(0)[0],
//        transform.getBasis().getRow(0)[1],
//        transform.getBasis().getRow(0)[2]
//        );
//pose_stamped.pose.orientation.x = ;
//pose_stamped.pose.orientation.y = ;
//pose_stamped.pose.orientation.z = ;
//pose_stamped.pose.orientation.w = ;
//delete glmat;
