#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <sstream>

#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>

#include "phantom_omni/PhantomButtonEvent.h"

struct OmniState
{
    hduVector3Dd position; //3x1 vector of position
    hduVector3Dd rot;
	hduVector3Dd joints;
    hduVector3Dd force; //3 element double vector force[0], force[1], force[2]
    float thetas[7];
    int buttons[2];
    int buttons_prev[2];
	//hduMatrix omni_mx;
};

/**
 * Test this.
 */
void construct_transform(tf::Transform *transform, double theta, double alpha, double a, double d)
{
    double c_alpha = cos(alpha);
    double s_alpha = sin(alpha);
    double c_theta = cos(theta);
    double s_theta = sin(theta);

    transform->setOrigin(tf::Vector3(a, -s_alpha*d, c_alpha*d));
    transform->getBasis().setValue(c_theta, -s_theta, 0,
                                   c_alpha*s_theta, c_alpha*c_theta, -s_alpha,
                                   s_alpha*s_theta, s_alpha*c_theta, c_alpha);
    //B = np.matrix([[c(theta),         -s(theta),           0,         a], 
    //               [c(alpha)*s(theta), c(alpha)*c(theta), -s(alpha), -s(alpha)*d], 
    //               [s(alpha)*s(theta), s(alpha)*c(theta),  c(alpha),  c(alpha)*d], 
    //               [0,                 0,                  0,         1]])
}

tf::Transform angles_to_transform(float *thetas)
{
    tf::Transform B_0_6 = btTransform::getIdentity();
    double pi = M_PI;
    double a[]     = {0.,  0.,    131., 0.,     0.,     0.,      0.};
    double alpha[] = {0., -pi/2., 0.,  -pi/2., -pi/2., -pi/2.,   0};
    double d[]     = {0., -137., -17.8, 17.8,  -131.,   0.,      38.7};
    double t_off[] = {0.,  0.,    0.,   0.,     pi,     (3/2.)*pi, pi};

    for (int i = 0; i < 6; i++)
    {
        tf::Transform t;
        construct_transform(&t, t_off[i+1]+thetas[i+1], alpha[i], a[i],  d[i+1]);
        B_0_6 = B_0_6 * t;
    }
    return B_0_6;
}

    //btVector3 v  = B_0_6.getBasis().getRow(0);
    //btVector3 v1 = B_0_6.getBasis().getRow(1);
    //btVector3 v2 = B_0_6.getBasis().getRow(2);
    //B_0_6.setOrigin(tf::Vector3(B_0_6.getOrigin()[0]/ 1000.0,
    //                            B_0_6.getOrigin()[1]/ 1000.0,
    //                            B_0_6.getOrigin()[2]/ 1000.0));
    //printf("========================== Transform ============================\n");
    //printf("[[%f, %f, %f, %f],\n", v[0],  v[1],  v[2], B_0_6.getOrigin()[0]);
    //printf(" [%f, %f, %f, %f],\n", v1[0], v1[1], v1[2], B_0_6.getOrigin()[1]);
    //printf(" [%f, %f, %f, %f],\n", v2[0], v2[1], v2[2], B_0_6.getOrigin()[2]);
    //printf("trans [%f %f %f].T\n", );

class PhantomROS {

	public:
	ros::NodeHandle n;
	ros::Publisher pose_publisher;
	ros::Publisher omni_pose_publisher;

    ros::Publisher button_publisher;
	ros::Subscriber haptic_sub;
    std::string omni_name;
    OmniState *state;
    tf::TransformBroadcaster br;
    std::string link_names[7];

    void init(OmniState *s) 
    {
        pose_publisher = n.advertise<geometry_msgs::PoseStamped>("omni_pose", 100);
        omni_pose_publisher = n.advertise<geometry_msgs::PoseStamped>("omni_pose_internal", 100);
        button_publisher = n.advertise<phantom_omni::PhantomButtonEvent>("phantom_button", 100);
        ros::param::param(std::string("omni_name"), omni_name, std::string("omni1"));
        haptic_sub = n.subscribe("force_feedback", 100, &PhantomROS::force_callback, this);
        state = s;

        for (int i = 0; i < 7; i++)
        {
            std::ostringstream stream1;
            stream1 << omni_name << "_link" << i;
            link_names[i] = std::string(stream1.str());
        }
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

    void publish_omni_state()
    {
        //Construct transforms
        tf::Transform link;
        link.setOrigin(tf::Vector3(0., 0, 0.15));
        link.setRotation(tf::Quaternion(0, 0, 0));
        br.sendTransform(tf::StampedTransform(link, ros::Time::now(), "world", link_names[0].c_str()));

        link.setOrigin(tf::Vector3(0., 0, 0));
        link.setRotation(tf::Quaternion(-M_PI/2, 0, M_PI/2));
        br.sendTransform(tf::StampedTransform(link, ros::Time::now(), "world", "sensable"));

        link.setOrigin(tf::Vector3(0., 0, 0.));
        link.setRotation(tf::Quaternion(-state->thetas[1], 0, 0));
        br.sendTransform(tf::StampedTransform(link, ros::Time::now(), link_names[0].c_str(), link_names[1].c_str()));
                                                                                                          
        link.setOrigin(tf::Vector3(0., 0, 0.));                                                           
        link.setRotation(tf::Quaternion(0, state->thetas[2], 0));                                         
        br.sendTransform(tf::StampedTransform(link, ros::Time::now(), link_names[1].c_str(), link_names[2].c_str()));
                                                                                                          
        link.setOrigin(tf::Vector3(-.131, 0, 0.));                                                        
        link.setRotation(tf::Quaternion(0, state->thetas[3], 0));                            
        br.sendTransform(tf::StampedTransform(link, ros::Time::now(), link_names[2].c_str(), link_names[3].c_str()));
                                                                                                          
        link.setOrigin(tf::Vector3(0., 0, -.137));                                                        
        link.setRotation(tf::Quaternion(state->thetas[4]+M_PI, 0, 0));                       
        br.sendTransform(tf::StampedTransform(link, ros::Time::now(), link_names[3].c_str(), link_names[4].c_str()));
                                                                                                          
        link.setOrigin(tf::Vector3(0., 0., 0.));                                                          
        link.setRotation(tf::Quaternion(0., -state->thetas[5]+M_PI,0));                      
        br.sendTransform(tf::StampedTransform(link, ros::Time::now(), link_names[4].c_str(), link_names[5].c_str()));

        link.setOrigin(tf::Vector3(0., 0., 0.));
        link.setRotation(tf::Quaternion(0.,0, state->thetas[6]+M_PI));
        br.sendTransform(tf::StampedTransform(link, ros::Time::now(), link_names[5].c_str(), link_names[6].c_str()));

        //Sample 'end effector' pose
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = link_names[6].c_str();
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.pose.position.x = .03;
        pose_stamped.pose.position.y = 0.0;
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation.x = 0;
        pose_stamped.pose.orientation.y = 0;
        pose_stamped.pose.orientation.z = 0;
        pose_stamped.pose.orientation.w = 1.;
        pose_publisher.publish(pose_stamped);

        geometry_msgs::PoseStamped omni_internal_pose;
        omni_internal_pose.header.frame_id = "sensable";
        omni_internal_pose.header.stamp = ros::Time::now();
        omni_internal_pose.pose.position.x = state->position[0]/1000.0;
        omni_internal_pose.pose.position.y = state->position[1]/1000.0;
        omni_internal_pose.pose.position.z = state->position[2]/1000.0;
        omni_pose_publisher.publish(omni_internal_pose);

        if ((state->buttons[0] != state->buttons_prev[0]) or (state->buttons[1] != state->buttons_prev[1]))
        {
            phantom_omni::PhantomButtonEvent button_event;
            button_event.grey_button = state->buttons[0];
            button_event.white_button = state->buttons[1];
            state->buttons_prev[0] = state->buttons[0];
            state->buttons_prev[1] = state->buttons[1];
            button_publisher.publish(button_event);
        }
    }
};

HDCallbackCode HDCALLBACK omni_state_callback(void *pUserData)
{
    OmniState *omni_state = static_cast<OmniState *>(pUserData);

	hdBeginFrame(hdGetCurrentDevice());
	//hdGetDoublev(HD_CURRENT_TRANSFORM,     omni_state->omni_mx);  Wrong
    //hdGetDoublev(HD_CURRENT_VELOCITY, omni_state>vel);            Too lagged

    //Get angles, set forces
	hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, omni_state->rot);      
	hdGetDoublev(HD_CURRENT_POSITION,      omni_state->position);
	hdGetDoublev(HD_CURRENT_JOINT_ANGLES,  omni_state->joints);
	hdSetDoublev(HD_CURRENT_FORCE,         omni_state->force);  

    //hduVector3Dd force_read; 
	//hdGetDoublev(HD_CURRENT_FORCE,         force_read);  
    //printf("%.3f %.3f %.3f\n", force_read[0], force_read[1], force_read[2]);

    //Get buttons
    int nButtons = 0;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
    omni_state->buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0;
    omni_state->buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0;

	hdEndFrame(hdGetCurrentDevice());
    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
		hduPrintError(stderr, &error, "Error during main scheduler callback\n");
		if (hduIsSchedulerError(&error))
			return HD_CALLBACK_DONE;
    }

            //pos.joint2 = state.joint[2]-state.joint[1];		//joint[2] of omni is coupled with joint[1] angle
    float t[7] = {0., omni_state->joints[0], omni_state->joints[1], omni_state->joints[2]-omni_state->joints[1], 
                      omni_state->rot[0],    omni_state->rot[1],    omni_state->rot[2]};
    for (int i = 0; i < 7; i++)
        omni_state->thetas[i] = t[i];
    //printf("%f %f %f\n", omni_state->rot[0], omni_state->rot[1], omni_state->rot[2]);

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
   state.buttons[0] = 0;
   state.buttons[1] = 0;
   state.buttons_prev[0] = 0;
   state.buttons_prev[1] = 0;
   omni_ros.init(&state);

   int publish_rate;
   omni_ros.n.param(std::string("publish_rate"), publish_rate, 50);
   ros::Rate loop_rate(publish_rate);

   hdScheduleAsynchronous(omni_state_callback, &state, HD_DEFAULT_SCHEDULER_PRIORITY);

   ////////////////////////////////////////////////////////////////
   // Loop and publish 
   ////////////////////////////////////////////////////////////////
   while(ros::ok())
   {
       omni_ros.publish_omni_state();
       ros::spinOnce();
       loop_rate.sleep();
   }

   ROS_INFO("Ending Session....\n");
   hdStopScheduler();
   hdDisableDevice(hHD);

   return 0;
}


       








































        //std::string l1_str("_link1")
        //std::string l1_str("_link2")
        //std::string l1_str("_link3")
        //std::string l1_str("_link4")
        //std::string l1_str("_link5")
        //std::string l1_str("_link6")


        //tf::Transform transform = angles_to_transform(state->thetas);
        //tf::Transform Ry90;
        ////Ry90.setOrigin(
        //Ry90.setOrigin(tf::Vector3(0., 0, 0.));
        //Ry90.getBasis().setEulerYPR(0, M_PI/2.0, 0);
        //transform.setBasis(Ry90.getBasis() * transform.getBasis());
        ////br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", frame_id.c_str()));
        //btScalar y, p, r;
        //transform.getBasis().getEulerYPR(y, p, r);
        //transform.getBasis().setEulerYPR(-r, p, r);
        ////printf("%f\n", transform.getRotation()[2]);
        ////pose_stamped.pose.setEuler(M_PI, 0, 0);
        //pose_stamped.pose.orientation.x = transform.getRotation()[0];
        //pose_stamped.pose.orientation.y = transform.getRotation()[1];
        //pose_stamped.pose.orientation.z = transform.getRotation()[2];
        //pose_stamped.pose.orientation.w = transform.getRotation()[3];
        //pose_publisher.publish(pose_stamped);

        //pose_stamped.pose.position.x = state->position[0] / 1000.;
        //pose_stamped.pose.position.y = state->position[1] / 1000.;
        //pose_stamped.pose.position.z = state->position[2] / 1000.;
        //printf("=================== Transform ===================\n");
        //for (int row = 0; row < 4; row++)
        //{
        //    for (int col = 0; col < 4; col++)
        //    {
        //        printf("%.3f, ", state->omni_mx[col][row]);
        //    }
        //    printf("\n");
        //}
    
        //tf::Transform transform;
        //transform.setOrigin(tf::Vector3(state->omni_mx[3][0], state->omni_mx[3][1], state->omni_mx[3][2]));
        //transform.getBasis().setValue(  state->omni_mx[0][0], state->omni_mx[1][0], state->omni_mx[2][0],
        //                                state->omni_mx[0][1], state->omni_mx[1][1], state->omni_mx[2][1], 
        //                                state->omni_mx[0][2], state->omni_mx[1][2], state->omni_mx[2][2]);




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
