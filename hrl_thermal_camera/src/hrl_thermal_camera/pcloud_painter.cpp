#include "pcloud_painter.h"
#include <sensor_msgs/CameraInfo.h>
#include <pcl/point_types.h>
#include <math.h>
#include <algorithm>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv/cv.h>

PCloud_Painter::PCloud_Painter(const ros::NodeHandle& nh_, const tf::TransformListener& tf_listener)
  : nh(nh_), sync(MySyncPolicy(10), thermal_image_sub, rgb_pcloud_sub)//, it_(nh_)
{
	// Set verbosity of pcl.
//	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

	// In initialization I pull the transform from the left rgb camera (the frame for the rgb cloud) to the left thermal camera.
//	while (!tf_listener.canTransform("/thermal_camera","/head_mount_kinect_rgb_optical_frame",ros::Time(0)))
//	    ROS_INFO_THROTTLE(1.0, "Waiting for transforms");
//	tf_listener.lookupTransform("thermal_camera","head_mount_kinect_rgb_optical_frame",ros::Time(0),rgb_to_thermal_tf);
	while (!tf_listener.canTransform("/thermal_camera_frame","/camera_rgb_optical_frame",ros::Time(0)))
	    ROS_INFO_THROTTLE(1.0, "Waiting for transforms");
	tf_listener.lookupTransform("thermal_camera_frame","camera_rgb_optical_frame",ros::Time(0),rgb_to_thermal_tf);


	// In initialization I set the frame_id of the output message to be the multisense/thermal/left_camera_optical_frame
//	pointcloud_msg_header.frame_id = "head_mount_kinect_rgb_optical_frame";
	pointcloud_msg_header.frame_id = "camera_rgb_optical_frame";

	// setup publishers/subscribers and the synchronizer.
	painted_pcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("painted_pcloud", 1);
	thermal_camera_info_sub = nh.subscribe("/thermal_camera/thermal_camera_driver/camera_info", 1, &PCloud_Painter::thermal_camera_info_cb, this);


	currently_painting = false;
    has_camera=false;

	ROS_INFO("The Point Cloud Painter has been initialized. Subscribe to <node_name>/painted_cloud (default node name is pcloud_painter) to activate the painter and get the painted cloud!");
}

// This function is a callback for the thermal camera's camera info. Typically that topic is not published frequently
// and is just latched, so this usually will only run once at start-up of the painter node.
void
PCloud_Painter::thermal_camera_info_cb (const sensor_msgs::CameraInfoConstPtr& info_msg)
{
	tcam_info = info_msg;
	cam_model_.fromCameraInfo(info_msg);
    has_camera=true;
    thermal_camera_info_sub.shutdown();
}

// This function is the callback of the pointcloud and thermal image.
// It using a synchronizer to ensure that the rgb pointcloud and thermal image came in at close to the same time.
// Receives as input the rectified thermal image from the left camera and the rgb pointcloud (without color).
// Basically everything of interest in this node occurs in the callback (the painting of the cloud).
void
PCloud_Painter::painting_callback (const sensor_msgs::ImageConstPtr& thermal_image_msg,
						   	   	   const sensor_msgs::PointCloud2ConstPtr& rgb_pcloud_msg)
{
  try
  {
    cv_ptr = cv_bridge::toCvCopy(thermal_image_msg, sensor_msgs::image_encodings::MONO16);
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
//    std::cout << "hi";
//    ROS_INFO_STREAM("test_hi");
//    std::cout << rgb_to_thermal_tf;
    converted_cloud=*rgb_pcloud_msg;

	// Convert the ros pointcloud2 into a pcl cloud that I transform to the thermal camera frame
	pcl::fromROSMsg (*rgb_pcloud_msg, rgbcloud);
	pcl_ros::transformPointCloud(rgbcloud,rgb_transformed_cloud,rgb_to_thermal_tf);

	// For each point in the transformed cloud, find the associated pixel in the thermal image. I then add that point to the converted_cloud.
    union
    {
      uint32_t value;
      char bytes[sizeof(uint32_t)];
    } color;

	for (uint i=0; i< rgb_transformed_cloud.points.size(); ++i)
	{
		// Find the uv coordinates in the thermal image associated with the 3d point.
		// In uv coordinates, uv.x is to the right, uv.y is down. 0,0 is top left.
		cv::Point3d pt_cv(rgb_transformed_cloud.points[i].x, rgb_transformed_cloud.points[i].y, rgb_transformed_cloud.points[i].z);
		uv = cam_model_.project3dToPixel(pt_cv);

    this_temp=0;

		// Check that the point is within the image.
    int clip_value=2;
		if (int(uv.x)<int(thermal_image_msg->width)-clip_value && int(uv.x)>=clip_value && int(uv.y)<int(thermal_image_msg->height)-clip_value && int(uv.y)>=clip_value) {
          // Find the thermal value of that pixel in the thermal image.
          this_temp = cv_ptr->image.at<uint16_t>(uv.y, uv.x);
//          ind = 2*(int(uv.y)*thermal_image_msg->width + int(uv.x));
//          color.bytes[0]=thermal_image_msg->data[ind];
//          color.bytes[1]=thermal_image_msg->data[ind+1];
    }
//    std::cout<<"temp: "<<this_temp<<"  ";
		// Save the converted point with intensity.
        float *fltPtr= reinterpret_cast<float*>(&(converted_cloud.data[i*converted_cloud.point_step+16]));
        fltPtr[0] = static_cast<float>(this_temp);
	}
  converted_cloud.fields[3].name = "intensity";
	// Publish the converted cloud!
    painted_pcloud_pub.publish(converted_cloud);
}

bool
PCloud_Painter::spin()
{
	ros::Rate loop(30);
	while (nh.ok())
	{
      if (has_camera)
        thermal_camera_info_sub.shutdown();

		if (painted_pcloud_pub.getNumSubscribers() > 0)
		{
			if (!currently_painting)
			{
				currently_painting = true;
				ROS_INFO_STREAM("Connecting to rgb pointcloud and thermal image. Starting to publish painted cloud.");
                thermal_image_sub.subscribe(nh,"/thermal_camera/thermal_camera_driver/image_rect", 1);
//                rgb_pcloud_sub.subscribe(nh,"/head_mount_kinect/qhd/points", 1);
                rgb_pcloud_sub.subscribe(nh,"/camera/depth_registered/points", 1);

				callback_control = sync.registerCallback(&PCloud_Painter::painting_callback, this);
			}
		}
		else
		{
			if(currently_painting)
			{
				currently_painting = false;
                thermal_image_sub.unsubscribe();
                rgb_pcloud_sub.unsubscribe();
				ROS_INFO_STREAM("Disconnecting from the rgb pointcloud and thermal image. No longer publishing painted cloud.");
				callback_control.disconnect();
			}
		}


		ros::spinOnce();
		loop.sleep();
	}
	return true;
}


