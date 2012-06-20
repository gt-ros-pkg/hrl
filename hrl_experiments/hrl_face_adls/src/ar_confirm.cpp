#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <geometry_msgs/PoseStamped.h>

#include <ar_pose/ARMarker.h>

typedef pcl::PointXYZRGB PRGB;
typedef pcl::PointCloud<PRGB> PCRGB;

ar_pose::ARMarker::Ptr ar_tag_l, ar_tag_r;
ros::Subscriber ar_tag_l_sub, ar_tag_r_sub;
double last_l_time, last_r_time;

image_transport::CameraSubscriber camera_sub;
image_transport::Publisher overlay_pub;
image_geometry::PinholeCameraModel cam_model;
boost::shared_ptr<tf::TransformListener> tf_list;
cv_bridge::CvImagePtr cv_img;

void subARTagLCallback(const ar_pose::ARMarker::Ptr& ar_tag_)
{
    ar_tag_l = ar_tag_;
    last_l_time = ros::Time::now().toSec();
}

void subARTagRCallback(const ar_pose::ARMarker::Ptr& ar_tag_)
{
    ar_tag_r = ar_tag_;
    last_r_time = ros::Time::now().toSec();
}

void writeTag(const ar_pose::ARMarker::Ptr& ar_tag, const sensor_msgs::ImageConstPtr& img_msg,
              const cv::Scalar& color)
{
    if(!tf_list->waitForTransform(img_msg->header.frame_id, ar_tag->header.frame_id,
                                 img_msg->header.stamp, ros::Duration(3)))
        return;
    geometry_msgs::PoseStamped ar_tag_pose, ar_tag_tf;
    ar_tag_pose.header = ar_tag->header;
    ar_tag_pose.pose = ar_tag->pose.pose;
    tf_list->transformPose(img_msg->header.frame_id, ar_tag_pose, ar_tag_tf);

    double marker_width;
    ros::param::param<double>("~marker_width", marker_width, 0.135);
    const cv::Point** cv_poly_list = new const cv::Point*[4];
    std::vector<cv::Point> cv_pts;
    tf::Pose tf_pose;
    tf::poseMsgToTF(ar_tag_tf.pose, tf_pose);
    for(int i=0;i<2;i++) {
        for(int j=0;j<2;j++) {
            btVector4 tag_pt;
            tag_pt.setX(i * marker_width - marker_width/2.);
            tag_pt.setY(j * marker_width - marker_width/2.);
            tag_pt.setZ(0);
            tag_pt.setW(1);
            btVector3 tag_pt_rot = tf_pose * tag_pt;
            cv::Point3d proj_pt_cv(tag_pt_rot.getX(), tag_pt_rot.getY(), tag_pt_rot.getZ());
            cv_pts.push_back(cam_model.project3dToPixel(proj_pt_cv));
        }
    }
    cv::Point* tag_pts = new cv::Point[4];
    tag_pts[0] = cv_pts[0]; tag_pts[1] = cv_pts[1]; 
    tag_pts[2] = cv_pts[3]; tag_pts[3] = cv_pts[2]; 
    cv_poly_list[0] = tag_pts;
    int npts[1] = {4};
    cv::polylines(cv_img->image, cv_poly_list, npts, 1, 1, color, 4);
}

void doOverlay(const sensor_msgs::ImageConstPtr& img_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg) {

    // convert camera image into opencv
    cam_model.fromCameraInfo(info_msg);
    cv_img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);

    double timeout_time;
    ros::param::param<double>("~timeout_time", timeout_time, 1.0);

    double cur_time = ros::Time::now().toSec();
    if(ar_tag_l && cur_time - last_l_time < timeout_time) {
        cv::Scalar color = CV_RGB(255, 0, 0);
        writeTag(ar_tag_l, img_msg, color);
    }
    if(ar_tag_r && cur_time - last_r_time < timeout_time) {
        cv::Scalar color = CV_RGB(0, 255, 0);
        writeTag(ar_tag_r, img_msg, color);
    }
    
    overlay_pub.publish(cv_img->toImageMsg());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "head_tracking");
    ros::NodeHandle nh;
    image_transport::ImageTransport img_trans(nh);
    tf_list = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener());

    ar_tag_l_sub = nh.subscribe("/ar_tag_l", 1, &subARTagLCallback);
    ar_tag_r_sub = nh.subscribe("/ar_tag_r", 1, &subARTagRCallback);
    camera_sub = img_trans.subscribeCamera("/camera", 1, 
                                           &doOverlay);
    overlay_pub = img_trans.advertise("/confirmation", 1);

    ros::Rate r(5);
    while(ros::ok()) {
        cv::Mat image(480, 640, CV_8UC3);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
