#include <numeric>
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "pcl/point_types.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/features/principal_curvatures.h>

#include <tf/transform_listener.h>
#include "pcl_ros/transforms.h"
#include <boost/make_shared.hpp>
#include <boost/thread/mutex.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseArray.h>
#include <hrl_table_detect/DetectTableStart.h>
#include <hrl_table_detect/DetectTableStop.h>
#include <hrl_table_detect/DetectTableInst.h>
#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>
#include <hrl_cvblobslib/Blob.h>
#include <hrl_cvblobslib/BlobResult.h>
#include <image_geometry/pinhole_camera_model.h>

using namespace sensor_msgs;
using namespace std;
namespace enc = sensor_msgs::image_encodings;
namespace hrl_table_detect {
    
    typedef pcl::PointXYZRGB PRGB;
    typedef pcl::PointXYZRGBNormal PRGBN;

    class SurfaceSegmentation {
        public:
            ros::Subscriber pc_sub, cam_sub;
            ros::NodeHandle nh;
            ros::Publisher pc_pub, pc_pub2, pc_pub3;
            tf::TransformListener tf_listener;
            image_geometry::PinholeCameraModel cam_model;

            SurfaceSegmentation();
            void onInit();
            void pcCallback(sensor_msgs::PointCloud2::ConstPtr pc_msg);
            //void modelCallback(image_geometry::PinholeCameraModel::ConstPtr pin_msg);
    };

    SurfaceSegmentation::SurfaceSegmentation() {
    }

    void SurfaceSegmentation::onInit() {
        pc_pub = nh.advertise<pcl::PointCloud<PRGB> >("/normal_vis", 1);
        pc_pub2 = nh.advertise<pcl::PointCloud<PRGB> >("/normal_vis2", 1);
        pc_pub3 = nh.advertise<pcl::PointCloud<PRGB> >("/normal_vis3", 1);
        pc_sub = nh.subscribe("/kinect_head/rgb/points", 1, &SurfaceSegmentation::pcCallback, this);
        //cam_sub = nh.subscribe("/kinect_head/rgb/camera_info", 1, &SurfaceSegmentation::modelCallback, this);
        ros::Duration(1.0).sleep();
    }

    void SurfaceSegmentation::pcCallback(sensor_msgs::PointCloud2::ConstPtr pc_msg) {
        double min_z_val = 0.1, max_z_val = 1.5;
        double norm_ang_thresh = 0.5;

        pcl::PointCloud<PRGB> pc_full, pc_full_frame;
        pcl::fromROSMsg(*pc_msg, pc_full);
        string base_frame("/base_link");
        ros::Time now = ros::Time::now();

        // Transform PC to base frame
        tf_listener.waitForTransform(pc_msg->header.frame_id, base_frame, now, ros::Duration(3.0));
        pcl_ros::transformPointCloud(base_frame, pc_full, pc_full_frame, tf_listener);
        pcl::PointCloud<PRGB>::ConstPtr pc_full_frame_ptr = boost::make_shared<pcl::PointCloud<PRGB> >(pc_full_frame);

        // Filter floor and ceiling
        pcl::PointCloud<PRGB> pc_filtered;
        pcl::PointCloud<PRGB>::ConstPtr pc_filtered_ptr = 
                  boost::make_shared<const pcl::PointCloud<PRGB> >(pc_filtered);
        pcl::PassThrough<PRGB> z_filt;
        z_filt.setFilterFieldName("z");
        z_filt.setFilterLimits(min_z_val);
        z_filt.setFilterLimits(max_z_val);
        z_filt.setInputCloud(pc_full_frame_ptr);
        z_filt.applyFilter(pc_filtered);

        // Compute normals
        pcl::PointCloud<PRGBN> cloud_normals;
        //pcl::PointCloud<PRGB>::ConstPtr pc_filtered_ptr = 
        //          boost::make_shared<const pcl::PointCloud<PRGB> >(cloud_normals_ptr);
        pcl::KdTree<PRGB>::Ptr normals_tree = boost::make_shared<pcl::KdTreeFLANN<PRGB> > ();
        pcl::NormalEstimation<PRGB, PRGBN> norm_est;
        norm_est.setKSearch(15);
        norm_est.setSearchMethod(normals_tree);
        norm_est.setInputCloud(pc_filtered_ptr);
        norm_est.compute(cloud_normals);
        
        BOOST_FOREACH(const PRGBN& pt, cloud_normals.points) {
            double ang = fabs((acos(pt.normal[2]) - CV_PI/2)/CV_PI*2);

            if(ang > norm_ang_thresh)
                ((uint32_t*) &cur_pt.rgb)[0] = ((uint32_t) ((ang-thresh)/(CV_PI/2)*256))<<16 | 0xFF000000;
            else
                ((uint32_t*) &cur_pt.rgb)[0] = ((uint32_t) ((thresh-ang)/thresh*256)) | 0xFF000000;
            
            cloud_normals_filtered.push_back(cur_pt);

            PRGB p_cur_pt = pc_full_frame.at(i-1, 0);
            pcl::PrincipalCurvatures pcurves = princip_curves.at(i-1, 0);
            //cout << pcurves << " ";
            if(pcurves.pc1 < 0.8)
                ((uint32_t*) &p_cur_pt.rgb)[0] = 0xFF0000FF;
            else if(pcurves.pc2 > 0.0001)
                ((uint32_t*) &p_cur_pt.rgb)[0] = ((uint32_t) (min(fabs((double)pcurves.pc1/pcurves.pc2), 30.0)/30.0*256))<<16 | 0xFF000000;
            else
                ((uint32_t*) &p_cur_pt.rgb)[0] = (256)<<16 | 0xFF00FF00;
            princip_curves_colors.push_back(p_cur_pt);
        }
        cloud_normals_colors.header.stamp = ros::Time::now();
        cloud_normals_colors.header.frame_id = base_frame;
        pc_pub.publish(cloud_normals_colors);

        princip_curves_colors.header.stamp = ros::Time::now();
        princip_curves_colors.header.frame_id = base_frame;
        pc_pub2.publish(princip_curves_colors);

        // define a plane to project onto
        geometry_msgs::PoseStamped plane_pose;
        plane_pose.pose.position.x=0; plane_pose.pose.position.y=0; plane_pose.pose.position.z=0.542; 
        plane_pose.pose.orientation.x=0; plane_pose.pose.orientation.y=0; 
        plane_pose.pose.orientation.z=0; plane_pose.pose.orientation.w=1; 
        plane_pose.header.stamp = now; plane_pose.header.frame_id = base_frame;
        tf_listener.transformPose(pc_msg->header.frame_id, plane_pose, plane_pose);
        btQuaternion plane_quat(plane_pose.pose.orientation.x,
                                plane_pose.pose.orientation.y,
                                plane_pose.pose.orientation.z,
                                plane_pose.pose.orientation.w);
        btMatrix3x3 plane_rot(plane_quat);
        btVector3 plane_norm = plane_rot.getColumn(2);
        btVector3 plane_pt(plane_pose.pose.position.x,plane_pose.pose.position.y,
                           plane_pose.pose.position.z);
        i=0;
        pcl::PointCloud<PRGB> flat_pc;
        BOOST_FOREACH(const PRGB& pt, pc_full.points) {
            i++;
            if(pt.x != pt.x || pt.y != pt.y || pt.z != pt.z)
                continue;
            btVector3 pc_pt(pt.x,pt.y,pt.z);
            double t = plane_norm.dot(plane_pt)/plane_norm.dot(pc_pt);
            btVector3 surf_pt = pc_pt*t;
            PRGB n_pt;
            n_pt.x = surf_pt.x(); n_pt.y = surf_pt.y(); n_pt.z = surf_pt.z(); 
            n_pt.rgb = pt.rgb;
            if(t > 1.01) {
                uint32_t red = 0xFFFF0000;
                ((uint32_t*) &n_pt.rgb)[0] = red;
            } else if (t < 0.99) {
                uint32_t blue = 0xFF0000FF;
                ((uint32_t*) &n_pt.rgb)[0] = blue;
            } else {
                uint32_t green = 0xFF00FF00;
                ((uint32_t*) &n_pt.rgb)[0] = green;
            }
            flat_pc.push_back(n_pt);
        }
        flat_pc.header.stamp = ros::Time::now();
        flat_pc.header.frame_id = pc_full.header.frame_id;
        pc_pub3.publish(flat_pc);

    }

    //void SurfaceSegmentation::modelCallback(image_geometry::PinholeCameraModel::ConstPtr pin_msg) {
    //    cam_model.fromCameraInfo(pin_msg);
    //}
};

using namespace hrl_table_detect;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_normals");
    SurfaceSegmentation ta;
    ta.onInit();
    ros::spin();
    return 0;
}



