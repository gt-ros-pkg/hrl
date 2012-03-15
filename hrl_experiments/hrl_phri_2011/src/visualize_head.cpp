#include <hrl_phri_2011/ellipsoid_space.h>
#include <ros/ros.h>
//#include <hrl_phri_2011/pc_utils.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>
#include <Eigen/Eigen>

typedef pcl::PointXYZRGB PRGB;
typedef pcl::PointCloud<PRGB> PCRGB;

using namespace std;

using namespace Eigen;
namespace enc = sensor_msgs::image_encodings;

void pubLoop(PCRGB &pc, const std::string& topic) {
    ros::NodeHandle nh;
    ros::Publisher pub_pc = nh.advertise<sensor_msgs::PointCloud2>(topic, 1);
    ros::Rate r(1);
    sensor_msgs::PointCloud2 pc_msg;
    pcl::toROSMsg(pc, pc_msg);
    while(ros::ok()) {
        pc_msg.header.stamp = ros::Time::now();
        pub_pc.publish(pc_msg);
        r.sleep();
    }
}

void transformPC(const PCRGB &in_pc, PCRGB &out_pc, 
                 const Eigen::Affine3d& transform) {
    MatrixXd pt_mat = MatrixXd::Constant(4, in_pc.points.size(), 1.0);
    uint32_t i = 0;
    BOOST_FOREACH(PRGB const pt, in_pc.points) {
        pt_mat(0, i) = pt.x; pt_mat(1, i) = pt.y; pt_mat(2, i) = pt.z; 
        i++;
    }
    MatrixXd trans_pt_mat = transform.matrix() * pt_mat;
    for(i=0;i<in_pc.points.size();i++) {
        PRGB pt;
        pt.x = trans_pt_mat(0, i); pt.y = trans_pt_mat(1, i); pt.z = trans_pt_mat(2, i); 
        pt.rgb = in_pc.points[i].rgb;
        out_pc.points.push_back(pt);
    }
}


void imgPubLoop(cv::Mat& img_mat) {
    ros::NodeHandle nh;
    image_transport::ImageTransport img_trans(nh);
    image_transport::Publisher img_pub = img_trans.advertise("/camera/img_pub", 1);
    cv_bridge::CvImage cvb_img;
    cvb_img.image = img_mat;
    cvb_img.header.frame_id = "/base_link";
    cvb_img.encoding = enc::RGB8;
    ros::Rate r(1);
    while(ros::ok()) {
        cvb_img.header.stamp = ros::Time::now();
        img_pub.publish(cvb_img.toImageMsg());
        ros::spinOnce();
        r.sleep();
    }
}

void visualize(PCRGB::Ptr& pc_head) {
    Ellipsoid e(0.05, 0.061);
    double lat, lon, height, mapx, mapy;
    int i=0;
    cv::Mat img(1000, 1000, CV_8UC3);//, cv::Vec3b(0xff, 0xff, 0xff));
    double minmapx = 1000, maxmapx = -1000, minmapy = 1000, maxmapy = -1000;
    double size = 200, offx = 0, offy = 0;
    BOOST_FOREACH(PRGB const pt, pc_head->points) {
        e.cartToEllipsoidal(pt.x, pt.y, pt.z, lat, lon, height);
        e.mollweideProjection(lat, lon, mapx, mapy);
        minmapx = min(lon, minmapx);
        maxmapx = max(lon, maxmapx);
        minmapy = min(lat, minmapy);
        maxmapy = max(lat, maxmapy);
        uint8_t r = ((uint8_t*) &pt.rgb)[2];
        uint8_t g = ((uint8_t*) &pt.rgb)[1];
        uint8_t b = ((uint8_t*) &pt.rgb)[0];
        int pixx = mapx*3400 + 500;
        int pixy = mapy*3400 + 500;
        //int pixx = mapx*4000 + 1000;
        //int pixy = mapy*3000 + 700;
        if(i++ % 100 == 0) {
            //printf("(%f, %f, %f) ", mapx, mapy, height);
            //printf("(%f, %f, %f) ", lat, lon, height);
            printf("[pt (%f, %f, %f) ;", pt.x, pt.y, pt.z);
            printf("ell (%f, %f, %f) ;", lat, lon, height);
            printf("pix (%d, %d)] ", pixx, pixy);
        }
        if(pixx < 0 || pixx >= 1000 || pixy < 0 || pixy >= 1000)
            continue;
        img.at<cv::Vec3b>(pixx, pixy)[0] = b;
        img.at<cv::Vec3b>(pixx, pixy)[1] = g;
        img.at<cv::Vec3b>(pixx, pixy)[2] = r;
    }
    printf("\nMins, max %f %f %f %f\n", minmapx, minmapy, maxmapx, maxmapy);
    IplImage img_ipl = img;
    cvSaveImage("test.png", &img_ipl);
    imgPubLoop(img);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_head");

    // Load bag
    rosbag::Bag bag;
    bag.open(std::string(argv[1]), rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery("/stitched_head"));

    PCRGB::Ptr pc_head(new PCRGB());
    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
        sensor_msgs::PointCloud2::Ptr pc2 = m.instantiate<sensor_msgs::PointCloud2>();
        pcl::fromROSMsg(*pc2, *pc_head);
        break;
    }
    PCRGB::Ptr pc_head_tf(new PCRGB());
    Eigen::Affine3d aff_tf(Eigen::Quaternion<double>(0.00163   , 0.25178997, 0.18013405,  0.95086849));
    aff_tf = (Translation3d(0.03, 0.0, 0.03)) * aff_tf;
    aff_tf = Quaterniond(0, 0, -1.,  0) * aff_tf;
    printf("\n\n %f %f %f %f",aff_tf(0,0), aff_tf(0,1), aff_tf(0,2),  aff_tf(0,3));
    transformPC(*pc_head, *pc_head_tf, aff_tf);
    pc_head_tf->header.frame_id = "/base_link";
    pubLoop(*pc_head_tf, "/tf_head");
    visualize(pc_head_tf);
    return 0;
}
