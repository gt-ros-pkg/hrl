#include <hrl_phri_2011/ellipsoid_space.h>
#include <ros/ros.h>
#include <boost/random/uniform_real.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <hrl_phri_2011/EllipsoidParams.h>
#include <hrl_phri_2011/utils.h>

typedef pcl::PointXYZRGB PRGB;
typedef pcl::PointCloud<PRGB> PCRGB;

void pubLoop(PCRGB &pc, const std::string& topic) 
{
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

class EllispoidVisualizer 
{
private:
    ros::Publisher pub_pc;
    ros::Subscriber sub_e_params;
    PCRGB head_pc;
    bool is_head;

public:
    EllispoidVisualizer(const PCRGB& pc_head, const std::string& topic, bool use_head=true) : 
        head_pc(pc_head), is_head(use_head)
    {
        ros::NodeHandle nh;
        pub_pc = nh.advertise<sensor_msgs::PointCloud2>(topic, 1);
        sub_e_params = nh.subscribe("/ellipsoid_params", 1, &EllispoidVisualizer::publishEllipoid, this);
    }
    void sampleEllipse(double A, double B, double height, PCRGB& pc);
    void projectEllipsoid(double A, double B, double height, const PCRGB& pc, PCRGB& pc_ell);
    void publishEllipoid(hrl_phri_2011::EllipsoidParams::ConstPtr e_params);
};

void EllispoidVisualizer::sampleEllipse(double A, double B, double height, PCRGB& pc) 
{
    Ellipsoid e(A, B);
    double lat = 0, lon = 0;
    int numlat = 8, numlon = 16;
    for(int i=0;i<numlat;i++) {
        lat += PI / numlat;
        lon = 0;
        for(int j=0;j<600;j++) {
            lon += 2 * PI / 600;
            PRGB pt;
            ((uint32_t*) &pt.rgb)[0] = 0xffffffff;
            double x, y, z;
            e.ellipsoidalToCart(lat, lon, height, x, y, z);
            pt.x = x; pt.y = y; pt.z = z;
            pc.points.push_back(pt);
        }
    }
    lat = 0; lon = 0;
    for(int i=0;i<numlon;i++) {
        lat = 0;
        lon += 2 * PI / numlon;
        for(int j=0;j<600;j++) {
            lat += PI / 600;
            PRGB pt;
            ((uint32_t*) &pt.rgb)[0] = 0xffffffff;
            double x, y, z;
            e.ellipsoidalToCart(lat, lon, height, x, y, z);
            pt.x = x; pt.y = y; pt.z = z;
            pc.points.push_back(pt);
        }
    }
}

void EllispoidVisualizer::projectEllipsoid(double A, double B, double height, const PCRGB& pc, PCRGB& pc_ell) 
{
    Ellipsoid e(A, B);
    double lat, lon, hei, x, y, z; //, h, s, l;
    BOOST_FOREACH(PRGB const pt, pc.points) {
        PRGB npt;
        //extractHSL(pt.rgb, h, s, l);
        e.cartToEllipsoidal(pt.x, pt.y, pt.z, lat, lon, hei);
        e.ellipsoidalToCart(lat, lon, height, x, y, z);
        npt.x = x; npt.y = y; npt.z = z;
        npt.rgb = pt.rgb;
        //writeHSL(0, 0, l, npt.rgb);
        pc_ell.points.push_back(npt);
    }
    pc_ell.header.frame_id = "base_link";
    //pubLoop(pc_ell, "/proj_head");
}

void EllispoidVisualizer::publishEllipoid(hrl_phri_2011::EllipsoidParams::ConstPtr e_params) 
{
    PCRGB pc, proj_head;
    double A = 1;
    double E = e_params->E;
    double B = sqrt(1 - SQ(E));
    pc.header.frame_id = "/ellipse_frame";
    sampleEllipse(A, B, e_params->height, pc);
    if(is_head) {
        projectEllipsoid(A, B, e_params->height, head_pc, proj_head);
        proj_head.header.frame_id = "/ellipse_frame";
        pc += proj_head;
    }
    pc.header.stamp = ros::Time::now();
    pub_pc.publish(pc);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ellipsoid_visualizer");
    PCRGB pc_head;
    Ellipsoid ell;
    loadRegisteredHead(argv[1], argv[2], pc_head, ell);
    EllispoidVisualizer ev(pc_head, "/ellipsoid", false);
    ros::spin();
    /*
    ros::NodeHandle nh;
    double lat, lon, height, x, y, z, newx, newy, newz, newlat, newlon, newheight;
    boost::mt19937 rand_gen;
    rand_gen.seed(static_cast<uint32_t>(std::time(0)));
    boost::uniform_real<> rand_lat_dist(-PI/2, PI/2);
    boost::uniform_real<> rand_lon_dist(0, 2 * PI);
    boost::uniform_real<> rand_height_dist(0, 4);
    boost::uniform_real<> rand_x_dist(-1, 1);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > rand_x(rand_gen, rand_x_dist);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > rand_lat(rand_gen, rand_lat_dist);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > rand_lon(rand_gen, rand_lon_dist);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > rand_height(rand_gen, rand_height_dist);
    double maxlon = -100, minlon = 100, maxlat = -100, minlat = 100;
    height = 1.0;
    for(int i=0;i<100;i++) {
        x = rand_x(); y = rand_x(); z = rand_x();
        lat = rand_lat(); lon = rand_lon(); height = rand_height();
        e.ellipsoidalToCart(lat, lon, height, x, y, z);
        e.cartToEllipsoidal(x, y, z, newlat, newlon, newheight);
        printf("%f %f %f\n", lat-newlat, lon-newlon, height-newheight);
        //cout << x << ", " << y << ", " << z << endl;
        e.mollweideProjection(lat, lon, x, y);
        maxlon = max(lon, maxlon);
        minlon = min(lon, minlon);
        maxlat = max(lat, maxlat);
        minlat = min(lat, minlat);
    }
    printf("lon %f, %f; lat %f, %f\n", minlon, maxlon, minlat, maxlat);
    sampleEllipse(e, 0.2);
    */
    return 0;
}
