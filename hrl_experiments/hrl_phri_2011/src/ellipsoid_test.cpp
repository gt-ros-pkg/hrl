#include <hrl_phri_2011/ellipsoid_space.h>
#include <ros/ros.h>
#include <boost/random/uniform_real.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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

void sampleEllipse(Ellipsoid& e, double height) 
{
    PCRGB pc;
    double lat = 0, lon = 0;
    int numlat = 8, numlon = 14;
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
    pc.header.frame_id = "/base_link";
    pc.header.stamp = ros::Time::now();
    pubLoop(pc, "/sample_ellipsoid");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    Ellipsoid e(1.0, 2.5);
    double lat, lon, height, x, y, z, newx, newy, newz, newlat, newlon, newheight;
    boost::mt19937 rand_gen;
    rand_gen.seed(static_cast<uint32_t>(std::time(0)));
    boost::uniform_real<> rand_lat_dist(0, PI);
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
        if(x > 0 && y > 0)
            printf("1:");
        if(x < 0 && y > 0)
            printf("2:");
        if(x < 0 && y < 0)
            printf("3:");
        if(x > 0 && y < 0)
            printf("4:");
        //printf("%f %f %f\n", lon-newlon, lon, newlon);
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
    return 0;
}
