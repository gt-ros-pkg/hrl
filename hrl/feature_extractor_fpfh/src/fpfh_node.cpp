#include "ros/ros.h"
//#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"

#include <boost/make_shared.hpp>
#include "pcl/io/pcd_io.h"
#include <pcl/features/fpfh_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include "pcl/filters/voxel_grid.h"

//#include "feature_extractor_fpfh/AddTwoInts.h"
#include "feature_extractor_fpfh/FPFHCalc.h"
#include "feature_extractor_fpfh/FPFHHist.h"
#include "feature_extractor_fpfh/fpfh_node.h"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

using namespace pcl;
using namespace std;
using namespace message_filters;
typedef KdTree<PointXYZ>::Ptr KdTreePtr;

using namespace message_filters;


FPFHNode::FPFHNode(ros::NodeHandle &n): n_(n)
{
    hist_publisher = n_.advertise<feature_extractor_fpfh::FPFHHist>("fpfh_hist", 10);
}


void FPFHNode::process_point_cloud(const sensor_msgs::Image::ConstPtr& image, 
                        const sensor_msgs::PointCloud2::ConstPtr& input_cloud,
                        const feature_extractor_fpfh::FPFHHist &hist)
{
}


void FPFHNode::callback(const sensor_msgs::Image::ConstPtr& image, 
                        const sensor_msgs::PointCloud2::ConstPtr& input_cloud)
{
    if (hist_publisher.getNumSubscribers() <= 0)
    {
        ROS_DEBUG("no subscribers. stopped processing.");
        return;
    }

    float fpfh_leaf_size = .02;
    ROS_INFO("cloud has %d points", input_cloud->width * input_cloud->height);

    sensor_msgs::PointCloud2::Ptr cloud_filtered(new sensor_msgs::PointCloud2());
    pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
    sor.setInputCloud(input_cloud);
    sor.setLeafSize (fpfh_leaf_size, fpfh_leaf_size, fpfh_leaf_size);
    sor.filter(*cloud_filtered);


    float leaf_size = .005;
    sensor_msgs::PointCloud2::Ptr cloud_filtered2(new sensor_msgs::PointCloud2());
    pcl::VoxelGrid<sensor_msgs::PointCloud2> sor2;
    sor2.setInputCloud(input_cloud);
    sor2.setLeafSize (leaf_size, leaf_size, leaf_size);
    sor2.filter(*cloud_filtered2);
    PointCloud<PointXYZ> pcl_input_cloud_small;
    fromROSMsg(*cloud_filtered2, pcl_input_cloud_small);

    ROS_INFO("after filtering: %d points", cloud_filtered->width * cloud_filtered->height);
    if ((cloud_filtered->width * cloud_filtered->height) == 0)
    {
        return;
    }
    PointCloud<PointXYZ> cloud, pcl_input_cloud;
    fromROSMsg(*cloud_filtered, cloud);
    fromROSMsg(*input_cloud, pcl_input_cloud);
    std::vector<int> indices;
    indices.resize (cloud.points.size());
    for (size_t i = 0; i < indices.size (); ++i) { indices[i] = i; }

    KdTreePtr tree;
    tree.reset(new KdTreeFLANN<PointXYZ> (false));
    tree->setInputCloud(cloud.makeShared());

    PointCloud<Normal>::Ptr normals(new PointCloud<Normal>());
    boost::shared_ptr< vector<int> > indicesptr(new vector<int> (indices));
    NormalEstimation<PointXYZ, Normal> normal_estimator;

    normal_estimator.setIndices(indicesptr);
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setKSearch(10); // Use 10 nearest neighbors to estimate the normals

    // estimate
    ROS_DEBUG("fpfh_Calculating normals...\n");
    normal_estimator.setInputCloud(cloud.makeShared());
    normal_estimator.compute(*normals);


    FPFHEstimationOMP<PointXYZ, Normal, FPFHSignature33> fpfh(4);    // instantiate 4 threads
    // object
    PointCloud<FPFHSignature33>::Ptr fphists (new PointCloud<FPFHSignature33>());

    // set parameters
    int d1, d2, d3;
    d1 = d2 = d3 = 11;
    fpfh.setNrSubdivisions(d1, d2, d3);
    fpfh.setSearchMethod(tree);
    fpfh.setIndices(indicesptr);
    fpfh.setKSearch(40);

    ROS_DEBUG("Calculating fpfh...\n");
    fpfh.setInputNormals(normals);
    fpfh.setInputCloud(cloud.makeShared());
    fpfh.compute(*fphists);

    feature_extractor_fpfh::FPFHHist hist;
    hist.header = input_cloud->header;
    hist.dim[0] = d1;
    hist.dim[1] = d2;
    hist.dim[2] = d3;
    unsigned int total_size = d1+d2+d3;
    hist.histograms.resize(total_size * cloud.points.size());
    hist.hpoints3d.resize(3*cloud.points.size());
    ROS_DEBUG("copying into message...\n");
    for (unsigned int i = 0; i < cloud.points.size(); i++)
    {
        for (unsigned int j = 0; j < total_size; j++)
        {
            hist.histograms[i*total_size + j] = fphists->points[i].histogram[j];
        }

        hist.hpoints3d[3*i]   = cloud.points[i].x;
        hist.hpoints3d[3*i+1] = cloud.points[i].y;
        hist.hpoints3d[3*i+2] = cloud.points[i].z;
    }
    hist.hist_npoints = cloud.points.size();

    hist.origpoints.resize(3*pcl_input_cloud_small.points.size());
    for (unsigned int i = 0; i < pcl_input_cloud_small.points.size(); i++)
    {
        hist.origpoints[3*i]   = pcl_input_cloud_small.points[i].x;
        hist.origpoints[3*i+1] = pcl_input_cloud_small.points[i].y;
        hist.origpoints[3*i+2] = pcl_input_cloud_small.points[i].z;
        //ROS_INFO("%.2f %.2f %.2f", pcl_input_cloud.points[i].x,
        //        pcl_input_cloud.points[i].y,
        //        pcl_input_cloud.points[i].z);
    }
    hist.original_npoints = pcl_input_cloud_small.size();

    hist.image.header = image->header;
    hist.image.height = image->height;
    hist.image.width = image->width;
    hist.image.encoding = image->encoding;
    hist.image.is_bigendian = image->is_bigendian;
    hist.image.step = image->step;
    hist.image.data.resize(image->data.size());
    for (unsigned int i = 0; i < image->data.size(); i++)
    {
        hist.image.data[i] = image->data[i];
    }

    hist_publisher.publish(boost::make_shared<const feature_extractor_fpfh::FPFHHist>(hist));
    ROS_INFO("publish histogram done.\n");

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fpfh_calc");
  ros::NodeHandle nh;
  FPFHNode mc(nh);

  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "image", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, "points2", 1);
  TimeSynchronizer<sensor_msgs::Image, sensor_msgs::PointCloud2> sync(image_sub, pc_sub, 10);
  sync.registerCallback(boost::bind(&FPFHNode::callback, &mc, _1, _2));

  ROS_INFO("started!");
  ros::spin();
  return 0;
}





