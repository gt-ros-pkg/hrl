#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"

#include <boost/make_shared.hpp>
#include "pcl/io/pcd_io.h"
#include <pcl/features/fpfh_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include "pcl/filters/voxel_grid.h"

//#include "feature_extractor_fpfh/AddTwoInts.h"
#include "feature_extractor_fpfh/FPFHCalc.h"

using namespace pcl;
using namespace std;
typedef KdTree<PointXYZ>::Ptr KdTreePtr;


bool fpfh_cb(feature_extractor_fpfh::FPFHCalc::Request &req,
             feature_extractor_fpfh::FPFHCalc::Response &res)
{
    float leaf_size = .01;
    pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
    sensor_msgs::PointCloud2::Ptr input_cloud(new sensor_msgs::PointCloud2());
    sensor_msgs::convertPointCloudToPointCloud2(req.input, *input_cloud);
    //sensor_msgs::PointCloud2::Ptr input_cloud(&req.input);

    sensor_msgs::PointCloud2::Ptr cloud_filtered(new sensor_msgs::PointCloud2());
    sor.setInputCloud(input_cloud);
    sor.setLeafSize (leaf_size, leaf_size, leaf_size);
    sor.filter(*cloud_filtered);
    ROS_INFO("after filtering: %d points", cloud_filtered->width * cloud_filtered->height);

    PointCloud<PointXYZ> cloud;
    fromROSMsg(*cloud_filtered, cloud);
    std::vector<int> indices;
    indices.resize (cloud.points.size());
    for (size_t i = 0; i < indices.size (); ++i) { indices[i] = i; }

    // make tree
    KdTreePtr tree;
    tree.reset(new KdTreeFLANN<PointXYZ> (false));
    tree->setInputCloud(cloud.makeShared());

    PointCloud<Normal>::Ptr normals(new PointCloud<Normal>());
    boost::shared_ptr< vector<int> > indicesptr(new vector<int> (indices));
    NormalEstimation<PointXYZ, Normal> normal_estimator;

    // set normal estimation parameters
    normal_estimator.setIndices(indicesptr);
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setKSearch(10); // Use 10 nearest neighbors to estimate the normals

    // estimate
    ROS_INFO("Calculating normals...\n");
    normal_estimator.setInputCloud(cloud.makeShared());
    normal_estimator.compute(*normals);

    // calculate FPFH
    //FPFHEstimation<PointXYZ, Normal, FPFHSignature33> fpfh;
    FPFHEstimationOMP<PointXYZ, Normal, FPFHSignature33> fpfh(4);    // instantiate 4 threads

    // object
    PointCloud<FPFHSignature33>::Ptr fphists (new PointCloud<FPFHSignature33>());

    // set parameters
    int d1, d2, d3;
    d1 = d2 = d3 = 11;
    fpfh.setNrSubdivisions(d1, d2, d3);
    fpfh.setIndices(indicesptr);
    fpfh.setSearchMethod(tree);
    fpfh.setKSearch(50);

    // estimate
    ROS_INFO("Calculating fpfh...\n");
    fpfh.setInputNormals(normals);
    fpfh.setInputCloud(cloud.makeShared());
    fpfh.compute(*fphists);

    res.hist.dim[0] = d1;
    res.hist.dim[1] = d2;
    res.hist.dim[2] = d3;
    unsigned int total_size = d1+d2+d3;
    res.hist.histograms.resize(total_size * cloud.points.size());
    res.hist.points3d.resize(3*cloud.points.size());

    ROS_INFO("copying into message...\n");
    for (unsigned int i = 0; i < cloud.points.size(); i++)
    {
        for (unsigned int j = 0; j < total_size; j++)
        {
            res.hist.histograms[i*total_size + j] = fphists->points[i].histogram[j];
            //if (i == 0)
            //{
            //    printf(">> %.2f \n", fphists->points[i].histogram[j]);
            //}

            //if (i == 4)
            //{
            //    printf("X %.2f \n", fphists->points[i].histogram[j]);
            //}
        }

        res.hist.points3d[3*i]   = cloud.points[i].x;
        res.hist.points3d[3*i+1] = cloud.points[i].y;
        res.hist.points3d[3*i+2] = cloud.points[i].z;
        //if (i == 0)
        //    printf(">> 0  %.4f %.4f %.4f \n", cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
        //if (i == 4)
        //    printf(">> 4  %.4f %.4f %.4f \n", cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
    }

    res.hist.npoints = cloud.points.size();
    ROS_INFO("done.\n");
    //printf("%d\n", );
    // sensor_msgs::PointCloud2 req
    // new feature_extractor::FPFHist()
    return true;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "fpfh_feature_extractor");
  ros::NodeHandle n;
  //ros::ServiceServer service = n.advertiseService("fpfh", fpfh);
  //ros::ServiceServer service = n.advertiseService("add", add);
  ros::ServiceServer fpfh_service = n.advertiseService("fpfh", fpfh_cb);

  //ros::ServiceServer service = n.advertiseService("fpfh", boost::bind(&fpfh, _1, _2));
  ROS_INFO("ready.\n");
  ros::spin();
  return 0;
}







//bool add(feature_extractor::AddTwoInts::Request  &req,
//         feature_extractor::AddTwoInts::Response &res )
//{
//  res.sum = req.a + req.b;
//  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
//  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
//  return true;
//}
