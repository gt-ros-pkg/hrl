#include <boost/make_shared.hpp>
#include "pcl/io/pcd_io.h"
//#include <pcl/features/feature.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include "pcl/filters/voxel_grid.h"

using namespace pcl;
using namespace std;
typedef KdTree<PointXYZ>::Ptr KdTreePtr;

int main(int argc, char** argv)
{
    // load cloud
    //sensor_msgs::PointCloud2 point_cloud_data;
    sensor_msgs::PointCloud2::Ptr point_cloud_data(new sensor_msgs::PointCloud2());

    printf("Loading point cloud...\n");
    if (pcl::io::loadPCDFile(argv[1], *point_cloud_data) == -1)
    {
        ROS_ERROR_STREAM("Was not able to open pcd file \"" << argv[1] << "\".\n");
        return -1;
    }
  
    ROS_INFO("PointCloud before filtering: %d data points (%s).", 
            point_cloud_data->width * point_cloud_data->height, 
            pcl::getFieldsList(*point_cloud_data).c_str());
    
    printf("height %d\n", point_cloud_data->height);
    printf("width %d\n", point_cloud_data->width);
    printf("fields %d\n", point_cloud_data->fields.size());
    printf("endian %d\n", point_cloud_data->is_bigendian);
    printf("point_step %d\n", point_cloud_data->point_step);
    printf("row_step %d\n", point_cloud_data->row_step);
    printf("is_dense %d\n", point_cloud_data->is_dense);

    // downsample
    ROS_INFO("Downsampling..");
    sensor_msgs::PointCloud2::Ptr cloud_filtered(new sensor_msgs::PointCloud2());
    pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
    sor.setInputCloud(point_cloud_data);
    sor.setLeafSize (0.2, 0.2, 0.2);
    sor.filter(*cloud_filtered);
    ROS_INFO("           after filtering: %d points", cloud_filtered->width * cloud_filtered->height);

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

    //estimate
    printf("Calculating normals...\n");
    normal_estimator.setInputCloud(cloud.makeShared());
    normal_estimator.compute(*normals);

    // calculate FPFH
    //FPFHEstimation<PointXYZ, Normal, FPFHSignature33> fpfh;
    FPFHEstimationOMP<PointXYZ, Normal, FPFHSignature33> fpfh(4);    // instantiate 4 threads

    // object
    PointCloud<FPFHSignature33>::Ptr fpfhs (new PointCloud<FPFHSignature33>());

    // set parameters
    fpfh.setNrSubdivisions(11, 11, 11);
    fpfh.setIndices(indicesptr);
    fpfh.setSearchMethod(tree);
    fpfh.setKSearch(indices.size());

    // estimate
    printf("Calculating fpfh...\n");
    fpfh.setInputNormals(normals);
    fpfh.setInputCloud(cloud.makeShared());
    fpfh.compute(*fpfhs);
    //printf("%d", fpfhs->points);

    for (unsigned int i = 0; i < indicesptr->size(); i++)
    {
        printf("%d ", i);
        for (unsigned int j = 0; j < (11*3); j++)
            printf("%.2f ", fpfhs->points[i].histogram[j]);
        //printf("%d ", (*indicesptr)[i]);
        printf("\n");
    }

    //printf("done.\n");
    //printf("W00t! your code ran!\n");
    return 0;
}
