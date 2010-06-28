
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

#include "point_cloud_ros/occupancy_grid.h"
#include "point_cloud_ros/OccupancyGrid.h" // ROS Message

using namespace std;

class PointCloudToOccupancyGrid 
{
    private:
        // ROS
        ros::NodeHandle nh_;
        ros::Subscriber sub_points_;
        ros::Publisher pub_og_;

        int queue_size_;
        string points_in_, og_out_;

    public:

        PointCloudToOccupancyGrid () : nh_ ("~"), queue_size_ (1), points_in_ ("/points_in"), og_out_ ("/og_out")
        {
            pub_og_ = nh_.advertise<point_cloud_ros::OccupancyGrid> (og_out_, queue_size_);
            sub_points_ = nh_.subscribe (points_in_, queue_size_, &PointCloudToOccupancyGrid::cloud_cb_points, this);
            ROS_INFO ("PointCloudToOccupancyGrid initialized to transform from PointCloud (%s) to OccupancyGrid (%s).", nh_.resolveName (points_in_).c_str (), nh_.resolveName (og_out_).c_str ());
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /** \brief PointCloud (old format) callback */
        void cloud_cb_points (const sensor_msgs::PointCloudConstPtr &cloud)
        {
            if (pub_og_.getNumSubscribers () <= 0)
            {
                ROS_DEBUG ("[occupancy_grid_converter] Got a PointCloud with %d points on %s, but no subscribers.", (int)cloud->points.size(), nh_.resolveName(points_in_).c_str());
                return;
            }

            float cx = 0;
            float cy = 0;
            float cz = 0;
            for (size_t i = 0; i < cloud->points.size(); i++)
            {
                cx += cloud->points[i].x;
                cy += cloud->points[i].y;
                cz += cloud->points[i].z;
            }

            cx = cx / cloud->points.size();
            cy = cy / cloud->points.size();
            cz = cz / cloud->points.size();
            ROS_INFO("Centroid of the point cloud: (%.2f, %.2f, %.2f)", cx, cy, cz);

            float rx, ry, rz;
            rx = 0.02;
            ry = 0.02;
            rz = 0.02;

            float sx, sy, sz;
            sx = 1.5;
            sy = 1.5;
            sz = 1.5;
            occupancy_grid::OccupancyGrid *v = new
                occupancy_grid::OccupancyGrid(cx, cy, cz, sx, sy, sz, rx, ry, rz);
            v->fillOccupancyGrid(*cloud);

            point_cloud_ros::OccupancyGrid og_msg;
            uint32_t* d = v->getData();
            int nCells = v->nX() * v->nY() * v->nZ();

            og_msg.data.resize(nCells);
            for (int i=0; i<nCells; i++)
                og_msg.data[i] = d[i];

            og_msg.header = cloud->header;
            og_msg.center.x = cx;
            og_msg.center.y = cy;
            og_msg.center.z = cz;

            ROS_INFO("frame_id: %s", cloud->header.frame_id.c_str());

            og_msg.resolution.x = rx;
            og_msg.resolution.y = ry;
            og_msg.resolution.z = rz;

            og_msg.grid_size.x = sx;
            og_msg.grid_size.y = sy;
            og_msg.grid_size.z = sz;

            pub_og_.publish(og_msg);

            delete v;
        }
};


int main (int argc, char** argv)
{
    // ROS init
    ros::init (argc, argv, "pc_to_og", ros::init_options::AnonymousName);
    PointCloudToOccupancyGrid p;
    ros::spin();

    return (0);
}


