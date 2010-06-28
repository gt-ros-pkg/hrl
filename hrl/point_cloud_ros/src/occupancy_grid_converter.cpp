
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

#include "point_cloud_ros/occupancy_grid.h"
#include "point_cloud_ros/OccupancyGrid.h" // ROS Message

using namespace std;

class OccupancyGridConverter 
{
    private:
        // ROS
        ros::NodeHandle nh_;
        ros::Subscriber sub_points_, sub_og_;
        ros::Publisher pub_points_, pub_og_;

        int queue_size_;
        string og_in_, points_in_, og_out_, points_out_;

    public:
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        OccupancyGridConverter () : nh_ ("~"), queue_size_ (100), og_in_ ("/og_in"), points_in_ ("/points_in"),
                                    og_out_ ("/og_out"), points_out_ ("/points_out")
        {
            // Subscribe to the cloud topic using both the old message format and the new
            sub_points_ = nh_.subscribe (points_in_, queue_size_, &OccupancyGridConverter::cloud_cb_points, this);
            sub_og_ = nh_.subscribe (og_in_, queue_size_, &OccupancyGridConverter::cloud_cb_og, this);
            pub_points_ = nh_.advertise<sensor_msgs::PointCloud> (points_out_, queue_size_);
            pub_og_ = nh_.advertise<point_cloud_ros::OccupancyGrid> (og_out_, queue_size_);
            ROS_INFO ("OccupancyGridConverter initialized to transform from PointCloud (%s) to OccupancyGrid (%s).", nh_.resolveName (points_in_).c_str (), nh_.resolveName (og_out_).c_str ());
            ROS_INFO ("OccupancyGridConverter initialized to transform from OccupancyGrid (%s) to PointCloud (%s).", nh_.resolveName (og_in_).c_str (), nh_.resolveName (points_out_).c_str ());
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /** \brief OccuancyGrid callback */
        void cloud_cb_og (const point_cloud_ros::OccupancyGrid &msg)
        {
            if (pub_points_.getNumSubscribers () <= 0)
            {
                ROS_DEBUG ("[occupancy_grid_converter] Got an OccuancyGrid but no subscribers.");
                return;
            }

            float cx = msg.center.x;
            float cy = msg.center.y;
            float cz = msg.center.z;

            float rx = msg.resolution.x;
            float ry = msg.resolution.y;
            float rz = msg.resolution.z;

            float sx = msg.grid_size.x;
            float sy = msg.grid_size.y;
            float sz = msg.grid_size.z;

            occupancy_grid::OccupancyGrid *v = new
                occupancy_grid::OccupancyGrid(cx, cy, cz, sx, sy, sz, rx, ry, rz);

            uint32_t* d = v->getData();
            int nCells = v->nX() * v->nY() * v->nZ();
            for (int i=0; i<nCells; i++)
                d[i] = msg.data[i];

            pub_points_.publish(v->gridToPoints());
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /** \brief PointCloud (old format) callback */
        void cloud_cb_points (const sensor_msgs::PointCloudConstPtr &msg)
        {
            if (pub_og_.getNumSubscribers () <= 0)
            {
                ROS_DEBUG ("[occupancy_grid_converter] Got a PointCloud with %d points on %s, but no subscribers.", (int)msg->points.size (), nh_.resolveName (points_in_).c_str ());
                return;
            }

            point_cloud_ros::OccupancyGrid output;
            pub_og_.publish (output);
        }

};

/* ---[ */
int main (int argc, char** argv)
{
    // ROS init
    ros::init (argc, argv, "occupancy_grid_converter", ros::init_options::AnonymousName);

    OccupancyGridConverter p;
    ros::spin ();

    return (0);
}

