
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
        void cloud_cb_og (const point_cloud_ros::OccupancyGrid::ConstPtr &msg)
        {
            if (pub_points_.getNumSubscribers () <= 0)
            {
                ROS_DEBUG ("[occupancy_grid_converter] Got an OccuancyGrid but no subscribers.");
                return;
            }

            sensor_msgs::PointCloud output;
//            // Convert to the new PointCloud format
//            if (!sensor_msgs::convertPointCloud2ToPointCloud (*msg, output))
//            {
//                ROS_ERROR ("[occupancy_grid_converter] Conversion from sensor_msgs::PointCloud2 to sensor_msgs::PointCloud failed!");
//                return;
//            }
//            ROS_INFO ("[occupancy_grid_converter] Publishing a PointCloud with %d points on %s.", (int)output.points.size (), nh_.resolveName (points_out_).c_str ());
            pub_points_.publish (output);
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

