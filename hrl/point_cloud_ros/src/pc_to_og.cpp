
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>

#include "point_cloud_ros/occupancy_grid.h"
#include "point_cloud_ros/OccupancyGrid.h" // ROS Message

using namespace std;

class PointCloudToOccupancyGrid 
{
    private:
        // ROS
        ros::NodeHandle nh_;
        ros::Subscriber sub_points_;
        ros::Subscriber sub_og_params_;
        ros::Publisher pub_og_;

        int queue_size_;
        string points_in_, og_out_;

        string og_params_;
        geometry_msgs::Point32 center_;
        geometry_msgs::Point32 resolution_;
        geometry_msgs::Point32 size_;
        int occupancy_threshold_;
        string og_frame_id_;

        tf::TransformListener tf_listener;

    public:

        PointCloudToOccupancyGrid () : nh_ ("~"), queue_size_ (1), points_in_ ("/points_in"), og_out_ ("/og_out"), og_params_ ("/og_params")
        {
            pub_og_ = nh_.advertise<point_cloud_ros::OccupancyGrid> (og_out_, queue_size_);
            sub_points_ = nh_.subscribe (points_in_, queue_size_, &PointCloudToOccupancyGrid::cloud_cb_points, this);
            sub_og_params_ = nh_.subscribe(og_params_, queue_size_, &PointCloudToOccupancyGrid::og_params_cb, this);
            ROS_INFO ("PointCloudToOccupancyGrid initialized to transform from PointCloud (%s) to OccupancyGrid (%s).", nh_.resolveName (points_in_).c_str (), nh_.resolveName (og_out_).c_str ());

            resolution_.x = 0.01;
            resolution_.y = 0.01;
            resolution_.z = 0.01;

            size_.x = 0.1;
            size_.y = 0.1;
            size_.z = 0.1;

            occupancy_threshold_ = 1;
            og_frame_id_ = "/base_link";
        }

        /** \brief PointCloud (old format) callback */
        void cloud_cb_points (const sensor_msgs::PointCloudConstPtr &cloud)
        {
            if (pub_og_.getNumSubscribers () <= 0)
            {
                ROS_DEBUG ("[occupancy_grid_converter] Got a PointCloud with %d points on %s, but no subscribers.", (int)cloud->points.size(), nh_.resolveName(points_in_).c_str());
                return;
            }

            sensor_msgs::PointCloud c;
            c.header = cloud->header;
            c.points.resize((cloud->points).size());
            for (unsigned int i=0; i<c.points.size(); i++)
                c.points[i] = cloud->points[i];

            tf_listener.transformPointCloud(og_frame_id_, c, c);

            float cx = center_.x;
            float cy = center_.y;
            float cz = center_.z;

            float rx = resolution_.x;
            float ry = resolution_.y;
            float rz = resolution_.z;

            float sx = size_.x;
            float sy = size_.y;
            float sz = size_.z;

            occupancy_grid::OccupancyGrid *v = new
                occupancy_grid::OccupancyGrid(cx, cy, cz, sx, sy, sz, rx, ry, rz);
            v->fillOccupancyGrid(c);

            point_cloud_ros::OccupancyGrid og_msg;
            uint32_t* d = v->getData();
            int nCells = v->nX() * v->nY() * v->nZ();

            og_msg.data.resize(nCells);
            for (int i=0; i<nCells; i++)
                og_msg.data[i] = d[i];

            og_msg.header = c.header;
            og_msg.center.x = cx;
            og_msg.center.y = cy;
            og_msg.center.z = cz;

//            ROS_INFO("frame_id: %s", cloud->header.frame_id.c_str());
            og_msg.resolution.x = rx;
            og_msg.resolution.y = ry;
            og_msg.resolution.z = rz;

            og_msg.grid_size.x = sx;
            og_msg.grid_size.y = sy;
            og_msg.grid_size.z = sz;

            og_msg.occupancy_threshold = occupancy_threshold_;
            pub_og_.publish(og_msg);
            delete v;
        }

        void og_params_cb(const point_cloud_ros::OccupancyGrid &og_param)
        {
            center_ = og_param.center;
            size_ = og_param.grid_size;
            resolution_ = og_param.resolution;
            occupancy_threshold_ = og_param.occupancy_threshold;
            og_frame_id_ = og_param.header.frame_id;
            ROS_INFO("og_params_cb called");
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


