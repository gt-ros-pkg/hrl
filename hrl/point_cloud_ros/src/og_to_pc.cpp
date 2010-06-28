
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

#include "point_cloud_ros/occupancy_grid.h"
#include "point_cloud_ros/OccupancyGrid.h" // ROS Message

using namespace std;

class OccupancyGridToPointCloud 
{
    private:
        // ROS
        ros::NodeHandle nh_;
        ros::Subscriber sub_og_;
        ros::Publisher pub_points_;

        int queue_size_;
        string og_in_, points_out_;

    public:
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        OccupancyGridToPointCloud () : nh_ ("~"), queue_size_ (1), og_in_ ("/og_in"), points_out_ ("/points_out")
        {
            pub_points_ = nh_.advertise<sensor_msgs::PointCloud> (points_out_, queue_size_);
            sub_og_ = nh_.subscribe (og_in_, queue_size_, &OccupancyGridToPointCloud::cloud_cb_og, this);
            ROS_INFO ("OccupancyGridToPointCloud initialized to transform from OccupancyGrid (%s) to PointCloud (%s).", nh_.resolveName (og_in_).c_str (), nh_.resolveName (points_out_).c_str ());
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /** \brief OccuancyGrid callback */
        void cloud_cb_og(const point_cloud_ros::OccupancyGridConstPtr &msg)
        {
            ROS_INFO("cloud_cb_og called");
            if (pub_points_.getNumSubscribers () <= 0)
            {
                ROS_DEBUG ("[occupancy_grid_converter] Got an OccuancyGrid but no subscribers.");
                return;
            }

            float cx = msg->center.x;
            float cy = msg->center.y;
            float cz = msg->center.z;

            float rx = msg->resolution.x;
            float ry = msg->resolution.y;
            float rz = msg->resolution.z;

            float sx = msg->grid_size.x;
            float sy = msg->grid_size.y;
            float sz = msg->grid_size.z;

            occupancy_grid::OccupancyGrid *v = new
                occupancy_grid::OccupancyGrid(cx, cy, cz, sx, sy, sz, rx, ry, rz);

            uint32_t* d = v->getData();
            int nCells = v->nX() * v->nY() * v->nZ();
            for (int i=0; i<nCells; i++)
                d[i] = msg->data[i];

            sensor_msgs::PointCloud pc = v->gridToPoints();
            pc.header = msg->header;
            pub_points_.publish(pc);

            delete v;
        }
};


int main (int argc, char** argv)
{
    // ROS init
    ros::init (argc, argv, "og_to_pc", ros::init_options::AnonymousName);
    OccupancyGridToPointCloud p;
    ros::spin();

    return (0);
}


