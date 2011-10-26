
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <hrl_rfh_fall_2011/PCSwitch.h>

using namespace std;

class PCThrottledSwitch
{
    ros::Subscriber pc_sub;
    ros::Publisher pc_pub;
    ros::ServiceServer switch_srv;
    sensor_msgs::PointCloud2::Ptr cur_pc;
    ros::Timer pc_pub_timer;
    bool switch_on, throttle;

    public:
    PCThrottledSwitch(const string& topic_name, double rate) :
        switch_on(true), throttle(rate > 0)
    {
        ros::NodeHandle nh;
        pc_sub = nh.subscribe(topic_name, 1, &PCThrottledSwitch::subPCCallback, this);
        pc_pub = nh.advertise<sensor_msgs::PointCloud2>(topic_name + "_throttled", 10);
        switch_srv = nh.advertiseService("pc_switch", &PCThrottledSwitch::srvPCCallback, this);
        if(throttle)
            pc_pub_timer = nh.createTimer(ros::Duration(1. / rate), &PCThrottledSwitch::timerCB, this);
    }

    private:
    void subPCCallback(sensor_msgs::PointCloud2::Ptr msg)
    {
        cur_pc = msg;
        if(!throttle && switch_on)
            pc_pub.publish(cur_pc);
    }

    bool srvPCCallback(hrl_rfh_fall_2011::PCSwitch::Request& req, 
                       hrl_rfh_fall_2011::PCSwitch::Response& resp)
    {
        switch_on = req.switch_on;
        return true;
    }

    void timerCB(const ros::TimerEvent& event)
    {
        if(cur_pc && switch_on)
            pc_pub.publish(cur_pc);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pc_throttled_switch");

    if(argc <= 1)
        printf("Usage: pc_throttled_switch topic_name [rate]");

    double rate = -1;
    if(argc >= 3)
        rate = atof(argv[2]);

    PCThrottledSwitch pc_tswitch(argv[1], rate);
    ros::spin();

}
