#ifndef HRL_PHRI_2011_UTILS_H
#define HRL_PHRI_2011_UTILS_H

#include <string>
#include <boost/foreach.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>

#include <hrl_phri_2011/EllipsoidParams.h>
#include <hrl_phri_2011/pcl_basic.h>
#include <hrl_phri_2011/ellipsoid_space.h>
#include <tf_conversions/tf_eigen.h>

using namespace boost;
using namespace std;

template <class M>
void readBagTopic(const std::string& filename, vector< shared_ptr< M > >& msgs, 
                  const string& topic = string(""));

template <class M>
void readBagTopic(const std::string& filename, vector< shared_ptr< M > >& msgs, const string& topic) {
    rosbag::Bag bag;
    bag.open(filename, rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery(topic));
    shared_ptr< M > cur_msg;
    BOOST_FOREACH(rosbag::MessageInstance const msg, view) {
        msgs.push_back(msg.instantiate< M >());
        /*
        cur_msg = msg.instantiate< M >();
        if(cur_msg != NULL){
            msgs.push_back(shared_ptr< M >(new M(*cur_msg)));
            ROS_INFO("%s %s", filename.c_str(), topic.c_str());
        }
        */
    }
    bag.close();
}

void applyRegistration(const PCRGB& in_pc, const hrl_phri_2011::EllipsoidParams& params, PCRGB& out_pc);
void loadRegisteredHead(const string& head_bag, const string& params_bag, PCRGB& out_pc, Ellipsoid& ell);

#endif // HRL_PHRI_2011_UTILS_H
