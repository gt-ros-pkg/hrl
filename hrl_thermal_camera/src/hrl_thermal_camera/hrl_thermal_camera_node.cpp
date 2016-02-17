/**
 * @file ros_driver.cpp
 *
 * Copyright 2013
 * Carnegie Robotics, LLC
 * 4501 Hatfield Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Robotics, LLC nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CARNEGIE ROBOTICS, LLC BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

#include <multisense_ros/laser.h>
#include <multisense_ros/camera.h>
#include <multisense_ros/pps.h>
#include <multisense_ros/imu.h>
#include <multisense_ros/reconfigure.h>
#include <ros/ros.h>

using namespace crl::multisense;

int main(int    argc,
         char** argvPP)
{
    ros::init(argc, argvPP, "multisense_driver");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private_("~");

    //
    // Get parameters from ROS/command-line

    std::string sensor_ip;
    std::string tf_prefix;
    int         sensor_mtu;


    nh_private_.param<std::string>("sensor_ip", sensor_ip, "10.66.171.21");
    nh_private_.param<std::string>("tf_prefix", tf_prefix, "multisense");
    nh_private_.param<int>("sensor_mtu", sensor_mtu, 7200);

    Channel *d = NULL;

    try {

        d = Channel::Create(sensor_ip);
	if (NULL == d) {
            ROS_ERROR("multisense_ros: failed to create communication channel to sensor @ \"%s\"",
                      sensor_ip.c_str());
            return -2;
        }

        Status status = d->setMtu(sensor_mtu);
        if (Status_Ok != status) {
            ROS_ERROR("multisense_ros: failed to set sensor MTU to %d: %s",
                      sensor_mtu, Channel::statusString(status));
            Channel::Destroy(d);
            return -3;
        }

        //
        // Anonymous namespace so objects can deconstruct before channel is destroyed
        {
            multisense_ros::Laser        laser(d, tf_prefix);
            multisense_ros::Camera       camera(d, tf_prefix);
            multisense_ros::Pps          pps(d);
            multisense_ros::Imu          imu(d, tf_prefix);
            multisense_ros::Reconfigure  rec(d,
                                             boost::bind(&multisense_ros::Camera::resolutionChanged, &camera),
                                             boost::bind(&multisense_ros::Camera::borderClipChanged, &camera, _1, _2));
            ros::spin();
        }

        Channel::Destroy(d);
        return 0;

    } catch (const std::exception& e) {
        ROS_ERROR("multisense_ros: caught exception: %s", e.what());
        Channel::Destroy(d);
        return -4;
    }
}
