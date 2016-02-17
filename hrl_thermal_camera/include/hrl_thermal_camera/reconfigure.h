/**
 * @file reconfigure.h
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

#ifndef MULTISENSE_ROS_RECONFIGURE_H
#define MULTISENSE_ROS_RECONFIGURE_H

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>

#include <multisense_lib/MultiSenseChannel.hh>

#include <dynamic_reconfigure/server.h>
#include <multisense_ros/sl_bm_cmv2000Config.h>
#include <multisense_ros/sl_bm_cmv2000_imuConfig.h>
#include <multisense_ros/sl_bm_cmv4000Config.h>
#include <multisense_ros/sl_bm_cmv4000_imuConfig.h>
#include <multisense_ros/sl_sgm_cmv2000_imuConfig.h>
#include <multisense_ros/sl_sgm_cmv4000_imuConfig.h>
#include <multisense_ros/bcam_imx104Config.h>
#include <multisense_ros/st21_sgm_vga_imuConfig.h>
#include <multisense_ros/mono_cmv2000Config.h>
#include <multisense_ros/mono_cmv4000Config.h>

namespace multisense_ros {

class Reconfigure {
public:

    Reconfigure(crl::multisense::Channel* driver,
                boost::function<void ()> resolutionChangeCallback=0,
                boost::function<void (int, int)> borderClipChangeCallback=0);

    ~Reconfigure();

    void imuCallback(const crl::multisense::imu::Header& header);

private:

    //
    // Dynamic reconfigure callback variations

    void callback_sl_bm_cmv2000     (multisense_ros::sl_bm_cmv2000Config&      config, uint32_t level);
    void callback_sl_bm_cmv2000_imu (multisense_ros::sl_bm_cmv2000_imuConfig&  config, uint32_t level);
    void callback_sl_bm_cmv4000     (multisense_ros::sl_bm_cmv4000Config&      config, uint32_t level);
    void callback_sl_bm_cmv4000_imu (multisense_ros::sl_bm_cmv4000_imuConfig&  config, uint32_t level);
    void callback_sl_sgm_cmv2000_imu(multisense_ros::sl_sgm_cmv2000_imuConfig& config, uint32_t level);
    void callback_sl_sgm_cmv4000_imu(multisense_ros::sl_sgm_cmv4000_imuConfig& config, uint32_t level);
    void callback_bcam_imx104       (multisense_ros::bcam_imx104Config&        config, uint32_t level);
    void callback_st21_vga          (multisense_ros::st21_sgm_vga_imuConfig&   config, uint32_t level);
    void callback_mono_cmv2000      (multisense_ros::mono_cmv2000Config&       config, uint32_t level);
    void callback_mono_cmv4000      (multisense_ros::mono_cmv4000Config&       config, uint32_t level);

    //
    // Internal helper functions

    bool changeResolution(crl::multisense::image::Config& cfg,
                          int32_t width, int32_t height, int32_t disparities);
    template<class T> void configureSgm(crl::multisense::image::Config& cfg, const T& dyn);
    template<class T> void configureCamera(crl::multisense::image::Config& cfg, const T& dyn);
    template<class T> void configureImu(const T& dyn);
    template<class T> void configureBorderClip(const T& dyn);

    //
    // CRL sensor API

    crl::multisense::Channel* driver_;

    //
    // Resolution change callback

    boost::function<void ()> resolution_change_callback_;

    //
    // Driver nodes

    ros::NodeHandle device_nh_;

    //
    // Cached modes from the sensor

    std::vector<crl::multisense::system::DeviceMode> device_modes_;
    uint32_t                                         imu_samples_per_message_;
    std::vector<crl::multisense::imu::Config>        imu_configs_;

    //
    // Dynamic reconfigure server variations

    boost::shared_ptr< dynamic_reconfigure::Server<multisense_ros::sl_bm_cmv2000Config> >      server_sl_bm_cmv2000_;
    boost::shared_ptr< dynamic_reconfigure::Server<multisense_ros::sl_bm_cmv2000_imuConfig> >  server_sl_bm_cmv2000_imu_;
    boost::shared_ptr< dynamic_reconfigure::Server<multisense_ros::sl_bm_cmv4000Config> >      server_sl_bm_cmv4000_;
    boost::shared_ptr< dynamic_reconfigure::Server<multisense_ros::sl_bm_cmv4000_imuConfig> >  server_sl_bm_cmv4000_imu_;
    boost::shared_ptr< dynamic_reconfigure::Server<multisense_ros::sl_sgm_cmv2000_imuConfig> > server_sl_sgm_cmv2000_imu_;
    boost::shared_ptr< dynamic_reconfigure::Server<multisense_ros::sl_sgm_cmv4000_imuConfig> > server_sl_sgm_cmv4000_imu_;
    boost::shared_ptr< dynamic_reconfigure::Server<multisense_ros::bcam_imx104Config> >        server_bcam_imx104_;
    boost::shared_ptr< dynamic_reconfigure::Server<multisense_ros::st21_sgm_vga_imuConfig> >   server_st21_vga_;
    boost::shared_ptr< dynamic_reconfigure::Server<multisense_ros::mono_cmv2000Config> >       server_mono_cmv2000_;
    boost::shared_ptr< dynamic_reconfigure::Server<multisense_ros::mono_cmv4000Config> >       server_mono_cmv4000_;

    //
    // Cached values for supported sub-systems (these may be unavailable on
    // certain hardware variations: S7, S7S, M, etc.)

    bool lighting_supported_;
    bool motor_supported_;

    //
    // Cached value for the boarder clip. These are used to determine if we
    // should regenerate our border clipping mask

    enum clip_ {RECTANGULAR, CIRCULAR};

    int border_clip_type_;
    double border_clip_value_;

    //
    // Border clip change callback

    boost::function<void (int, int)> border_clip_change_callback_;
};

} // multisense_ros

#endif

