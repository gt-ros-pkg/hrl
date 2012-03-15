#include "pr2_overhead_grasping/sensor_filter.h"
#include <pluginlib/class_list_macros.h>
#include "omp.h"
PLUGINLIB_DECLARE_CLASS(collision_detection, sensor_filter, collision_detection::SensorFilter, nodelet::Nodelet)

namespace collision_detection {
  float compGauss(float sigma, float x) {
    return exp(-x*x / (2 * sigma * sigma));
  }

  int mod_wrap(long x, int n) {
    while(x < 0)
      x += n;
    return x % n;
  }

  void SensorFilter::onInit() {
    ///////////////////////////////////////////////////
    // constants defined
    //         position   effort   accel   pressuresums pos_errors
    num_sensors = 7 +       7 +      3 +        4 +          7;
    num_scales = 8;
    rate = 100;
    // doog_diffs_1 = {1, 2, 3, 6, 13, 26, 51, 102};
    ///////////////////////////////////////////////////
    
    ///////////////////////////////////////////////////
    // create filters
    sigmas = new float[num_scales];
    float sigma_mult = 0.8;
    float filter_start = 2.0;

    filters = new float*[num_scales];
    // these first values don't extrapolate well
    filters[0] = new float[1];
    filters[0][0] = 1.0;
    sigmas[0] = sigma_mult*1;
    filters[1] = new float[2];
    filters[1][0] = 0.5;
    filters[1][1] = 0.5;
    sigmas[1] = sigma_mult*2;
    filters[2] = new float[4];
    filters[2][0] = 0.18;
    filters[2][1] = 0.32;
    filters[2][2] = 0.32;
    filters[2][3] = 0.18;
    sigmas[2] = sigma_mult*4;

    int filter_len = 8;
    for(int i=3;i<num_scales;i++) {
      filters[i] = new float[filter_len];
      float x = filter_start * filter_len;
      float step = 2 * x / (filter_len - 1);
      float sigma = sigma_mult * filter_len;
      sigmas[i] = sigma;
      x = -x;

      float filter_sum = 0.0;
      // set filter values
      for(int j=0;j<filter_len;j++) {
        filters[i][j] = compGauss(sigma, x);
        filter_sum += filters[i][j];
        x += step;
      }
      // normalize filter
      for(int j=0;j<filter_len;j++) {
        filters[i][j] /= filter_sum;
      }

      buffer_len = filter_len;
      filter_len *= 2;
    }
    ///////////////////////////////////////////////////

    ///////////////////////////////////////////////////
    // instantiate arrays
    buffer = new float*[buffer_len];
    for(int i=0;i<buffer_len;i++) 
      buffer[i] = new float[num_sensors];

    filtered_buffer = new float**[buffer_len];
    for(int i=0;i<buffer_len;i++) {
      filtered_buffer[i] = new float*[num_sensors];
      for(int j=0;j<num_sensors;j++)
        filtered_buffer[i][j] = new float[num_scales];
    }
    ///////////////////////////////////////////////////

    ///////////////////////////////////////////////////
    // initialize variables
    cur_iter = 0;
    js_in = false; acc_in = false; press_in = false; error_in = false;
    ///////////////////////////////////////////////////

    // start loop
    sf_thread = boost::thread(&SensorFilter::startOnline, this);
  }

  void SensorFilter::applyFilter() {
    //double in = ros::Time::now().toSec();
    for(int n=0;n<num_sensors;n++) {
      int filter_len = 1;
      for(int i=0;i<num_scales;i++) {
        filtered_buffer[mod_wrap(cur_iter, buffer_len)][n][i] = 0.0;
        for(int j=0;j<filter_len;j++) {
          filtered_buffer[mod_wrap(cur_iter, buffer_len)][n][i] += buffer[mod_wrap(cur_iter - j, buffer_len)][n] * filters[i][j];
        }
        filter_len *= 2;
      }
    }
    //double out = ros::Time::now().toSec();
    //NODELET_INFO("Filter time: %f", 1000000000.0 * (out-in));
  }

  const int doog_diffs_1[] = {0, 1, 1, 3, 6, 13, 25, 51};
  const int doog_diffs_2[] = {1, 1, 2, 3, 7, 13, 26, 51};


  void SensorFilter::startOnline() {
    ros::NodeHandle nh = getMTNodeHandle();
    ros::NodeHandle nh_priv = getMTPrivateNodeHandle();
    ros::Publisher pub;
    std::string pub_topic, arm_str;
    nh_priv.getParam("arm", arm_str);
    arm_ = arm_str[0];

    if(arm_ == 'r') {
      js_sub = nh.subscribe("joint_states", 1, 
                            &SensorFilter::jsCallback, this);
      acc_sub = nh.subscribe("accelerometer/r_gripper_motor", 1, 
                             &SensorFilter::accCallback, this);
      press_sub = nh.subscribe("pressure/r_gripper_motor", 1, 
                               &SensorFilter::pressCallback, this);
      error_sub = nh.subscribe("r_arm_controller/state", 1, 
                               &SensorFilter::errorCallback, this);

      pub_topic.assign("r_arm_features");
      pub = nh.advertise<pr2_overhead_grasping::SensorPoint>("r_arm_features", 2);
    } else {
      js_sub = nh.subscribe("joint_states", 1, 
                            &SensorFilter::jsCallback, this);
      acc_sub = nh.subscribe("accelerometer/l_gripper_motor", 1, 
                             &SensorFilter::accCallback, this);
      press_sub = nh.subscribe("pressure/l_gripper_motor", 1, 
                               &SensorFilter::pressCallback, this);
      error_sub = nh.subscribe("l_arm_controller/state", 1, 
                               &SensorFilter::errorCallback, this);

      pub_topic.assign("l_arm_features");
      pub = nh.advertise<pr2_overhead_grasping::SensorPoint>("l_arm_features", 2);
    }

    zeroBuffer();

    ros::Rate r(rate);
    int seq = 0;
    int ind, f_ind;
    ROS_INFO("[sensor_filter] Sensor filter is publishing on %s", pub_topic.c_str());
    while(ros::ok()) {
      // Call callbacks to get most recent data
      ros::spinOnce();
      recordData(); // saves data to buffers
      applyFilter(); // applys filter to buffers and saves in filtered_buffer

      // create SensorPoint object
      boost::shared_ptr<pr2_overhead_grasping::SensorPoint> sp(new pr2_overhead_grasping::SensorPoint);
      sp->header.seq = seq;
      sp->header.stamp = ros::Time::now();
      sp->header.frame_id = "";

      ////////////////////////////////////////////////////////////////////////////////
      // Add features:
      // For 8 scales:
      // Raw: n
      // Gaussian: n * 28
      // DOOG: n * 20
      // DOG: n * 20
      // 
      // Total: n * 69
      // If n = 28 then total = 1932
      // Difference features start: 812
      // Raw 0, Gauss 28, DOOG 812, DOG 1372

      int st_ind = 0;
      ind = 0; f_ind = 0;
      // add raw data
      for(int n=0;n<num_sensors;n++)
        sp->raw_data.push_back(buffer[mod_wrap(cur_iter, buffer_len)][n]);
      /*int b_len = 1;
      for(int i=0;i<num_scales;i++) {
        sp->data[ind++] = filters[i][mod_wrap(cur_iter, b_len)];
        b_len *= 2;
      }*/

      //ROS_INFO("RAW FEATURES: %d, ind %d", ind - st_ind, ind);
      st_ind = ind;

      // add gaussian features
      // x = s - 1; x * (x + 1) / 2
      // s = 8; num_features = 28/n
      for(int n=0;n<num_sensors;n++) {
        int center_offset = 1;
        for(int i=1;i<num_scales;i++) { // offset iteration
          int scale_len = 1;
          for(int j=0;j<i;j++) { // length of half of smaller filter
            sp->gaussians.push_back(filtered_buffer[mod_wrap(cur_iter - center_offset + scale_len, buffer_len)][n][i]);
            scale_len *= 2;
          }
          center_offset *= 2;
        }
      }
      //ROS_INFO("GAUSS FEATURES: %d, ind %d", ind - st_ind, ind);
      st_ind = ind;

      /////////////////////////////////////////////////////////////////////////////////
      // Simple bug to fix eventually:
      // These loops:
      // for(int j=1;j<num_scales-2;j++) { 
      // should go to num_scales-1
      // this is not a big deal, it just means that only num_scales-2 features (6)
      // are really being used (2 - 64) lengths
      // note: use better variable names in the future...
      /////////////////////////////////////////////////////////////////////////////////

      // compute DOOG features
      // x = s - 2; x * (x + 1) / 2
      // s = 8; num_features = 21/n
      for(int n=0;n<num_sensors;n++) {
        int scale_len = 1; // length of half of both filters
        for(int j=1;j<num_scales-2;j++) { // scale iteration
          int center_offset = 2;
          for(int i=0;i<num_scales-1-j;i++) { // offset iteration
            sp->features.push_back(filtered_buffer[mod_wrap(cur_iter - center_offset + scale_len + doog_diffs_2[j], buffer_len)][n][j] - filtered_buffer[mod_wrap(cur_iter - center_offset  + scale_len - doog_diffs_1[j], buffer_len)][n][j]);
            //if(n == 0)
            //  ROS_INFO("center %d, scale_len %d, i %d, sum %d, sum2 %d", center_offset, scale_len, j, - center_offset + scale_len + doog_diffs_2[j], - center_offset  + scale_len - doog_diffs_1[j]);
            center_offset *= 2;
          }
          scale_len *= 2;
        }
        sp->split_inds1.push_back(sp->features.size());
        if(n+1 == 3 || n+1 == 10 || n+1 == 17 || n+1 == 21)
          sp->split_inds2.push_back(sp->features.size());
      }
      sp->split_inds1.pop_back();
      sp->split_inds3.push_back(sp->features.size());
      //ROS_INFO("DOOG FEATURES: %d, ind %d", ind - st_ind, ind);
      st_ind = ind;

      // compute DOG features
      // x = s - 2; x * (x + 1) / 2
      // s = 8; num_features = 21/n
      for(int n=0;n<num_sensors;n++) {
        int scale_len = 1; // length of half of smaller filter
        for(int j=1;j<num_scales-2;j++) { // scale iteration
          int center_offset = 2;
          for(int i=0;i<num_scales-1-j;i++) { // offset iteration
            sp->features.push_back(filtered_buffer[mod_wrap(cur_iter - center_offset + scale_len * 2, buffer_len)][n][j+1] - filtered_buffer[mod_wrap(cur_iter - center_offset + scale_len, buffer_len)][n][j]);
            ind++;
            center_offset *= 2;
          }
          scale_len *= 2;
        }
        sp->split_inds1.push_back(sp->features.size());
        if(n == 3 || n == 10 || n == 17 || n == 21)
          sp->split_inds2.push_back(sp->features.size());
      }
      sp->split_inds1.pop_back();
      sp->split_inds3.push_back(sp->features.size());
      //ROS_INFO("DOG FEATURES: %d, ind %d", ind - st_ind, ind);
      st_ind = ind;

      // add gaussian features for error
      // x = s - 1; x * (x + 1) / 2
      // s = 8; num_features = 28/n
      for(int n=21;n<num_sensors;n++) {
        int center_offset = 1;
        for(int i=1;i<num_scales-1;i++) { // offset iteration
          int scale_len = 1;
          for(int j=0;j<i;j++) { // length of half of smaller filter
            sp->features.push_back(filtered_buffer[mod_wrap(cur_iter - center_offset + scale_len, buffer_len)][n][i]);
            ind++;
            scale_len *= 2;
          }
          center_offset *= 2;
        }
        sp->split_inds1.push_back(sp->features.size());
      }
      sp->split_inds1.pop_back();

      //ROS_INFO("%d", ind);
      pub.publish(sp);

      r.sleep();
      cur_iter++;
    }

    ROS_INFO("[sensor_filter] Sensor filter stopping");
  }

  int JOINTSTATE_INDS_R[] = {17, 18, 16, 20, 19, 21, 22};
  int JOINTSTATE_INDS_L[] = {29, 30, 28, 32, 31, 33, 34};

  void SensorFilter::recordData() {
    int ind = 0;
    int* js_inds;
    if(arm_ == 'r')
      js_inds = JOINTSTATE_INDS_R;
    else
      js_inds = JOINTSTATE_INDS_L;
      
    // accelerometer data

    // average out samples
    float avg[3] = {0.0, 0.0, 0.0};
    for(uint32_t j=0;j<acc_msg->samples.size();j++) {
      avg[0] += acc_msg->samples[j].x;
      avg[1] += acc_msg->samples[j].y;
      avg[2] += acc_msg->samples[j].z;
    }

    for(int i=0;i<3;i++) {
      avg[i] /= acc_msg->samples.size();
      buffer[mod_wrap(cur_iter, buffer_len)][ind++] = avg[i];
    }

    // joint angles
    for(int i=0;i<7;i++) 
      buffer[mod_wrap(cur_iter, buffer_len)][ind++] = js_msg->position[js_inds[i]];

    // joint efforts
    for(int i=0;i<7;i++) 
      buffer[mod_wrap(cur_iter, buffer_len)][ind++] = js_msg->effort[js_inds[i]];

    // pressure data
    float r_periph_sum = 0.0, r_pad_sum = 0.0;
    float l_periph_sum = 0.0, l_pad_sum = 0.0;
    // periphery sensor sums (around the edge)
    for(int i=1;i<7;i++) {
      r_periph_sum += press_msg->r_finger_tip[i];
      l_periph_sum += press_msg->l_finger_tip[i];
    }
    // pad sensor sums (in the middle)
    for(int i=7;i<22;i++) {
      r_pad_sum += press_msg->r_finger_tip[i];
      l_pad_sum += press_msg->l_finger_tip[i];
    }
    buffer[mod_wrap(cur_iter, buffer_len)][ind++] = r_periph_sum;
    buffer[mod_wrap(cur_iter, buffer_len)][ind++] = l_periph_sum;
    buffer[mod_wrap(cur_iter, buffer_len)][ind++] = r_pad_sum;
    buffer[mod_wrap(cur_iter, buffer_len)][ind++] = l_pad_sum;

    // controller errors
    for(int i=0;i<7;i++) 
      buffer[mod_wrap(cur_iter, buffer_len)][ind++] = error_msg->error.positions[i];

  }

  void SensorFilter::zeroBuffer() {
    ros::Rate r(200);
    while(ros::ok()) {
      ros::spinOnce();
      if(js_in && acc_in && press_in && error_in)
        break;
      r.sleep();
    }

    for(int i=0;i<buffer_len;i++)
      for(int j=0;j<num_sensors;j++) 
        buffer[i][j] = buffer[cur_iter][j]; // just use initial value for right now
  }

  void SensorFilter::jsCallback(JointState::ConstPtr message) {
    js_in = true;
    js_msg = message;
  }

  void SensorFilter::accCallback(AccelerometerState::ConstPtr message) {
    acc_in = true;
    acc_msg = message;
  }

  void SensorFilter::pressCallback(PressureState::ConstPtr message) {
    press_in = true;
    press_msg = message;
  }

  void SensorFilter::errorCallback(JointTrajectoryControllerState::ConstPtr message) {
    error_in = true;
    error_msg = message;
  }

  SensorFilter::~SensorFilter() {
    sf_thread.interrupt();
  }
}
