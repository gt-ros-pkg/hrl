#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>
#include "opencv2/highgui/highgui.hpp"
#include <string>
#include <vector>

#include <cpl_superpixels/segment/segment.h>
#include <cpl_superpixels/SmoothClutter.h>

using cpl_superpixels::SmoothClutter;
// using cpl_superpixels::getSuperpixelImage;

class SuperpixelSmootherNode
{
 public:
  SuperpixelSmootherNode(ros::NodeHandle n): n_(n), it_(n), sigma_(0.3),
                                             k_(500), min_size_(10)

  {
    ros::NodeHandle n_private("~");
    n_private.param("superpixel_sigma", sigma_, 0.3);
    n_private.param("superpixel_k", k_, 500.0);
    n_private.param("min_patch_size", min_size_, 10);

    image_sub_ = it_.subscribe("image_topic", 1,
                               &SuperpixelSmootherNode::imageCallback, this);
    label_pub_ = it_.advertise("sp_labeled_image", 1);

    // Setup service server
    smoother_srv_ = n_.advertiseService(
        "smooth_clutter", &SuperpixelSmootherNode::smoothClutterService, this);

  }

  cv::Mat smoothClutter(cv::Mat& sp_img, cv::Mat& label_img, int num_regions)
  {
    ROS_INFO("Creating raw label array.");
    // Vector to hold the label results per region index
    std::vector<std::vector<int> > raw_region_labels;
    for (int i = 0; i < num_regions; ++i)
    {
      std::vector<int> labels;
      raw_region_labels.push_back(labels);
    }
    ROS_INFO("Populating raw label array.");
    ROS_INFO_STREAM("Num indexes: " << raw_region_labels.size());

    // Accumulate the pixel labels for each superpixel
    for (int r = 0; r < sp_img.rows; ++r)
    {
      for (int c = 0; c < sp_img.cols; ++c)
      {
        int idx = static_cast<int>(sp_img.at<float>(r,c));
        raw_region_labels[idx].push_back(label_img.at<uchar>(r,c));
      }
    }

    // Perform the majority vote for each region
    const int SURFACE_LABEL_VAL = 120;
    const int CLUTTER_LABEL_VAL = 255;
    const int UNLABELLED_VAL = 0;

    ROS_INFO("Calculating majority votes.");
    std::vector<float> maj_region_labels;
    for(unsigned int i = 0; i < raw_region_labels.size(); ++i)
    {
      std::vector<int> labels = raw_region_labels[i];
      unsigned int unlabelled_count = 0;
      unsigned int clutter_count = 0;
      unsigned int surface_count = 0;

      for (unsigned int j = 0; j < labels.size(); ++j)
      {
        if (labels[j] == SURFACE_LABEL_VAL)
        {
          surface_count++;
        }
        else if (labels[j] == CLUTTER_LABEL_VAL)
        {
          clutter_count++;
        }
        else
        {
          unlabelled_count++;
        }
      }
      // TODO: Maybe change how we deal with unlabelled stuff
      if (unlabelled_count > clutter_count &&
          unlabelled_count > surface_count)
      {
        maj_region_labels.push_back(UNLABELLED_VAL);
      }
      else if ( clutter_count >= surface_count)
      {
        maj_region_labels.push_back(CLUTTER_LABEL_VAL);
      }
      else
      {
        maj_region_labels.push_back(SURFACE_LABEL_VAL);
      }
    }

    // Propogate superpixel majority vote values to all members of the region
    ROS_INFO("Propogatting results.");
    cv::Mat smooth_labels(label_img.size(), label_img.type());
    for (int r = 0; r < sp_img.rows; ++r)
    {
      for (int c = 0; c < sp_img.cols; ++c)
      {
        int idx = static_cast<int>(sp_img.at<float>(r,c));
        smooth_labels.at<uchar>(r,c) = maj_region_labels[idx];
      }
    }
    return smooth_labels;
  }

  //
  // ROS Stuff
  //

  bool smoothClutterService(SmoothClutter::Request &req,
                            SmoothClutter::Response &res)
  {
    sensor_msgs::ImageConstPtr raw_img_msg =
        boost::shared_ptr<sensor_msgs::Image const>(
            new sensor_msgs::Image(req.raw_img));
    sensor_msgs::ImageConstPtr label_img_msg =
        boost::shared_ptr<sensor_msgs::Image const>(
            new sensor_msgs::Image(req.clutter_labels));
    cv::Mat raw_img = bridge_.imgMsgToCv(raw_img_msg);
    cv::Mat label_img = bridge_.imgMsgToCv(label_img_msg);
    int num_regions = 0;
    cv::Mat sp_img = cpl_superpixels::getSuperpixelImage(raw_img, num_regions,
                                                         sigma_, k_, min_size_);
    ROS_INFO("Performing majority vote.");
    cv::Mat smooth_labels = smoothClutter(sp_img, label_img, num_regions);

    ROS_INFO("Converting result for transport.");
    IplImage smooth_labels_ipl = smooth_labels;
    sensor_msgs::Image::ConstPtr label_out = bridge_.cvToImgMsg(&smooth_labels_ipl);
    res.smooth_clutter_labels = *label_out;
    return true;
  }

  /**
   * Method to take an image comping in on the image_sub_ topic and publishes
   * the superpixel label version of the image over the topic label_pub_
   *
   * @param msg_ptr Input from the image_sub_ subscriber
   */
  void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
  {
    cv::Mat input_img;
    input_img = bridge_.imgMsgToCv(msg_ptr);

    int num_regions = 0;
    cv::Mat label_img = cpl_superpixels::getSuperpixelImage(input_img,
                                                            num_regions, sigma_,
                                                            k_, min_size_);

    // Publish the label data
    sensor_msgs::ImageConstPtr label_msg;
    IplImage label_ipl = label_img;
    label_msg = bridge_.cvToImgMsg(&label_ipl);
    label_pub_.publish(label_msg);
  }

  /**
   * Simple method to keep the node running.
   */
  void spin()
  {
    while(n_.ok())
    {
      ros::spinOnce();
    }
  }

 protected:
  ros::NodeHandle n_;
  ros::ServiceServer smoother_srv_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher label_pub_;
  sensor_msgs::CvBridge bridge_;
  double sigma_;
  double k_;
  int min_size_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "");
  ros::NodeHandle n;
  SuperpixelSmootherNode smoother_node(n);
  smoother_node.spin();
}
