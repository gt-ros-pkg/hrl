#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>

// TF
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

// PCL
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/feature.h>
#include <pcl/common/eigen.h>
#include <pcl/io/io.h>
#include "pcl/filters/voxel_grid.h"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

// Visual features
#include <cpl_visual_features/sliding_window.h>
#include <cpl_visual_features/features/hsv_color_histogram.h>
#include <cpl_visual_features/features/attribute_learning_base_feature.h>

// Superpixels
#include <cpl_superpixels/segment/segment.h>

// tabletop_pushing
#include <tabletop_pushing/PushPose.h>
#include <tabletop_pushing/LocateTable.h>
#include <tabletop_pushing/ObjectSingulationAction.h>

// STL
#include <vector>
#include <set>
#include <map>
#include <queue>
#include <string>
#include <utility>
#include <math.h>

#define DISPLAY_SUPERPIXELS 1
// #define DISPLAY_TRACKER_OUTPUT 1
#define DISPLAY_MOVING_STUFF 1
// #define DISPLAY_IMAGE_DIFFERENCING 1
// #define DISPLAY_ELLIPSE_STUFF 1

using cpl_superpixels::getSuperpixelImage;
using tabletop_pushing::PushPose;
using tabletop_pushing::LocateTable;
using geometry_msgs::PoseStamped;
typedef pcl::PointCloud<pcl::PointXYZ> XYZPointCloud;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image,
                                                        sensor_msgs::PointCloud2> MySyncPolicy;

typedef std::vector<float> Descriptor;

const cv::RotatedRect EMPTY_ELLIPSE(cv::Point2f(0.0f, 0.0f),
                                    cv::Size2f(0.0f, 0.0f),
                                    0.0f);

bool operator==(cv::RotatedRect a, cv::RotatedRect b)
{
  return (a.center.x == b.center.x && a.center.y == b.center.y &&
          a.size.width == b.size.width && a.size.height == b.size.height);
}

bool operator!=(cv::RotatedRect a, cv::RotatedRect b)
{
  return !(a == b);
}

struct Flow
{
  Flow(int _x, int _y, int _dx, int _dy) : x(_x), y(_y), dx(_dx), dy(_dy)
  {
  }
  int x, y, dx, dy;
};

typedef std::pair<uchar, Flow> RegionMember;

class ProbImageDifferencing
{
 public:
  ProbImageDifferencing(unsigned int num_hist_frames=5, float T_in=3,
                        float T_out=15) :
      num_hist_frames_(num_hist_frames), ONES(1.0, 1.0, 1.0, 1.0),
      T_in_(T_in), T_out_(T_out)
  {
    g_kernel_ = cv::getGaussianKernel(3, 2.0, CV_32F);
  }

  void initialize(cv::Mat& color_frame, cv::Mat& depth_frame)
  {
    // Restart the motion history
    motion_probs_.create(color_frame.size(), CV_32FC4);
    d_motion_probs_.create(depth_frame.size(), CV_32FC1);

    // Restart the queue
    pixel_histories_.clear();
    d_histories_.clear();
    update(color_frame, depth_frame);
  }

  // std::vector<cv::Mat> update(cv::Mat& bgrd_frame)
  std::vector<cv::Mat> update(cv::Mat& color_frame, cv::Mat& depth_frame_pre_blur)
  {
    // Convert to correct format
    cv::Mat bgr_frame_pre_blur(color_frame.size(), CV_32FC3);
    color_frame.convertTo(bgr_frame_pre_blur, CV_32FC3, 1.0/255, 0);
    cv::Mat bgr_frame(color_frame.size(), CV_32FC3);
    cv::Mat depth_frame(color_frame.size(), CV_32FC3);
    // Smooth images before using
    cv::filter2D(bgr_frame_pre_blur, bgr_frame, CV_32F, g_kernel_);
    cv::filter2D(depth_frame_pre_blur, depth_frame, CV_32F, g_kernel_);

    // Calculate probability of being the same color and depth at each pixel
    std::vector<cv::Mat> motions;
    if (pixel_histories_.size() > 0)
      motions = calculateProbabilities(bgr_frame, depth_frame);

    // Store current frame in history
    pixel_histories_.push_back(bgr_frame);
    d_histories_.push_back(depth_frame);

    // Pop the front of the queue if we have too many frames
    if (pixel_histories_.size() > num_hist_frames_)
    {
      pixel_histories_.pop_front();
      d_histories_.pop_front();
    }

    return motions;
  }

  std::vector<cv::Mat> calculateProbabilities(cv::Mat& bgr_frame,
                                              cv::Mat& depth_frame)
  {
    // Update Gaussian estimates at each pixel
    // Calculate means
    cv::Mat means(bgr_frame.size(), CV_32FC3, cv::Scalar(0.0,0.0,0.0));
    cv::Mat d_means(depth_frame.size(), CV_32FC1, cv::Scalar(0.0));
    for (unsigned int i = 0; i < pixel_histories_.size(); ++i)
    {
      for (int r = 0; r < means.rows; ++r)
      {
        for (int c = 0; c < means.cols; ++c)
        {
          means.at<cv::Vec3f>(r,c) += pixel_histories_[i].at<cv::Vec3f>(r,c);
          d_means.at<float>(r,c) += d_histories_[i].at<float>(r,c);
        }
      }
    }
    means /= pixel_histories_.size();
    d_means /= d_histories_.size();

#ifdef DISPLAY_MEANS
    std::vector<cv::Mat> means_vec;
    cv::split(means, means_vec);

    cv::imshow("means_h", means_vec[0]);
    cv::imshow("means_s", means_vec[1]);
    cv::imshow("means_v", means_vec[2]);
    cv::imshow("d_means", d_means);
#endif // DISPLAY_MEANS

    // Calculate variances
    cv::Mat vars(bgr_frame.size(), CV_32FC3, cv::Scalar(0.0,0.0,0.0));
    cv::Mat d_vars(depth_frame.size(), CV_32FC1, cv::Scalar(0.0));
    for (unsigned int i = 0; i < pixel_histories_.size(); ++i)
    {
      for (int r = 0; r < means.rows; ++r)
      {
        for (int c = 0; c < means.cols; ++c)
        {
          cv::Vec3f diff = pixel_histories_[i].at<cv::Vec3f>(r,c) -
              means.at<cv::Vec3f>(r,c);

          vars.at<cv::Vec3f>(r,c) += diff.mul(diff);

          float d_diff = d_histories_[i].at<float>(r,c)-d_means.at<float>(r,c);
          d_vars.at<float>(r,c) += d_diff * d_diff;
        }
      }
    }
    if (pixel_histories_.size() > 1)
    {
      vars /= (pixel_histories_.size()-1.0);
      d_vars /= (d_histories_.size()-1.0);
    }
    else
    {
      // pass
    }

#ifdef DISPLAY_VARS
    std::vector<cv::Mat> vars_vec;
    cv::split(vars, vars_vec);
    cv::imshow("vars_h", vars_vec[0]);
    cv::imshow("vars_s", vars_vec[1]);
    cv::imshow("vars_v", vars_vec[2]);
    cv::imshow("d_vars", d_vars);
#endif // DISPLAY_VARS

    int d_probs_greater = 0;
    int d_probs_lesser = 0;
    // Calculate probability of pixels having moved
    for (int r = 0; r < motion_probs_.rows; ++r)
    {
      for (int c = 0; c < motion_probs_.cols; ++c)
      {
        cv::Vec3f x = bgr_frame.at<cv::Vec3f>(r,c);
        cv::Vec3f mu = means.at<cv::Vec3f>(r,c);
        cv::Vec3f var = vars.at<cv::Vec3f>(r,c);
        // NOTE: Probability of not belonging to the gaussian
        // cv::Vec4f probs = ONES - p_x_gaussian(x, mu, var);
        cv::Vec4f probs = p_x_fitzpatrick(x, mu, var);
        motion_probs_.at<cv::Vec4f>(r,c) = probs;
        float x_d = depth_frame.at<float>(r,c);
        float mu_d = d_means.at<float>(r,c);
        float var_d = d_vars.at<float>(r,c);
        // float prob_d = 1.0 - p_x_gaussian_kenney(x_d, mu_d, var_d);
        float prob_d = p_x_fitzpatrick(x_d, mu_d, var_d);
        if (prob_d > 1.0)
        {
          d_probs_greater++;
        }
        else if (prob_d < 0.0)
        {
          d_probs_lesser++;
        }
        d_motion_probs_.at<float>(r,c) = prob_d;
        // motion_probs_.at<cv::Vec4f>(r,c)[3]*= prob_d;
        motion_probs_.at<cv::Vec4f>(r,c)[3]= std::max(
            prob_d, motion_probs_.at<cv::Vec4f>(r,c)[3]);
      }
    }

    // TODO: Merge these into a single image?
    std::vector<cv::Mat> motions;
    motions.push_back(motion_probs_);
    motions.push_back(d_motion_probs_);
    return motions;
  }

  cv::Vec4f p_x_gaussian(cv::Vec3f x, cv::Vec3f mu, cv::Vec3f var)
  {
    cv::Vec4f p_x;
    p_x[0] = p_x_gaussian_kenney(x[0], mu[0], var[0]);
    p_x[1] = p_x_gaussian_kenney(x[1], mu[1], var[1]);
    p_x[2] = p_x_gaussian_kenney(x[2], mu[2], var[2]);
    p_x[3] = p_x[0]*p_x[1]*p_x[2];
    // p_x[3] = max(p_x[0],max(p_x[1],p_x[2]));
    return p_x;
  }

  cv::Vec4f p_x_fitzpatrick(cv::Vec3f x, cv::Vec3f mu, cv::Vec3f var)
  {
    cv::Vec4f p_x;
    p_x[0] = p_x_fitzpatrick(x[0], mu[0], var[0]);
    p_x[1] = p_x_fitzpatrick(x[1], mu[1], var[1]);
    p_x[2] = p_x_fitzpatrick(x[2], mu[2], var[2]);
    p_x[3] = p_x[0]*p_x[1]*p_x[2];
    // p_x[3] = max(p_x[0],max(p_x[1],p_x[2]));
    return p_x;
  }

  inline float p_x_fitzpatrick(const float x, const float mu, float var)
  {
    // float new_var = 5.0;
    if (isnan(var))
    {
      var = 0.1;
    }
    return fabs(x-mu)/(var+0.1);
  }

  float p_x_gaussian_kenney(float x, float mu, float var)
  {
    float mean_diff = abs(x-mu);
    float sigma = sqrt(var);
    float s0 = T_in_*sigma;
    float s1 = T_out_*sigma;

    if (mean_diff <= s0)
    {
      // Don't make things impossible (i.e. depth does not change when things
      // are too close)
      return 0.99;
      // return 1.0;
    }
    if (mean_diff >= s1)
    {
      return 0.01;
    }
    float m = 1.0/(s0-s1);
    return m*(mean_diff-s1);
  }


  float p_x_gaussian_hist(float x, float mu, float var)
  {
    // TODO: Figure out a better way of approximating width
    float x1 = x-0.05;
    float x2 = x+0.05;
    float p_x = (abs(gaussian_pdf(x2, mu, var) - gaussian_pdf(x1, mu, var))*
                 abs(x2-x1));
    return p_x;
  }

  float gaussian_pdf(float x, float mu, float var)
  {
    if (var == 0.0)
    {
      // Don't make things impossible
      if (x == mu)
      {
        return 0.99;
        // return 1.0;
      }
      else
        return 0.01;
    }
    float mean_diff = x-mu;
    return exp(-(mean_diff*mean_diff)/(2.0*var))/(sqrt(2.0*var*M_PI));
  }

  void setNumHistFrames(unsigned int num_hist_frames)
  {
    num_hist_frames_ = num_hist_frames;
  }
  void setT_inT_out(float T_in, float T_out)
  {
    T_in_ = T_in;
    T_out_ = T_out;
  }

 protected:
  cv::Mat motion_probs_;
  cv::Mat d_motion_probs_;
  std::deque<cv::Mat> pixel_histories_;
  std::deque<cv::Mat> d_histories_;
  unsigned int num_hist_frames_;
  const cv::Vec4f ONES;
  float T_in_;
  float T_out_;
  cv::Mat g_kernel_;
};

class FeatureTracker
{
 public:
  FeatureTracker(std::string name, double hessian_thresh=250, int num_octaves=4,
                 int num_layers=2, bool extended=true) :
      surf_(hessian_thresh, num_octaves, num_layers, extended),
      initialized_(false), ratio_threshold_(0.5), window_name_(name),
      min_flow_thresh_(0), max_corners_(500), klt_corner_thresh_(0.3),
      klt_corner_min_dist_(2)
  {
    prev_keypoints_.clear();
    cur_keypoints_.clear();
    prev_descriptors_.clear();
    cur_descriptors_.clear();
  }

  //
  // Main tracking logic functions
  //

  void initTracks(cv::Mat& frame)
  {
    updateCurrentDescriptors(frame);
    prev_keypoints_ = cur_keypoints_;
    prev_descriptors_ = cur_descriptors_;
    initialized_ = true;
  }

  std::vector<Flow> updateTracksKLT(cv::Mat& cur_frame, cv::Mat& prev_frame)
  {
    std::vector<Flow> sparse_flow;
    std::vector<cv::Point2f> prev_points;
    std::vector<cv::Point2f> new_points;
    ROS_INFO_STREAM("max_corners: " << max_corners_);
    ROS_INFO_STREAM("quality_level: " << klt_corner_thresh_);
    ROS_INFO_STREAM("min_distance: " << klt_corner_min_dist_);
    cv::goodFeaturesToTrack(prev_frame, prev_points, max_corners_,
                            klt_corner_thresh_, klt_corner_min_dist_);
    ROS_INFO_STREAM("Found " << prev_points.size() << " corners.");
    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(prev_frame, cur_frame, prev_points, new_points,
                             status, err);
    int moving_points = 0;
    for (unsigned int i = 0; i < prev_points.size(); i++)
    {
      if (! status[i]) continue;
      int dx = prev_points[i].x - new_points[i].x;
      int dy = prev_points[i].y - new_points[i].y;
      sparse_flow.push_back(Flow(new_points[i].x, new_points[i].y, dx, dy));
      if (sparse_flow[i].dx + sparse_flow[i].dy > min_flow_thresh_)
        moving_points++;
    }
    ROS_INFO_STREAM(window_name_ << ": num moving points: " << moving_points);

#ifdef DISPLAY_TRACKER_OUTPUT
    cv::Mat display_cur_frame(cur_frame.rows, cur_frame.cols, CV_8UC3);;
    cv::cvtColor(cur_frame, display_cur_frame, CV_GRAY2BGR);
    for (unsigned int i = 0; i < sparse_flow.size(); i++)
    {
      if (sparse_flow[i].dx + sparse_flow[i].dy > min_flow_thresh_)
      {
        ROS_DEBUG_STREAM("Point is moving (" << sparse_flow[i].dx << ", "
                         << sparse_flow[i].dy << ")");
        cv::line(display_cur_frame,
                 cv::Point(sparse_flow[i].x, sparse_flow[i].y),
                 cv::Point(sparse_flow[i].x + sparse_flow[i].dx,
                           sparse_flow[i].y + sparse_flow[i].dy),
                 cv::Scalar(0,0,255), 1);
      }
    }

    cv::imshow(window_name_, display_cur_frame);
#endif // DISPLAY_TRACKER_OUTPUT
    return sparse_flow;
  }

  std::vector<Flow> updateTracks(cv::Mat& frame)
  {
    cur_keypoints_.clear();
    cur_descriptors_.clear();
    updateCurrentDescriptors(frame);

    std::vector<int> matches_cur;
    std::vector<int> matches_prev;
    std::vector<Flow> sparse_flow;
    matches_cur.clear();
    matches_prev.clear();

    // Find nearest neighbors with previous descriptors
    findMatches(cur_descriptors_, prev_descriptors_, matches_cur, matches_prev);
    ROS_DEBUG_STREAM(window_name_ << ": num feature matches: "
                     << matches_cur.size());
    int moving_points = 0;
    for (unsigned int i = 0; i < matches_cur.size(); i++)
    {
      int dx = prev_keypoints_[matches_prev[i]].pt.x -
          cur_keypoints_[matches_cur[i]].pt.x;
      int dy = prev_keypoints_[matches_prev[i]].pt.y -
          cur_keypoints_[matches_cur[i]].pt.y;
      sparse_flow.push_back(Flow(cur_keypoints_[matches_cur[i]].pt.x,
                                 cur_keypoints_[matches_cur[i]].pt.y,
                                 dx, dy));
      if (sparse_flow[i].dx + sparse_flow[i].dy > min_flow_thresh_)
        moving_points++;
    }
    ROS_DEBUG_STREAM(window_name_ << ": num moving points: " << moving_points);

#ifdef DISPLAY_TRACKER_OUTPUT
    cv::Mat display_frame(frame.rows, frame.cols, CV_8UC3);;
    cv::cvtColor(frame, display_frame, CV_GRAY2BGR);
    for (unsigned int i = 0; i < matches_cur.size(); i++)
    {
      if (sparse_flow[i].dx + sparse_flow[i].dy > min_flow_thresh_)
      {
        ROS_DEBUG_STREAM("Point is moving (" << sparse_flow[i].dx << ", "
                         << sparse_flow[i].dy << ")");
        cv::line(display_frame,
                 prev_keypoints_[matches_prev[i]].pt,
                 cur_keypoints_[matches_cur[i]].pt,
                 cv::Scalar(0,0,255), 1);
      }
    }

    cv::imshow(window_name_, display_frame);
#endif // DISPLAY_TRACKER_OUTPUT

    prev_keypoints_ = cur_keypoints_;
    prev_descriptors_ = cur_descriptors_;
    return sparse_flow;
  }

  //
  // Feature Matching Functions
  //

  /*
   * SSD
   *
   * @short Computes the squareroot of squared differences
   * @param a First descriptor
   * @param b second descriptor
   * @return value of squareroot of squared differences
   */
  double SSD(Descriptor& a, Descriptor& b)
  {
    double diff = 0;

    for (unsigned int i = 0; i < a.size(); ++i) {
      float delta = a[i] - b[i];
      diff += delta*delta;
    }

    return diff;
  }

  /*
   * ratioTest
   *
   * @short Computes the  ratio test described in Lowe 2004
   * @param a Descriptor from the first image to compare
   * @param bList List of descriptors from the second image
   * @param threshold Threshold value for ratioTest comparison
   *
   * @return index of the best match, -1 if no match ratio is less than threshold
   */
  int ratioTest(Descriptor& a, std::vector<Descriptor>& bList, double threshold)
  {
    double bestScore = 1000000;
    double secondBest = 1000000;
    int bestIndex = -1;

    for (unsigned int b = 0; b < bList.size(); ++b) {
      double score = 0;
      score = SSD(a, bList[b]);

      if (score < bestScore) {
        secondBest = bestScore;
        bestScore = score;
        bestIndex = b;
      } else if (score < secondBest) {
        secondBest = score;
      }
      if ( bestScore / secondBest > threshold) {
        bestIndex = -1;
      }

    }

    return bestIndex;
  }

  /**
   * findMatches
   *
   * @param descriptors1 List of descriptors from image 1
   * @param descriptors2 List of descriptors from image 2
   * @param matches1 Indexes of matching points in image 1 (Returned)
   * @param matches2 Indexes of matching points in image 2 (Returned)
   */
  void findMatches(std::vector<Descriptor>& descriptors1,
                   std::vector<Descriptor>& descriptors2,
                   std::vector<int>& matches1, std::vector<int>& matches2)
  {
    // Determine matches using the Ratio Test method from Lowe 2004
    for (unsigned int a = 0; a < descriptors1.size(); ++a) {
      const int bestIndex = ratioTest(descriptors1[a], descriptors2,
                                      ratio_threshold_);
      if (bestIndex != -1) {
        matches1.push_back(a);
        matches2.push_back(bestIndex);
      }
    }

    // Check that the matches are unique going the other direction
    for (unsigned int x = 0; x < matches2.size();) {
      const int bestIndex = ratioTest(descriptors2[matches2[x]],
                                      descriptors1, ratio_threshold_);
      if (bestIndex != matches1[x]) {
        matches1.erase(matches1.begin()+x);
        matches2.erase(matches2.begin()+x);
      } else {
        x++;
      }
    }

  }

 protected:

  //
  // Helper Functions
  //

  void updateCurrentDescriptors(cv::Mat& frame)
  {
    std::vector<float> raw_descriptors;
    try
    {
      surf_(frame, cv::Mat(), cur_keypoints_, raw_descriptors);
      for (unsigned int i = 0; i < raw_descriptors.size(); i += 128)
      {
        Descriptor d(raw_descriptors.begin() + i,
                     raw_descriptors.begin() + i + 128);
        cur_descriptors_.push_back(d);
      }
    }
    catch(cv::Exception e)
    {
      std::cerr << e.err << std::endl;
    }
  }

 public:

  //
  // Getters & Setters
  //

  bool isInitialized() const
  {
    return initialized_;
  }

  void setMinFlowThresh(int min_thresh)
  {
    min_flow_thresh_= min_thresh;
  }

  void setKLTCornerThresh(double corner_thresh)
  {
    klt_corner_thresh_ = corner_thresh;
  }

  void setKLTCornerMinDist(double min_dist)
  {
    klt_corner_min_dist_ = min_dist;
  }

  void setKLTMaxCorners(int max_corners)
  {
    max_corners_ = max_corners;
  }

  void stop()
  {
    initialized_ = false;
  }

 protected:
  std::vector<cv::KeyPoint> prev_keypoints_;
  std::vector<cv::KeyPoint> cur_keypoints_;
  std::vector<Descriptor> prev_descriptors_;
  std::vector<Descriptor> cur_descriptors_;
  cv::SURF surf_;
  bool initialized_;
  double ratio_threshold_;
  std::string window_name_;
  int min_flow_thresh_;
  int max_corners_;
  double klt_corner_thresh_;
  double klt_corner_min_dist_;
};

class PR2ArmDetector
{
 public:
  PR2ArmDetector(int num_bins = 32) :
      have_hist_model_(false), theta_(0.5), num_bins_(num_bins),
      bin_size_(256/num_bins), laplace_denom_(num_bins_*num_bins_*num_bins_)
  {
    int sizes[] = {num_bins, num_bins, num_bins};
    fg_hist_.create(3, sizes, CV_32FC1);
    bg_hist_.create(3, sizes, CV_32FC1);
  }

  void setHISTColorModel(cv::Mat fg_hist, cv::Mat bg_hist, double theta)
  {
    have_hist_model_ = true;
    fg_hist_ = fg_hist;
    bg_hist_ = bg_hist;
    theta_ = theta;
  }

  float pixelArmProbability(cv::Vec3b pixel_color)
  {
    if (not have_hist_model_) return 0.0;
    cv::Vec3i bins = getBinNumbers(pixel_color);
    float p_arm = log((fg_hist_.at<float>(bins[0], bins[1], bins[2])+1) /
                      (fg_count_+laplace_denom_));
    float p_bg = log((bg_hist_.at<float>(bins[0], bins[1], bins[2])+1) /
                     (bg_count_+laplace_denom_));
    float prob = exp(p_arm - p_bg);
    return prob;
  }

  inline const cv::Vec3i getBinNumbers(cv::Vec3b pixel_color) const
  {
    const cv::Vec3i bins(pixel_color[0]/bin_size_,
                         pixel_color[1]/bin_size_,
                         pixel_color[2]/bin_size_);
    return bins;
  }

  /**
   * Class to use color model to detect pr2 arms.
   * Assumes input image is CV_8UC3 HSV image
   */
  cv::Mat imageArmProbability(cv::Mat& arm_img)
  {
    cv::Mat prob_img(arm_img.size(), CV_32FC1);
    for(int r = 0; r < arm_img.rows; ++r)
    {
      cv::Vec3b* arm_row = arm_img.ptr<cv::Vec3b>(r);
      float* prob_row = prob_img.ptr<float>(r);
      for (int c = 0; c < arm_img.cols; ++c)
      {
        float pixel_ratio = pixelArmProbability(arm_row[c]);
        if (pixel_ratio >= theta_)
        {
          prob_row[c] = 1.0;
          // ROS_INFO_STREAM("pixel_ratio_: " << pixel_ratio);
        }
        else
        {
          prob_row[c] = 0.0;
        }
      }
    }
    return prob_img;
  }

  void learnColorModels(std::string directory_path, int count,
                        std::string save_path)
  {
    ROS_INFO_STREAM("Learning color models.");
    // Empty histograms before starting
    for (int i = 0; i < num_bins_; ++i)
    {
      for (int j = 0; j < num_bins_; ++j)
      {
        for (int k = 0; k < num_bins_; ++k)
        {
          fg_hist_.at<float>(i,j,k) = 0;
          bg_hist_.at<float>(i,j,k) = 0;
        }
      }
    }
    fg_count_ = 0;
    bg_count_ = 0;

    // Add pixels form each of the things
    for (int i = 0; i < count; ++i)
    {
      std::stringstream img_name;
      img_name << directory_path << i << ".tiff";
      std::stringstream mask_name;
      mask_name << directory_path << i << "_mask.tiff";
      cv::Mat img_bgr = cv::imread(img_name.str());
      cv::Mat img;
      cv::cvtColor(img_bgr, img, CV_BGR2HSV);
      cv::Mat mask = cv::imread(mask_name.str(), 0);
      // Extract fg and bg regions and add to histogram
      for (int r = 0; r < img.rows; ++r)
      {
        cv::Vec3b* img_row = img.ptr<cv::Vec3b>(r);
        uchar* mask_row = mask.ptr<uchar>(r);
        for (int c = 0; c < img.cols; ++c)
        {
          if (mask_row[c] == 255)
          {
            cv::Vec3i bins = getBinNumbers(img_row[c]);
            fg_hist_.at<float>(bins[0], bins[1], bins[2]) += 1;
            fg_count_++;
          }
          else if (mask_row[c] == 0)
          {
            cv::Vec3i bins = getBinNumbers(img_row[c]);
            bg_hist_.at<float>(bins[0], bins[1], bins[2]) += 1;
            bg_count_++;
          }
        }
      }
    }
    saveColorModels(save_path);
  }

  void saveColorModels(std::string output_path)
  {
    std::ofstream data_out(output_path.c_str());
    // Write out header infos
    data_out << fg_count_ << std::endl;
    data_out << bg_count_ << std::endl;
    data_out << num_bins_ << std::endl;
    data_out << theta_ << std::endl;
    // Write out the histograms
    for (int i = 0; i < num_bins_; ++i)
    {
      for (int j = 0; j < num_bins_; ++j)
      {
        for (int k = 0; k < num_bins_; ++k)
        {
          data_out << fg_hist_.at<float>(i,j,k) << " ";
        }
        data_out << std::endl;
      }
    }
    data_out << std::endl;
    for (int i = 0; i < num_bins_; ++i)
    {
      for (int j = 0; j < num_bins_; ++j)
      {
        for (int k = 0; k < num_bins_; ++k)
        {
          data_out << bg_hist_.at<float>(i,j,k) << " ";
        }
        data_out << std::endl;
      }
    }
    data_out.close();
  }

  void loadColorModels(std::string input_path)
  {
    std::ifstream data_in(input_path.c_str());
    // Read in header infos
    data_in >> fg_count_;
    data_in >> bg_count_;
    data_in >> num_bins_;
    data_in >> theta_;
    laplace_denom_ = num_bins_*num_bins_*num_bins_;
    // Read in the histograms
    for (int i = 0; i < num_bins_; ++i)
    {
      for (int j = 0; j < num_bins_; ++j)
      {
        for (int k = 0; k < num_bins_; ++k)
        {
          data_in >> fg_hist_.at<float>(i,j,k);
        }
      }
    }
    for (int i = 0; i < num_bins_; ++i)
    {
      for (int j = 0; j < num_bins_; ++j)
      {
        for (int k = 0; k < num_bins_; ++k)
        {
          data_in >> bg_hist_.at<float>(i,j,k);
        }
      }
    }
    data_in.close();
    have_hist_model_ = true;
    // float prior = static_cast<float>(fg_count_) / static_cast<float>(bg_count_);
  }

  bool have_hist_model_;
  cv::Mat fg_hist_;
  cv::Mat bg_hist_;
  double theta_;
  int fg_count_;
  int bg_count_;
  int num_bins_;
  int bin_size_;
  int laplace_denom_;
};

class RegionTrackingNode
{
 public:
  RegionTrackingNode(ros::NodeHandle &n) :
      n_(n),
      image_sub_(n, "color_image_topic", 1),
      depth_sub_(n, "depth_image_topic", 1),
      cloud_sub_(n, "point_cloud_topic", 1),
      sync_(MySyncPolicy(1), image_sub_, depth_sub_, cloud_sub_),
      track_server_(n, "region_track_action",
                    boost::bind(&RegionTrackingNode::startTracker,
                                this, _1),
                    false),
      tf_(),
      tracker_("i_tracker"),
      have_depth_data_(false), min_flow_thresh_(0),
      num_region_points_thresh_(1), tracking_(true)
  {
    // Get parameters from the server
    ros::NodeHandle n_private("~");
    n_private.param("segment_k", k_, 500.0);
    n_private.param("segment_sigma", sigma_, 0.9);
    n_private.param("segment_min_size", min_size_, 30);
    n_private.param("segment_r_weight", wr_, 0.1);
    n_private.param("segment_g_weight", wg_, 0.1);
    n_private.param("segment_b_weight", wb_, 0.1);
    n_private.param("segment_depth_weight", wd_, 0.7);
    n_private.param("min_flow_thresh", min_flow_thresh_, 0);
    n_private.param("num_moving_points_per_region_thresh",
                    num_region_points_thresh_, 1);
    n_private.param("crop_min_x", crop_min_x_, 0);
    n_private.param("crop_max_x", crop_max_x_, 640);
    n_private.param("crop_min_y", crop_min_y_, 0);
    n_private.param("crop_max_y", crop_max_y_, 480);
    n_private.param("display_wait_ms", display_wait_ms_, 3);
    n_private.param("min_workspace_x", min_workspace_x_, 0.0);
    n_private.param("min_workspace_y", min_workspace_y_, 0.0);
    n_private.param("min_workspace_z", min_workspace_z_, 0.0);
    n_private.param("max_workspace_x", max_workspace_x_, 0.0);
    n_private.param("max_workspace_y", max_workspace_y_, 0.0);
    n_private.param("max_workspace_z", max_workspace_z_, 0.0);
    std::string default_workspace_frame = "/torso_lift_link";
    n_private.param("workspace_frame", workspace_frame_,
                    default_workspace_frame);
    n_private.param("min_table_z", min_table_z_, -0.5);
    n_private.param("max_table_z", max_table_z_, 1.5);
    int max_corners;
    n_private.param("klt_max_corners", max_corners, 500);
    tracker_.setKLTMaxCorners(max_corners);
    double klt_corner_thresh;
    n_private.param("klt_corner_thresh", klt_corner_thresh, 0.3);
    tracker_.setKLTCornerThresh(klt_corner_thresh);
    double klt_min_dist;
    n_private.param("klt_corner_min_dist", klt_min_dist, 2.0);
    tracker_.setKLTCornerMinDist(klt_min_dist);
    n_private.param("intensity_diff_thresh", intensity_diff_thresh_, 50);
    n_private.param("depth_diff_thresh", depth_diff_thresh_, 0.05);

    // Setup internal class stuff
    tracker_.setMinFlowThresh(min_flow_thresh_);

    // Setup ros node connections
    sync_.registerCallback(&RegionTrackingNode::sensorCallback,
                           this);
    push_pose_server_ = n_.advertiseService(
        "get_push_pose", &RegionTrackingNode::getPushPose, this);
    table_location_server_ = n_.advertiseService(
        "get_table_location", &RegionTrackingNode::getTableLocation,
        this);
  }

  void sensorCallback(const sensor_msgs::ImageConstPtr& img_msg,
                      const sensor_msgs::ImageConstPtr& depth_msg,
                      const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    // Convert images to OpenCV format
    cv::Mat color_frame(bridge_.imgMsgToCv(img_msg));
    cv::Mat depth_frame(bridge_.imgMsgToCv(depth_msg));

    // Swap kinect color channel order
    cv::cvtColor(color_frame, color_frame, CV_RGB2BGR);

    // Transform point cloud into the correct frame and convert to PCL struct
    XYZPointCloud cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);
    pcl_ros::transformPointCloud(workspace_frame_, cloud, cloud, tf_);

    // Convert nans to zeros
    for (int r = 0; r < depth_frame.rows; ++r)
    {
      for (int c = 0; c < depth_frame.cols; ++c)
      {
        float cur_d = depth_frame.at<float>(r,c);
        if (isnan(cur_d))
          depth_frame.at<float>(r,c) = 0.0;
      }
    }

    // Select inner ROI of images to remove border issues
    int x_size = crop_max_x_ - crop_min_x_;
    int y_size = crop_max_y_ - crop_min_y_;
    cv::Rect roi(crop_min_x_, crop_min_y_, x_size, y_size);

    cv::Mat depth_region = depth_frame(roi);
    cv::Mat color_region = color_frame(roi);

    // Save internally for use in the service callback
    prev_color_frame_ = cur_color_frame_.clone();
    prev_depth_frame_ = cur_depth_frame_.clone();
    cur_color_frame_ = color_region.clone();
    cur_depth_frame_ = depth_region.clone();
    cur_point_cloud_ = cloud;
    have_depth_data_ = true;

    // Started via actionlib call
    if (tracking_)
      trackRegions(cur_color_frame_, cur_depth_frame_, cur_point_cloud_);
  }

  bool getTableLocation(LocateTable::Request& req, LocateTable::Response& res)
  {
    if ( have_depth_data_ )
    {
      res.table_centroid = getTablePlane(cur_color_frame_, cur_depth_frame_,
                                         cur_point_cloud_);
      if (res.table_centroid.pose.position.x == 0.0 &&
          res.table_centroid.pose.position.y == 0.0 &&
          res.table_centroid.pose.position.z == 0.0)
      {
        return false;
      }

    }
    else
    {
      ROS_ERROR_STREAM("Calling getTableLocation prior to receiving sensor data.");
      return false;
    }
    return true;
  }

  bool getPushPose(PushPose::Request& req, PushPose::Response& res)
  {
    if ( have_depth_data_ )
    {
      res = findPushPose(cur_color_frame_, cur_depth_frame_, cur_point_cloud_);
    }
    else
    {
      ROS_ERROR_STREAM("Calling getPushPose prior to receiving sensor data.");
      return false;
    }
    return true;
  }

  PushPose::Response findPushPose(cv::Mat visual_frame,
                                  cv::Mat depth_frame,
                                  XYZPointCloud& cloud)
  {
    cv::Mat regions = getSuperpixels(visual_frame, depth_frame, cloud);

    // TODO: Choose a patch based on some simple criterian
    // TODO: Estimate the surface of the patch from the depth image
    // TODO: Extract the push pose as point in the center of that surface
    // TODO: Transform to be in the workspace_frame
    PushPose::Response res;
    PoseStamped p;
    res.push_pose = p;
    res.invalid_push_pose = false;
    return res;
  }

  //
  // Region tracking methods
  //

  void startTracker(const tabletop_pushing::ObjectSingulationGoalConstPtr &goal)
  {
    tracker_.stop();
    tracking_ = true;

  }
  void stopTracker(const tabletop_pushing::ObjectSingulationGoalConstPtr &goal)
  {
    tracker_.stop();
    tracking_ = false;
  }

  void trackRegions(cv::Mat color_frame, cv::Mat depth_frame,
                    XYZPointCloud& cloud)
  {
    cv::Mat regions = getSuperpixels(color_frame, depth_frame, cloud);

    if (!tracker_.isInitialized())
    {
      // Fit plane to table
      // TODO: Get extent as well
      table_centroid_ = getTablePlane(color_frame, depth_frame, cloud);
      ROS_INFO_STREAM("Got centroid");
      initRegionTracks(color_frame, depth_frame);
      ROS_INFO_STREAM("Initialized region tracks");
      return;
    }

    cv::Mat moving = updateRegionTracks(color_frame, depth_frame, regions);
    cv::RotatedRect el = computeEllipsoid2D(color_frame, moving);

    // Compute ellipsoid changes from previous frame and store as part of
    // current state
    if (cur_ellipse_ == EMPTY_ELLIPSE)
    {
      cur_ellipse_ = el;
    }
    else if (el != EMPTY_ELLIPSE)
    {
      // Calculate the deltas
      cv::RotatedRect delta(cv::Point2f(el.center.x - cur_ellipse_.center.x,
                                        el.center.y - cur_ellipse_.center.y),
                            cv::Size2f(el.size.width - cur_ellipse_.size.width,
                                       el.size.height - cur_ellipse_.size.height),
                            el.angle - cur_ellipse_.angle);
      delta_ellipse_ = delta;
    }
    else
    {
      // No detected movement so maintain keep the last one stationary
      delta_ellipse_ = EMPTY_ELLIPSE;
    }
  }

  void initRegionTracks(cv::Mat& color_frame, cv::Mat& depth_frame)
  {
    cv::Mat bw_frame(color_frame.rows, color_frame.cols, CV_8UC1);
    cv::cvtColor(color_frame, bw_frame, CV_BGR2GRAY);
    tracker_.initTracks(bw_frame);
  }

  cv::Mat updateRegionTracks(cv::Mat& color_frame, cv::Mat& depth_frame,
                             cv::Mat& regions)
  {
    cv::Mat bw_frame(color_frame.rows, color_frame.cols, CV_8UC1);
    cv::cvtColor(color_frame, bw_frame, CV_BGR2GRAY);
    cv::Mat prev_bw_frame(prev_color_frame_.rows, prev_color_frame_.cols,
                          CV_8UC1);
    cv::cvtColor(prev_color_frame_, prev_bw_frame, CV_BGR2GRAY);
    std::vector<Flow> sparse_flow = tracker_.updateTracksKLT(bw_frame,
                                                             prev_bw_frame);
    // std::vector<Flow> sparse_flow = imageDifferencing(color_frame, depth_frame,
    //                                                   prev_color_frame_,
    //                                                   prev_depth_frame_);
    ROS_INFO_STREAM("Sparse flow has size: " << sparse_flow.size());
    // Determine which regions are moving
    std::multimap<uchar, Flow> moving_regions;

    ROS_DEBUG_STREAM("Finding moving points");
    for (unsigned int i = 0; i < sparse_flow.size(); ++i)
    {
      // Filter out small flow
      if (sparse_flow[i].dx + sparse_flow[i].dy <= min_flow_thresh_)
        continue;

      // Ignore movement in the background regions
      if (cur_workspace_mask_.at<uchar>(sparse_flow[i].y,
                                        sparse_flow[i].x) == 0)
        continue;

      // Add regions associated with moving points to the set
      // Store the flow associated with each region
      moving_regions.insert(RegionMember
                            (regions.at<uchar>(sparse_flow[i].y,
                                               sparse_flow[i].x),
                             sparse_flow[i]));
    }

    // TODO: Remove table plane regions from possible moving regions
    // Create a mask of the moving regions drawn
    cv::Mat moving_regions_mask(regions.rows, regions.cols, CV_8UC1,
                                cv::Scalar(0));
    for (int r = 0; r < regions.rows; ++r)
    {
      for (int c = 0; c < regions.cols; ++c)
      {
        // Test if the region value at r,c is moving
        if (moving_regions.count(regions.at<uchar>(r,c)) >=
            num_region_points_thresh_)
        {
          moving_regions_mask.at<uchar>(r,c) = 255;
        }
      }
    }

#ifdef DISPLAY_MOVING_STUFF
    // Create a color image of the moving parts using the mask
    cv::Mat moving_regions_img;
    color_frame.copyTo(moving_regions_img, moving_regions_mask);
    cv::imshow("Mask", moving_regions_mask);
    cv::imshow("Moving regions", moving_regions_img);
    cv::waitKey(display_wait_ms_);
#endif // DISPLAY_MOVING_STUFF

    return moving_regions_mask;
  }

  std::vector<Flow> imageDifferencing(cv::Mat cur_color_frame,
                                      cv::Mat cur_depth_frame,
                                      cv::Mat prev_color_frame,
                                      cv::Mat prev_depth_frame)
  {
    cv::Mat color_diff_img = cv::abs(cur_color_frame - prev_color_frame);
    std::vector<cv::Mat> diff_channels;
    // TODO: Use cv::Vec3b instead of split...
    cv::split(color_diff_img, diff_channels);
    cv::Mat depth_diff_img = cv::abs(cur_depth_frame - prev_depth_frame);
    for (int r = 0; r < depth_diff_img.rows; ++r)
    {
      for (int c = 0; c < depth_diff_img.cols; ++c)
      {
        if (cur_depth_frame.at<float>(r,c) == 0.0 ||
            prev_depth_frame.at<float>(r,c) == 0.0)
          depth_diff_img.at<float>(r,c) = 0.0;
      }
    }
    cv::Mat change_mask(cur_color_frame.rows, cur_color_frame.cols, CV_8UC1,
                        cv::Scalar(0));
    for (int r = 0; r < change_mask.rows; ++r)
    {
      for (int c = 0; c < change_mask.cols; ++c)
      {
        if (diff_channels[0].at<uchar>(r,c) > (uchar) intensity_diff_thresh_ ||
            diff_channels[1].at<uchar>(r,c) > (uchar) intensity_diff_thresh_ ||
            diff_channels[2].at<uchar>(r,c) > (uchar) intensity_diff_thresh_ ||
            depth_diff_img.at<float>(r,c) > depth_diff_thresh_)
        {
          change_mask.at<uchar>(r,c) = 255;
        }
      }
    }
    cv::Mat change_morphed(change_mask.rows, change_mask.cols, CV_8UC1);
    cv::Mat element(5, 5, CV_8UC1, cv::Scalar(255));
    cv::erode(change_mask, change_morphed, element);

#ifdef DISPLAY_IMAGE_DIFFERENCING
    // cv::imshow("diff0", diff_channels[0]);
    // cv::imshow("diff1", diff_channels[1]);
    // cv::imshow("diff2", diff_channels[2]);
    // cv::imshow("diff_depth", depth_diff_img);
    cv::imshow("change_mask", change_mask);
    cv::imshow("change_morphed", change_morphed);
#endif // DISPLAY_IMAGE_DIFFERENCING
    std::vector<Flow> sparse_flow;
    for (int r = 0; r < change_morphed.rows; ++r)
    {
      for (int c = 0; c < change_morphed.cols; ++c)
      {
        if (change_morphed.at<uchar>(r,c))
          sparse_flow.push_back(Flow(c, r, 1, 1));
      }
    }
    return sparse_flow;
  }

  //
  // Controller State representations
  //

  cv::RotatedRect computeEllipsoid2D(cv::Mat& color_frame, cv::Mat moving)
  {
    cv::Mat contour_img;
    color_frame.copyTo(contour_img, moving);
    std::vector<std::vector<cv::Point> > moving_contours;
    moving_contours.clear();
    // NOTE: This method makes changes to the "moving" image
    cv::findContours(moving, moving_contours, cv::RETR_EXTERNAL,
                     CV_CHAIN_APPROX_NONE);

    // TODO: Compute a single ellipse to describe the motion?
    std::vector<cv::RotatedRect> ellipses;
    // Compute secondary features from these
    for (unsigned int i = 0; i < moving_contours.size(); ++i)
    {
      if (moving_contours[i].size() < 6) continue;
      // Get moments
      cv::Mat pt_mat(moving_contours[i]);
      cv::Moments m;
      m = cv::moments(pt_mat);
      // Fit an ellipse
      cv::RotatedRect el = cv::fitEllipse(pt_mat);
#ifdef SHOW_ELLIPSE_INFO
      ROS_INFO_STREAM("ellipse " << i << " has center (" << el.center.x
                      << ", " << el.center.y << ")"
                      <<  " and size (" << el.size.width << ", "
                      << el.size.height << ")");
#endif // SHOW_ELLIPSE_INFO
      // Draw ellipse for display purposes
      cv::Scalar ellipse_color(255, 0, 0);
      cv::ellipse(contour_img, el, ellipse_color, 2);
      ellipses.push_back(el);
    }
    cv::Scalar object_contour_color(0, 0, 255);
    if (moving_contours.size() > 0)
    {
      cv::drawContours(contour_img, moving_contours, -1,
                       object_contour_color, 2);
    }
#ifdef DISPLAY_ELLIPSE_STUFF
    cv::imshow("contour window", contour_img);
    cv::waitKey(display_wait_ms_);
#endif // DISPLAY_ELLIPSE_STUFF
    if (ellipses.size() > 0)
      return ellipses[0];
    return EMPTY_ELLIPSE;
  }

  void computeEllipsoid3D(cv::Mat& color_frame, cv::Mat& depth_frame,
                          cv::Mat moving)
  {
    // TODO: Determine return type to use (3D Pose and axes sizes)
  }


  //
  // Visual Feature Extraction
  //

  cv::Mat getSuperpixels(cv::Mat& color_frame, cv::Mat& depth_frame,
                         XYZPointCloud& cloud)
  {
    cv::Mat display_regions;
    int num_regions = 0;
    cv::Mat workspace_mask(color_frame.rows, color_frame.cols, CV_8UC1,
                           cv::Scalar(255));
    // Black out pixels in color and depth images outside of worksape
    for (int r = 0; r < depth_frame.rows; ++r)
    {
      for (int c = 0; c < depth_frame.cols; ++c)
      {
        // NOTE: Cloud is accessed by at(column, row)
        pcl::PointXYZ cur_pt = cloud.at(crop_min_x_ + c, crop_min_y_ + r);
        if (cur_pt.x < min_workspace_x_ || cur_pt.x > max_workspace_x_ ||
            cur_pt.y < min_workspace_y_ || cur_pt.y > max_workspace_y_ ||
            cur_pt.z < min_workspace_z_ || cur_pt.z > max_workspace_z_)
        {
          workspace_mask.at<uchar>(r,c) = 0;
        }
      }
    }
    cv::Mat color_for_seg;
    color_frame.copyTo(color_for_seg, workspace_mask);
    cv::Mat depth_for_seg;
    depth_frame.copyTo(depth_for_seg, workspace_mask);

#ifdef DISPLAY_SUPERPIXELS
    // cv::imshow("color_frame", color_frame);
    // cv::imshow("depth_frame", depth_frame);
    cv::imshow("color_for_seg_frame", color_for_seg);
    cv::imshow("depth_for_seg_frame", depth_for_seg);
#endif // DISPLAY_SUPERPIXELS

    // cv::cvtColor(color_for_seg, color_for_seg, CV_BGR2Lab);
    cv::Mat regions = getSuperpixelImage(color_for_seg, depth_for_seg,
                                         num_regions, display_regions,
                                         sigma_, k_, min_size_,
                                         wr_, wg_, wb_,wd_);
#ifdef SHOW_ELLIPSE_INFO
    ROS_INFO_STREAM("Computed " << num_regions << " regions");
#endif // SHOW_ELLIPSE_INFO

#ifdef DISPLAY_SUPERPIXELS
    cv::imshow("regions", display_regions);
    // cv::imshow("real_regions", regions);
#endif // DISPLAY_SUPERPIXELS
    cur_workspace_mask_ = workspace_mask;
    return regions;
  }

  PoseStamped getTablePlane(cv::Mat& color_frame, cv::Mat& depth_frame,
                            XYZPointCloud& cloud)
  {
    // // Downsample using a voxel grid for faster performance
    // pcl::PointCloud<pcl::PointXYZ> cloud_downsampled;
    // pcl::VoxelGrid<pcl::PointXYZ> downsample;
    // downsample.setInputCloud(
    //     boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud));
    // downsample.setLeafSize(0.005, 0.005, 0.005);
    // downsample.filter(cloud_downsampled);
    // ROS_INFO_STREAM("Voxel Downsampled Cloud");

    // Filter Cloud to not look for table planes on the ground
    // TODO: Transform from globabl ground plane limits
    pcl::PointCloud<pcl::PointXYZ> cloud_z_filtered;
    pcl::PassThrough<pcl::PointXYZ> z_pass;
    z_pass.setInputCloud(
        boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud));
    // z_pass.setInputCloud(
    //     boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud));
    z_pass.setFilterFieldName ("z");
    z_pass.setFilterLimits(min_table_z_, max_table_z_);
    z_pass.filter(cloud_z_filtered);

    // Segment the tabletop from the points using RANSAC plane fitting
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices plane_inliers;
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> plane_seg;
    plane_seg.setOptimizeCoefficients (true);
    plane_seg.setModelType (pcl::SACMODEL_PLANE);
    plane_seg.setMethodType (pcl::SAC_RANSAC);
    plane_seg.setDistanceThreshold (0.01);
    plane_seg.setInputCloud (
        boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud_z_filtered));
    plane_seg.segment(plane_inliers, coefficients);
    // Check size of plane_inliers
    if (plane_inliers.indices.size() < 1)
    {
      ROS_WARN_STREAM("No points found by RANSAC plane fitting");
      PoseStamped p;
      p.pose.position.x = 0.0;
      p.pose.position.y = 0.0;
      p.pose.position.z = 0.0;
      p.header = cloud.header;
      return p;
    }

    // Extract the plane members into their own point cloud
    pcl::PointCloud<pcl::PointXYZ> plane_cloud;
    pcl::copyPointCloud(cloud_z_filtered, plane_inliers, plane_cloud);

    // Return plane centroid
    Eigen::Vector4f xyz_centroid;
    pcl::compute3DCentroid(plane_cloud, xyz_centroid);
    PoseStamped p;
    p.pose.position.x = xyz_centroid[0];
    p.pose.position.y = xyz_centroid[1];
    p.pose.position.z = xyz_centroid[2];
    p.header = cloud.header;
    ROS_INFO_STREAM("Table centroid is: ("
                    << p.pose.position.x << ", "
                    << p.pose.position.y << ", "
                    << p.pose.position.z << ")");
    // TODO: Get extent as well
    return p;
  }

  /**
   * Executive control function for launching the node.
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
  message_filters::Subscriber<sensor_msgs::Image> image_sub_;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
  message_filters::Synchronizer<MySyncPolicy> sync_;
  actionlib::SimpleActionServer<tabletop_pushing::ObjectSingulationAction> track_server_;
  sensor_msgs::CvBridge bridge_;
  tf::TransformListener tf_;
  ros::ServiceServer push_pose_server_;
  ros::ServiceServer table_location_server_;
  cv::Mat cur_color_frame_;
  cv::Mat cur_depth_frame_;
  cv::Mat prev_color_frame_;
  cv::Mat prev_depth_frame_;
  XYZPointCloud cur_point_cloud_;
  cv::Mat cur_workspace_mask_;
  FeatureTracker tracker_;
  bool have_depth_data_;
  double k_;
  double sigma_;
  int min_size_;
  double wr_;
  double wg_;
  double wb_;
  double wd_;
  int min_flow_thresh_;
  int num_region_points_thresh_;
  int crop_min_x_;
  int crop_max_x_;
  int crop_min_y_;
  int crop_max_y_;
  int display_wait_ms_;
  cv::RotatedRect cur_ellipse_;
  cv::RotatedRect delta_ellipse_;
  double min_workspace_x_;
  double max_workspace_x_;
  double min_workspace_y_;
  double max_workspace_y_;
  double min_workspace_z_;
  double max_workspace_z_;
  double min_table_z_;
  double max_table_z_;
  std::string workspace_frame_;
  PoseStamped table_centroid_;
  int intensity_diff_thresh_;
  double depth_diff_thresh_;
  bool tracking_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "tabletop_perception_node");
  ros::NodeHandle n;
  RegionTrackingNode tracking_node(n);
  tracking_node.spin();
}
