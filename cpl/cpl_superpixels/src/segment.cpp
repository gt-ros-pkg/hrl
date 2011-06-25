#include <ros/ros.h>
//#include <opencv2/core/core.hpp>
#include <segment/segment.h>
#include <opencv2/highgui/highgui.hpp>
#include <string>

#include <segment/segment-image.h>
#include <segment/image.h>
#include <segment/converter.h>
#include <cpl_superpixels/SmoothClutter.h>

//#define DISPLAY_SUPERPIXELS

namespace cpl_superpixels
{

cv::Mat getSuperpixelImage(cv::Mat input_img, int& num_ccs, double sigma,
                           int k, int min_size)
{
  IplImage ipl_img = input_img;

  // Superpixels, Felzenszwalb
  image<rgb>* im = IPLtoFELZS(&ipl_img);

  image<rgb> *disp_im = segment_image(im, sigma, k, min_size, &num_ccs);
  delete im;

  cv::Mat label_img(input_img.size(), CV_32FC1);

  // Get segment IDs from disp_im
  IplImage* disp_ipl;
  for(int y = 0; y < input_img.rows; ++y)
  {
    for (int x = 0; x <input_img.cols; ++x)
    {
      // Make the indecies zero based
      int idx = disp_im->data[y*disp_im->width() + x].idx - 1;
      label_img.at<float>(y,x) = idx;
    }
  }
  disp_ipl = FELZStoIPL(disp_im);
  delete disp_im;
  cv::Mat disp_img(disp_ipl);
  cv::Mat sp_save_img(disp_img.size(), CV_8UC3);
  disp_img.convertTo(sp_save_img, CV_8UC3);
  cv::imwrite("/home/thermans/Desktop/sp.png", sp_save_img);

#ifdef DISPLAY_SUPERPIXELS
  // Save segmented image for return when queried? or, constantly publish
  ROS_INFO_STREAM("Have: " << num_ccs << " components.");
  cv::imshow("superpixels", disp_img);
  cv::imshow("labels", label_img);
  cv::waitKey(3);
#endif // DISPLAY_SUPERPIXELS

  cvReleaseImage(&disp_ipl);
  return label_img;
}
};
