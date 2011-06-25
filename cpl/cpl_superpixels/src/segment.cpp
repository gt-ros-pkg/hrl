#include <ros/ros.h>
#include <cpl_superpixels/segment/segment.h>
#include <opencv2/highgui/highgui.hpp>
#include <string>

#include <cpl_superpixels/segment/segment-image.h>
#include <cpl_superpixels/segment/image.h>
#include <cpl_superpixels/segment/converter.h>
#include <cpl_superpixels/SmoothClutter.h>

// #define DISPLAY_SUPERPIXELS

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
  // Convert to cv::Mat
  IplImage* disp_ipl;
  disp_ipl = FELZStoIPL(disp_im);
  delete disp_im;
  cv::Mat tmp_img(disp_ipl);
  cv::Mat disp_img(tmp_img.size(), tmp_img.type());
  tmp_img.copyTo(disp_img);
  cvReleaseImage(&disp_ipl);
  return disp_img;
}
};
