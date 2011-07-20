#include <ros/ros.h>
#include <cpl_superpixels/segment/segment.h>
#include <opencv2/highgui/highgui.hpp>
#include <string>

#include <cpl_superpixels/segment/segment-image.h>
#include <cpl_superpixels/segment/image.h>
#include <cpl_superpixels/segment/converter.h>

namespace cpl_superpixels
{

cv::Mat getSuperpixelImage(cv::Mat input_img, int& num_ccs, cv::Mat& disp_img,
                           double sigma, double k, int min_size)
{
  IplImage ipl_img = input_img;
  // Superpixels, Felzenszwalb
  image<rgb>* im = IPLtoFELZS(&ipl_img);
  image<rgb> *disp_im = segment_image(im, sigma, k, min_size, &num_ccs);
  delete im;
  // Convert to cv::Mat
  IplImage* disp_ipl;
  IplImage* idx_ipl;
  disp_ipl = FELZStoIPL(disp_im);
  idx_ipl = FELZSIDXtoIPL(disp_im);
  delete disp_im;
  cv::Mat tmp_img(disp_ipl);
  tmp_img.copyTo(disp_img);
  cvReleaseImage(&disp_ipl);
  cv::Mat tmp_img2(idx_ipl);
  cv::Mat idx_img(tmp_img2.size(), tmp_img2.type());
  tmp_img2.copyTo(idx_img);
  cvReleaseImage(&idx_ipl);
  return idx_img;
}

cv::Mat getSuperpixelImage(cv::Mat color_img, cv::Mat depth_img, int& num_ccs,
                           cv::Mat& disp_img, double sigma, double k,
                           int min_size,
                           double wr, double wg, double wb, double wd)
{
  IplImage color_ipl_img = color_img;
  // Superpixels, Felzenszwalb
  image<rgb>* color_im = IPLtoFELZS(&color_ipl_img);
  image<float>* depth_im = DEPTHtoFELZS(depth_img);
  cv::Mat depth_revert;
  image<rgb> *disp_im = segment_image(color_im, depth_im, sigma, k, min_size,
                                      &num_ccs, wr, wg, wb, wd);
  delete color_im;
  delete depth_im;
  // Convert to cv::Mat
  IplImage* disp_ipl;
  IplImage* idx_ipl;
  cv::Mat idx_img;
  disp_ipl = FELZStoIPL(disp_im);
  idx_img = FELZSIDXtoMAT(disp_im);
  delete disp_im;
  cv::Mat tmp_img(disp_ipl);
  tmp_img.copyTo(disp_img);
  cvReleaseImage(&disp_ipl);
  return idx_img;
}

};
