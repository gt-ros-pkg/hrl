#include <opencv2/core/core.hpp>
namespace cpl_superpixels
{
cv::Mat getSuperpixelImage(cv::Mat input_img, int& num_ccs,
                           cv::Mat& display_img, double sigma=0.3,
                           double k=500.0, int min_size=10);
cv::Mat getSuperpixelImage(cv::Mat color_img, cv::Mat depth_img, int& num_ccs,
                           cv::Mat& display_img, double sigma=0.3,
                           double k=500.0, int min_size=10, double wr=0.1,
                           double wg=0.1, double wb=0.1, double wd=0.7);
};
