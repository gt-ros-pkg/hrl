#include <opencv2/core/core.hpp>
namespace cpl_superpixels
{
cv::Mat getSuperpixelImage(cv::Mat input_img, int& num_ccs, double sigma=0.3,
                           int k=500, int min_size=10);
};
