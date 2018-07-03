
#include <iostream>

//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/opencv.hpp>
#include <opencv/cv.h>

using namespace std;

using namespace cv;

int main(int argc, char **argv)
{ 
  const int length = 6;

  cv::Mat m = (Mat1b(6,6) <<
           0,2,0,2,0,1,
           0,2,0,2,0,2,
           0,2,0,2,0,2,
           0,2,0,2,0,2,
           0,2,0,2,0,2,
           0,2,0,2,0,2);
  cout << "Mat m = (Mat_<uchar>(6,6) << 0,2,...)\n" << m << "\n" << endl;


  cv::Mat m_resized = cv::Mat(0, length, CV_16U);

  cv::Mat m_t = m.t();  
  for (int i=1; i<m.cols; i=i+2) {
    m_resized.push_back(m_t.row(i));
  }
  m = m_resized.t();
  cout << m << endl;


  //st = cvCreateImage(cvSize( src->width / 10, src->height / 10 ), src->depth, src->nChannels );

  //cv::resize( m, m_resized, Size(), 0.5, 0.5, INTER_NEAREST );
  
  cout<<"BEGIN!"<<endl;

}


