#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv/cxcore.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <sstream>
#include <algorithm>

using namespace cv;
using namespace std;
typedef vector<float> Descriptor;
static const bool DRAW_KEYPOINTS = false;
/*
 * SSD
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
 * RatioTest
 * @short Computes the  ratio test described in Lowe 2004
 * @param a Descriptor from the first image to compare
 * @param bList List of descriptors from the second image
 * @param threshold Threshold value for ratioTest comparison
 *
 * @return index of the best match, -1 if no match ratio is less than threshold
 */
int ratioTest(Descriptor& a, vector<Descriptor>& bList, double threshold)
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

/*
 * FindMatches
 * @short Finds matching points on two images based on the set of descriptors
 * extracted from two images
 * @param descriptors1 Descriptors from first image
 * @param descriptors2 Descriptors from second image
 * @param matches1 Indexes of matching points in descriptors1 (returned)
 * @param matches2 Indexes of matching points in descriptors2 (returned)
 * @param dType Descriptor type
 * @param mType Matching distance metric type
 */
void findMatches(vector<Descriptor>& descriptors1,
                 vector<Descriptor>& descriptors2,
                 vector<int>& matches1, vector<int>& matches2)
{
  double threshold = 0.01;

  for (unsigned int a = 0; a < descriptors1.size(); ++a)
  {
    const int bestIndex = ratioTest(descriptors1[a], descriptors2,
                                    threshold);
    if (bestIndex != -1) {
      matches1.push_back(a);
      matches2.push_back(bestIndex);
    }
  }
}
/**
 * improvedFindMatches
 * @short Improved matching algorithm for Question 5.3
 * @param descriptors1 List of descriptors from image 1
 * @param descriptors2 List of descriptors from image 2
 * @param matches1 Indexes of matching points in image 1 (Returned)
 * @param matches2 Indexes of matching points in image 2 (Returned)
 */
void improvedFindMatches(vector<Descriptor>& descriptors1,
                        vector<Descriptor>& descriptors2,
                        vector<int>& matches1, vector<int>& matches2)
{
    // Ratio threshold for accepting a descriptor
    const double threshold = 0.7;

    // Determine matches using the Ratio Test method from Lowe 2004
    for (unsigned int a = 0; a < descriptors1.size(); ++a) {
        const int bestIndex = ratioTest(descriptors1[a], descriptors2,
                                        threshold);
        if (bestIndex != -1) {
            matches1.push_back(a);
            matches2.push_back(bestIndex);
        }
    }

    // Check that the matches are unique going the other direction
    for (unsigned int x = 0; x < matches2.size();) {
        const int bestIndex = ratioTest(descriptors2[matches2[x]],
                                        descriptors1, threshold);
        if (bestIndex != matches1[x]) {
            matches1.erase(matches1.begin()+x);
            matches2.erase(matches2.begin()+x);
        } else {
            x++;
        }
    }

}

int main(int argc, char** argv)
{
  int count = 1;
  if (argc > 1)
    count = atoi(argv[1]);
  namedWindow("obj_reg");

  vector<KeyPoint> prev_keypoints;
  vector<KeyPoint> cur_keypoints;
  vector<KeyPoint> cur_sift_keypoints;
  vector<KeyPoint> prev_sift_keypoints;
  vector<Point2f> cur_points;
  vector<Descriptor> prev_descriptors;
  vector<Descriptor> cur_descriptors;
  vector<Descriptor> prev_sift_descriptors;
  vector<Descriptor> cur_sift_descriptors;

  prev_keypoints.clear();
  cur_keypoints.clear();
  cur_sift_keypoints.clear();
  cur_points.clear();
  prev_descriptors.clear();
  cur_descriptors.clear();
  prev_sift_descriptors.clear();
  cur_sift_descriptors.clear();

  SURF surf(800, 4, 2, true);
  SIFT sift(800, 4, 2, true); //TODO: Fix these parameters
  for (int i = 0; i < count; i++)
  {
    stringstream filepath;
    filepath << "/home/thermans/data/mars_lab_aff_pics/balls0/" << i << ".png";
    Mat frame;
    frame = imread(filepath.str());
    Mat bw_frame(frame.rows, frame.cols, CV_8UC1);

    cvtColor(frame, bw_frame, CV_RGB2GRAY);
    cur_keypoints.clear();
    cur_points.clear();
    cur_descriptors.clear();

    vector<float> raw_descriptors;
    Mat raw_sift_descriptors;

    try
    {
      // goodFeaturesToTrack(bw_frame, cur_points, 750, 0.001, 0.1);
      // for (unsigned int j = 0; j < cur_points.size(); ++j)
      // {
      //   cur_keypoints.push_back(KeyPoint(cur_points[j], 16.0));
      // }
      surf(bw_frame, Mat(), cur_keypoints, raw_descriptors, false);
      //sift(bw_frame, Mat(), cur_sift_keypoints, raw_sift_descriptors, false);
      for (unsigned int j = 0; j < raw_descriptors.size(); j += 128)
      {
        Descriptor d(raw_descriptors.begin() + j,
                     raw_descriptors.begin()+j+128);
        cur_descriptors.push_back(d);
      }
    }
    catch(cv::Exception e)
    {
      cerr << e.err << endl;
    }
    vector<int> matches_cur;
    vector<int> matches_prev;
    matches_cur.clear();
    matches_prev.clear();

    cout << "Displaying image " << i << endl;
    for (unsigned int j = 0; DRAW_KEYPOINTS && j < cur_keypoints.size(); ++j)
    {
      circle(frame, cur_keypoints[j].pt,
             max(pow(2.0, cur_keypoints[j].octave),2.0),
             Scalar(0, 255, 0), 2);
    }
    if(i > 0)
    {
      // Find nearest neighbors with previous descriptors
      improvedFindMatches(cur_descriptors, prev_descriptors,
                          matches_cur, matches_prev);
      cout << "matches size: " << matches_cur.size() << endl;
      // Draw feature tracks on the image
      for (unsigned int j = 0; j < matches_cur.size(); j++)
      {
        line(frame,
             prev_keypoints[matches_prev[j]].pt,
             cur_keypoints[matches_cur[j]].pt,
             Scalar(0,0,255), 1);
        circle(frame,
               prev_keypoints[matches_prev[j]].pt,
               4,Scalar(0,0,255));
        circle(frame,
               cur_keypoints[matches_cur[j]].pt,
               4, Scalar(255,0,0), 1);
      }
    }
    imshow("obj_reg", frame);
    cout << endl;
    waitKey();
    prev_keypoints = cur_keypoints;
    prev_descriptors = cur_descriptors;
  }
  return 0;
}
