/*
  Copyright (C) 2009 Georgia Institute of Technology

  This library is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser Public License for more details.

  You should have received a copy of the GNU General Public License
  and the GNU Lesser Public License along with Man.  If not, see
  <http://www.gnu.org/licenses/>.
*/

#include "image.h"
#include <opencv/cv.h>

image<rgb>* IPLtoFELZS(IplImage* input)
{
    image<rgb> *output;
    output = new image<rgb>(input->width, input->height, true);
    unsigned char* p;
    double rColor, gColor, bColor;

    for(int y = 0; y < input->height; ++y) {
        for(int x = 0; x < input->width; ++x) {
            p = (unsigned char*) &input->imageData[input->widthStep * y+
                                                   input->nChannels * x];
            bColor = *p;
            gColor = *(p+1);
            rColor = *(p+2);
            output->data[y * output->width() + x].b = bColor;
            output->data[y * output->width() + x].g = gColor;
            output->data[y * output->width() + x].r = rColor;
        }
    }

    return output;
}

IplImage* FELZStoIPL(image<rgb>* input)
{
    IplImage *output = cvCreateImage(cvSize(input->width(), input->height()),
                                     IPL_DEPTH_8U, 3);

    for(int i = 0; i < output->height; ++i) {
        for(int j = 0; j < output->width; ++j) {
            rgb val = imRef(input, j, i);
            output->imageData[i * output->widthStep +
                              j * output->nChannels + 0] = val.b;
            output->imageData[i * output->widthStep +
                              j * output->nChannels + 1] = val.g;
            output->imageData[i * output->widthStep +
                              j * output->nChannels + 2] = val.r;
        }
    }

    return output;
}

IplImage* FELZSIDXtoIPL(image<rgb>* input)
{
    IplImage *output = cvCreateImage(cvSize(input->width(), input->height()),
                                     IPL_DEPTH_8U, 1);

    for(int i = 0; i < output->height; ++i) {
        for(int j = 0; j < output->width; ++j) {
            output->imageData[i * output->widthStep +
                              j * output->nChannels] = imRef(input, j, i).idx;
        }
    }

    return output;
}

cv::Mat FELZSIDXtoMAT(image<rgb>* input)
{
  cv::Mat output(input->height(), input->width(), CV_8UC1);
  for(int i = 0; i < output.rows; ++i) {
    for(int j = 0; j < output.cols; ++j) {
      output.at<uchar>(i,j) = imRef(input, j, i).idx;
    }
  }
  return output;
}

image<float>* DEPTHtoFELZS(cv::Mat& input)
{
  image<float> *output;
  output = new image<float>(input.cols, input.rows, true);
  for(int y = 0; y < input.rows; ++y) {
    for(int x = 0; x < input.cols; ++x) {
      output->data[y * output->width() + x] = input.at<float>(y,x);
    }
  }
  return output;
}

cv::Mat FELZSDEPTHtoMAT(image<float>* input)
{
  cv::Mat output(input->height(), input->width(), CV_32FC1);
  for(int i = 0; i < output.rows; ++i) {
    for(int j = 0; j < output.cols; ++j) {
      output.at<float>(i, j) = imRef(input, j, i);
    }
  }
  return output;
}
