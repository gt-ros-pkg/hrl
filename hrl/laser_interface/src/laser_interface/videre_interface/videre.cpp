#include "videre.h"
#include <libdc1394/dc1394_control.h>

using namespace videre;

void videre::set_gain(CvCapture *cap, double v)
{
    //uchar vc = (uchar) v;
    cvSetCaptureProperty(cap, FEATURE_GAIN, v);
    //printf("gain %d\n", vc);
}

void videre::set_exposure(CvCapture *cap, double v)
{
    //uchar vc = (uchar) v;
    cvSetCaptureProperty(cap, FEATURE_EXPOSURE, v);
    //printf("exposure %d\n", vc);
}

void videre::split_img(IplImage *src, IplImage* left, IplImage* right)
{
    int size = src->width * src->height * src->nChannels;
    for (int i = 0; i < size; i++)
    {
        if (i % 2 == 0)
            left->imageData[i/2] = src->imageData[i];
        else
            right->imageData[(i-1)/2] = src->imageData[i];
    }
}

void videre::debayer(IplImage *in_img, IplImage *out_img)
{
    cvCvtColor(in_img, out_img, CV_BayerGB2BGR);
}

VidereCap::VidereCap(CvCapture *cap)
{
    capture         = cap;
    init();
}

VidereCap::VidereCap(int cv_camera)
{
    capture = cvCaptureFromCAM(cv_camera);
    init();
}

void VidereCap::init()
{
    IplImage *f     = cvQueryFrame(capture);
    left            = cvCreateImage(cvSize(f->width, f->height), IPL_DEPTH_8U, 1);
    right           = cvCreateImage(cvSize(f->width, f->height), IPL_DEPTH_8U, 1);
    left_debayered  = cvCreateImage(cvSize(f->width, f->height), IPL_DEPTH_8U, 3);
    right_debayered = cvCreateImage(cvSize(f->width, f->height), IPL_DEPTH_8U, 3);
}

VidereCap::~VidereCap()
{
    cvReleaseImage(&left);
    cvReleaseImage(&right);
    cvReleaseImage(&left_debayered);
    cvReleaseImage(&right_debayered);
}

void VidereCap::process_frames()
{
    IplImage *f = cvQueryFrame(capture);
    split_img(f, right, left);
    cvCvtColor(left,  left_debayered,  CV_BayerGB2BGR);
    cvCvtColor(right, right_debayered, CV_BayerGB2BGR);
}

void VidereCap::set_mode(int mode)
{
    cvSetCaptureProperty(capture, CV_CAP_PROP_VIDERE, mode);
}


