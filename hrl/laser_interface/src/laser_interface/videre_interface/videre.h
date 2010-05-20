#include "cv.h"
#include "highgui.h"

namespace videre
{
    const int NONE          = 1;
    const int RECTIFIED     = 3;
    const int DISPARITY     = 4;
    const int DISPARITY_RAW = 5;
    
    void split_img(IplImage *src, IplImage* left, IplImage* right);
    void debayer(IplImage *in_img, IplImage *out_img);
    void set_gain(CvCapture *cap, double v);
    void set_exposure(CvCapture *cap, double v);

    class VidereCap {
        public:
            CvCapture *capture;
            IplImage *left, *right, *left_debayered, *right_debayered;

            VidereCap(CvCapture *c);
            VidereCap(int cv_camera);
            ~VidereCap();
            void process_frames();
            void set_mode(int mode);
        private:
            void init();
    };
}




















//{
//    
//    
//    
//    
//
//    void split_img(IplImage *src, IplImage* left, IplImage* right);
//    void debayer(IplImage *in_img, IplImage *out_img);
//    void set_gain(CvCapture *cap, char v);
//    void set_exposure(CvCapture *cap, char v);
//
//    class VidereCap {
//        public:
//            CvCapture *capture;
//            IplImage *left, *right, *left_debayered, *right_debayered;
//
//            VidereCap(CvCapture *c);
//            VidereCap(int cv_camera);
//            ~VidereCap();
//            void process_frames();
//            void set_mode(int mode);
//        private:
//            void init();
//    };
//}
