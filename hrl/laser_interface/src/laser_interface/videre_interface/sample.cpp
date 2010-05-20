#include "cv.h"
#include "highgui.h"
#include "videre.h"

#include <stdio.h>
#include <stdlib.h>

using namespace std;

int main(int argc, char** argv)
{
    cvNamedWindow("cam1", 1);
    cvNamedWindow("cam2", 1);
    videre::VidereCap cap = videre::VidereCap(0);
    cap.set_mode(videre::NONE);
    while(true)
    {
        cap.process_frames();
        cvShowImage("cam1", cap.left_debayered);
        cvShowImage("cam2", cap.right_debayered);
        cvWaitKey(33);
    }
}

