#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"
#include <opencv2/videoio.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

int main()
{
    namedWindow("out");
    Mat hsv = Mat(Size(900, 150), CV_8UC3, Scalar(0, 255, 255));
    Mat bgr;
    int hue = 0;

    for (int col = 0; col < 900; col++)
    {
        for (int row = 0; row < hsv.size().height; row++)
        {
            hsv.at<Vec3b>(row, col) = Vec3b(hue, 255, 255);
        }
        if (col % 5 == 0)
        {
            hue++;
        }

        cvtColor(hsv, bgr, CV_HSV2BGR);
        imshow("out", bgr);
        waitKey(2);
    }
    for (int col = 50; col < bgr.size().width; col += 50)
    {
        line(bgr, Point(col, bgr.size().height-30), Point(col, bgr.size().height-50), Scalar(0,0,0), 2);
        putText(bgr, to_string(col / 5), Point(col-10, bgr.size().height-15), FONT_HERSHEY_COMPLEX_SMALL, .75, Scalar(0, 0, 0), 2);
    }
    imshow("out", bgr);
    waitKey(0);
    return 0;
}