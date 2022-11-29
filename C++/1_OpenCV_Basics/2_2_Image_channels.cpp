#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main() {
    Mat color_image = imread("lena.jpg");
    vector<Mat> channels;
    split(color_image, channels);

    imshow("Blue", channels[0]);
    imshow("Green", channels[1]);
    imshow("Red", channels[2]);

    waitKey(0);
}