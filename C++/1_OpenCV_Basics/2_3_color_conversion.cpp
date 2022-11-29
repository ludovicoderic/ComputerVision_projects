#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char* argv[] ) {
    Mat image = imread(argv[1]);
    Mat converted;
    Mat converted1;
    Mat converted2;
    Mat converted3;
    Mat converted4;
    vector<Mat> channels1;
    vector<Mat> channels2;
    vector<Mat> channels3;
    vector<Mat> channels4;


    if (!image.data) {
        cout << "Error opening image" << endl;
        return -1;
    }

    cvtColor(image, converted, COLOR_BGR2GRAY); // Convert RGB to grayscale
    cvtColor(image, converted1, CV_BGR2Lab); // Convert to CIELab
    cvtColor(image, converted2, CV_BGR2HSV); // Convert to HSV
    cvtColor(image, converted3, CV_BGR2HLS); // Convert to HLS
    cvtColor(image, converted4, CV_BGR2YCrCb); // Convert to YCbCr

    //We divide the images into their channels to represent them individually on the screen
    split(converted1, channels1);
    split(converted2, channels2);
    split(converted3, channels3);
    split(converted4, channels4);

    // show the channels
    namedWindow( "RGB", WINDOW_AUTOSIZE ); // Create a window
    imshow("RGB", image);
    imshow("CIELab", channels1[0]); //Channel L
    imshow("HSV", channels2[2]); //V-channel
    imshow("HLS", channels3[1]); //Channel L
    imshow("YCbCr", channels4[0]); //Channel Y
 
    //Save the channels
    imwrite("cielab_l.jpg",channels1[0]); //Save the channel
    imwrite("hsv_v.jpg",channels2[2]);
    imwrite("hsl_l.jpg",channels3[1]);
    imwrite("ycrcb_y.jpg",channels4[0]);
    
    waitKey(0);
}