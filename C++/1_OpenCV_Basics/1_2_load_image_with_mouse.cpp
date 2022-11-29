#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;


void handleMouseEvent(int event, int x, int y, int flags, void* param)
{
    Mat* m = (Mat*)param;
    if (event == CV_EVENT_LBUTTONDOWN) {
        Vec3b d = m->at<Vec3b>(y,x);
        cout << x << ", " << y << ": " << d << endl;
        }
}

int main(int argc, char* argv[])
{
    Mat m = imread("lena.jpg");

    if ( m.empty() ) {
        cout << "Error loading the image" << endl;
        return -1;
    }

    namedWindow("Ventana", 1);

    // asignar la función callback para cualquier evento de raton
    setMouseCallback("Ventana", handleMouseEvent, &m);

    imshow("Ventana", m);

    waitKey(0);

    return 0;
}