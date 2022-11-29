// eceives the name of an image as a parameter
// (load it in grayscale) and show by
// the terminal its brightness and contrast.

#include <opencv2/opencv.hpp> // Incluimos OpenCV
#include <iostream>
#include "math.h"

using namespace cv;
using namespace std;

int main( int argc, char* argv[] ) {

    Mat image = imread(argv[1],  IMREAD_GRAYSCALE);   
    int i, j, numeroPixeles = image.rows * image.cols;
    
    if (!image.data ) {        
        cout <<  "Could not open or find the image" << endl;
        return -1;
    }

    Scalar brillo = mean(image);

    double sumas = 0;
    
    for(i = 0;i < image.rows;i++){
        for(j = 0; j < image.cols;j++){
            double value = image.at<uchar>(i,j);
            sumas = sumas + pow((value - brillo[0]),2);
        }
    }
  
    float contraste = sqrt(sumas/numeroPixeles);
    
    cout << "b=" << brillo[0] << endl << "c=" << contraste << endl;

    waitKey(0);   

    return 0;
}
