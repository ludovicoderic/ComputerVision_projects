#include <opencv2/opencv.hpp>
#include <iostream>


using namespace cv;
using namespace std;

int main(int argc, const char* argv[])
{
    if (argc!=4) {
        cerr << "Syntax: " << argv[0] << " <image> <mask> <output>" << endl;
        exit(-1);
    }

    Mat img = imread(argv[1]);
    Mat img_gray, img_gray2, img_gray3,img_gray4,img_gray5;
    Mat mask = imread(argv[2]), mask2, mask3;

    Mat masked = img & mask;
    cvtColor(masked, img_gray, COLOR_BGR2GRAY);  
    cvtColor(mask, mask2, COLOR_BGR2GRAY);  

    //hacemos un threshold para que los valores oscuros se vean mas oscuros y al contrario con los blancos
    adaptiveThreshold(img_gray,img_gray2,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,17,4);

    //Hacemos una erosion
    int erosion_type = MORPH_ELLIPSE; // Forma del filtro
    Mat element = getStructuringElement(erosion_type, Size(10, 10));
    Mat element2 = getStructuringElement(erosion_type, Size(3, 3));

    morphologyEx(img_gray2, img_gray3, MORPH_OPEN, element2);
    erode(mask2,mask3,element);
    
    medianBlur(img_gray3,img_gray3,3);

    morphologyEx(img_gray3, img_gray4, MORPH_OPEN, element2);
    medianBlur(img_gray4,img_gray4,3);

    Mat fin = 255 - img_gray4;
    fin = fin & mask3;

    // Mostramos el resultado
    imshow("Input", fin);
    imshow("Result", img_gray4);
  
    imwrite(argv[3],fin);

    waitKey(0);

    return 0;
}