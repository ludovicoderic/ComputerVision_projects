#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main() {
    Mat src = imread("damas_corrected.jpg");
    Mat src_gray, dst;

    // Pasar la imagen a tipo float y escala de grises
    cvtColor(src, src_gray, COLOR_BGR2GRAY);    
    src_gray.convertTo(src_gray, CV_32F);

    // Detectar las esquinas con Harris. Parametros: blockSize=2, apertureSize=3, k=0.04.
    float blockSize=2, apertureSize=3, k=0.04;

    cornerHarris(src_gray, dst, blockSize, apertureSize, k);

    // Sobre la imagen original, poner en color azul los píxeles detectados como borde.
    // Son aquellos que en los que dst(i,j) tiene un valor mayor de 10000.

    for( int i = 0; i < dst.rows ; i++ ){ 
        for( int j = 0; j <dst.cols; j++ ){
            if(dst.at<float>(i,j) > 10000 ){
                src.at <Vec3b>(i,j)[0]=255;
                src.at <Vec3b>(i,j)[1]=0;
                src.at <Vec3b>(i,j)[2]=0;
             }
        }
     }
    

    // Mostrar por pantalla src y ademas guardarlo en el fichero llamado  "damasHarris.jpg"
    namedWindow("Result", WINDOW_AUTOSIZE);
    imshow("Result",src);
    imwrite("damasHarris.jpg",src);

    waitKey(0);

    return(0);
}
