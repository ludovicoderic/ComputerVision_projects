#include "opencv2/opencv.hpp"
#include <iostream>

using namespace cv;

int main(int argc, const char* argv[]) {

  Mat m = imread((argv[1]), IMREAD_GRAYSCALE);

  // Calculamos gradiente horizontal y vertical
  Mat dx, dy;
  Sobel(m, dx, CV_32F, 1, 0);
  Sobel(m, dy, CV_32F, 0, 1);

  // Calculamos la magnitud
  Mat magn;
  magnitude(dx, dy, magn);

  // Convertimos de float a uchar y mostramos el resultado
  magn.convertTo(magn, CV_8UC1);
  imshow("Sobel", magn);
  waitKey(0);

  return 0;  
}