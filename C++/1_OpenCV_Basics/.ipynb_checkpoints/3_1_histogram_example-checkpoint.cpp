#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main() {
    Mat image = imread("lena.jpg", IMREAD_GRAYSCALE);

    if (!image.data) {
        cout << "Error opening image" << endl;
        return -1;
    }

    Mat hist;  // Variable donde guardaremos el histograma
    int histSize = 256;  // Numero de bins del histograma
    calcHist(&image, 1, 0, Mat(), hist, 1, &histSize, 0); // Calculamos el histograma 

    // Mostramos los valores por pantalla
    for( int i = 0; i < histSize; i++ )
            cout << " " <<  hist.at<float>(i);
    cout << endl;

    // Mostramos una grafica con el histograma
    int hist_w = 512; int hist_h = 400;
    int bin_w = cvRound((double)hist_w/histSize);

    Mat histImage(hist_h, hist_w, CV_8UC1, Scalar(0,0,0));

    // Normalizamos el histograma entre 0 y histImage.rows
    normalize(hist, hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());

    // Dibujamos la grafica del histograma usando line, que crea una linea entre dos puntos.
    for( int i = 1; i < histSize; i++ ) {
      line( histImage, Point(bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1))),
                       Point(bin_w*(i), hist_h - cvRound(hist.at<float>(i))),
                       Scalar(255, 0, 0), 2, 8, 0  );
    }

    namedWindow("Result", WINDOW_AUTOSIZE);
    imshow("Result", histImage);

    waitKey(0);
}
