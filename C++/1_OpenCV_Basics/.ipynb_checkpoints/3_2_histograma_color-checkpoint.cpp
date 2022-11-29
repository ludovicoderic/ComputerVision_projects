// receives by parameter the filename of an image and the name of
// another image where we will save the result. The program 
// displays in a window the histogram of its three basic colors,
// and also save it to the output file.



#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char* argv[] ) {
    Mat image = imread(argv[1]);
  
    if (!image.data) {
        cout << "Error opening image" << endl;
        return -1;
    }

    vector<Mat> channels;
    split(image, channels);

    // Histograma azul
    Mat histB;  // Variable donde guardaremos el histograma
    int histBSize = 256;  // Numero de bins del histograma
    calcHist(&channels[0], 1, 0, Mat(), histB, 1, &histBSize, 0); // Calculamos el histograma 
   
    // Histograma Verde
    Mat histG;  
    int histGSize = 256;  
    calcHist(&channels[1], 1, 0, Mat(), histG, 1, &histGSize, 0); // Calculamos el histograma 
    
    // Histograma Rojo
    Mat histR;  
    int histRSize = 256; 
    calcHist(&channels[2], 1, 0, Mat(), histR, 1, &histRSize, 0); // Calculamos el histograma 

    // Mostramos los valores por pantalla del Hitograma Azul
    cout<<"Histograma Azul"<<endl;
    
    for( int i = 0; i < histBSize; i++ )
        cout << " " <<  histB.at<float>(i);
    cout << endl;
    cout << endl;

    // Histograma Verde
     cout<<"Histograma Verde"<<endl;
     
     for( int i = 0; i < histGSize; i++ )
         cout << " " <<  histG.at<float>(i);
     cout << endl;
     cout << endl;
     
    // Histograma Rojo
    cout<<"Histograma Rojo"<<endl;
    
    for( int i = 0; i < histRSize; i++ )
        cout << " " <<  histR.at<float>(i);
    cout << endl;
    cout << endl;
    

    // Mostramos una grafica con cada histograma
    int hist_w = 512; int hist_h = 400;
    int bin_w = cvRound((double)hist_w/histBSize);


    Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0,0,0));
   
    // Normalizamos el histograma entre 0 y histImage.rows
    normalize(histB, histB, 0, histImage.rows, NORM_MINMAX, -1, Mat());
    normalize(histG, histG, 0, histImage.rows, NORM_MINMAX, -1, Mat());
    normalize(histR, histR, 0, histImage.rows, NORM_MINMAX, -1, Mat());
    
    // Dibujamos la grafica del histograma usando line, que crea una linea entre dos puntos.
    for( int i = 1; i < histBSize; i++ ) {
    // Histograma Azul
    line( histImage, Point(bin_w*(i-1), hist_h - cvRound(histB.at<float>(i-1))),
                    Point(bin_w*(i), hist_h - cvRound(histB.at<float>(i))),
                    Scalar(255, 0, 0), 2, 8, 0  );
    // Histograma Azul
    line( histImage, Point(bin_w*(i-1), hist_h - cvRound(histG.at<float>(i-1))),
                    Point(bin_w*(i), hist_h - cvRound(histG.at<float>(i))),
                    Scalar(0, 255, 0), 2, 8, 0  );
    // Histograma Azul
    line( histImage, Point(bin_w*(i-1), hist_h - cvRound(histR.at<float>(i-1))),
                    Point(bin_w*(i), hist_h - cvRound(histR.at<float>(i))),
                    Scalar(0, 0, 255), 2, 8, 0  );
    }

    namedWindow("Result", WINDOW_AUTOSIZE);
    imshow("Result", histImage);
    imwrite(argv[2],histImage);

    waitKey(0);
    
    return 0;
}