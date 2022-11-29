#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char* argv[])
{
    Mat image = imread("damas_corrected.jpg");
    
    if (!image.data) {
        cout << "Error opening image" << endl;
        return -1;
    }

    //Creamos una imagen para la umbralizacion y otra para aplicar la erosion
    Mat image2(image.cols,image.rows, CV_8UC3,Scalar(0,0,0));
    Mat image3(image.cols,image.rows, CV_8UC3,Scalar(0,0,0));
    Mat image4(image.cols,image.rows, CV_8UC3,Scalar(0,0,0));
    Mat image5(image.cols,image.rows, CV_8UC3,Scalar(0,0,0));
    Mat image6(image.cols,image.rows, CV_8UC3,Scalar(0,0,0));

    Mat converted1;
    vector<Mat> channels1;

    //FICHAS ROJAS////////////////////////////////////////////
    //umbralización quedándonos sólo con los píxeles que tengan un color dentro de un rango BGR entre (0,0,50) y (40,30,255).
    inRange(image, Scalar(0, 0, 50), Scalar(40, 30, 255), image2);
    
    
    int erosion_type = MORPH_ELLIPSE; // Forma del filtro
    int erosion_size = 10;             // Tamaño del filtro (10x10)
    int erosion_type2 = MORPH_RECT; // Forma del filtro
    int erosion_size2 = 50;

    Mat element = getStructuringElement(erosion_type, Size(erosion_size, erosion_size));
    Mat element2 = getStructuringElement(erosion_type2,Size(erosion_size2, erosion_size2));
    morphologyEx(image2, image3, MORPH_CLOSE, element);

    //FICHAS BLANCAS////////////////////////////////////////////
    //umbralización quedándonos sólo con los píxeles que tengan un color dentro de un rango BGR entre (0,0,50) y (40,30,255).
    inRange(image, Scalar(80, 80, 80), Scalar(250, 250, 250), image4);
   
    morphologyEx(image4, image5, MORPH_TOPHAT, element2);
    morphologyEx(image5, image6, MORPH_OPEN, element);

    //Mostramos y guardamos la imagen/////////////////////////////
    namedWindow("Imagen Original"), WINDOW_AUTOSIZE );  // Crear una ventana
    imshow("Imagen Original", image);
    imshow("Fichas Rojas", image3);
    imshow("Fichas Blancas", image6);
    
    imwrite("rojas.jpg",image3);
    imwrite("blancas.jpg",image6);
 
    waitKey(0);
}