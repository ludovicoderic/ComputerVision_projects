#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char* argv[])
{
    Mat image = imread("damas.jpg");
    
    if (!image.data) {
        cout << "Error opening image" << endl;
        return -1;
    }

    Mat image2(640,640, CV_8UC3,Scalar(0,0,0));

    Point2f inputQuad[4]; // Vector con los puntos de la imagen original
    Point2f outputQuad[4]; // Puntos a donde escalar

    inputQuad[0] = Point2f(278,27);
    inputQuad[1] = Point2f(910,44);
    inputQuad[2] = Point2f(27,546);
    inputQuad[3] = Point2f(921,638);
     
    outputQuad[0] = Point2f(0,0);
    outputQuad[1] = Point2f(640,0);
    outputQuad[2] = Point2f(0,640);
    outputQuad[3] = Point2f(640,640);

    // Asignamos valores a esos puntos
    // Obtenemos la matriz de transformación de perspectiva
    Mat lambda = getPerspectiveTransform(inputQuad, outputQuad);

    // Aplicamos la transformacion
    warpPerspective(image, image2, lambda, image2.size());

    //Mostramos y guardamos la imagen
    //namedWindow( "Imagen Original", WINDOW_AUTOSIZE );  // Crear una ventana
    //imshow("Imagen Original", image);
    //imshow("Imagen Alterada", image2);
    imwrite("damas_corrected.jpg",image2);
    
    waitKey(0);
}