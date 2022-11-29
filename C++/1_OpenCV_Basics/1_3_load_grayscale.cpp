#include <opencv2/opencv.hpp> // Incluimos OpenCV
#include <iostream>

using namespace cv;
using namespace std;

int main( int argc, char* argv[] ) {

    if( argc != 3) {  //cambiamos a 3 porque al ejecutarlo desde el terminal introducimos 2 palabras
      cout <<" Uso: display_image <imagen>" << endl;
      return -1;
    }

    Mat image = imread(argv[1],  IMREAD_GRAYSCALE);   // Leer fichero de imagen

    if (!image.data ) {        // Comprobar lectura
        cout <<  "Could not open or find the image" << endl;
        return -1;
    }

    namedWindow( "Ventana", WINDOW_AUTOSIZE );  // Crear una ventana
    imshow( "Ventana", image );                 // Mostrar la imagen en la ventana
    imwrite(argv[2],image);

    waitKey(0);   // Esperar a pulsar una tecla en la ventana

    return 0;
}
