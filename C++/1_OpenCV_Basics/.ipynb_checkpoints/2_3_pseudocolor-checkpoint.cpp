#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char* argv[] ) {
    Mat image = imread(argv[1], IMREAD_GRAYSCALE);
    Mat image_color;

    if (!image.data) {
        cout << "Error opening image" << endl;
        return -1;
    }

    applyColorMap(image, image_color, COLORMAP_JET); //Coloreamos la imagen con el pack de colores jet

    //Mostramos la imagen coloreada
    namedWindow( "RGB", WINDOW_AUTOSIZE );  // Crear una ventana
    imshow("imagen coloreada", image_color);
    
    waitKey(0);
}