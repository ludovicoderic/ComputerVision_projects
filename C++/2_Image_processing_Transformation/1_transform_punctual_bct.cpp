#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

const string keys =
       "{help h usage ? |   | imprimir este mensaje }"
       "{@image         |   | imagen original       }" 
       "{@alpha         |   | valor de contraste    }"
       "{@beta          |   | valor de brillo       }";

int main(int argc, char* argv[])
{
   CommandLineParser parser(argc, argv, keys);

   if (parser.has("help")) {
       parser.printMessage(); 
       return 0;
   }

   if (!parser.has("@image") || !parser.has("@alpha") || !parser.has("@beta")) {
       parser.printMessage();
       return -1;
   }

   string imageFilename = parser.get<string>("@image");
   double alpha = parser.get<double>("@alpha");
   double beta = parser.get<int>("@beta");


    Mat image = imread(argv[1],IMREAD_GRAYSCALE);
    
    if (!image.data) {
        cout << "Error opening image" << endl;
        return -1;
    }

    Mat image2 = alpha * (image - 128) + 128 + beta;

    //Mostramos y guardamos la imagen
    namedWindow( "Imagen Original", WINDOW_AUTOSIZE );  // Crear una ventana
    imshow("Imagen Original", image);
    imshow("Imagen Alterada", image2);
    imwrite(imageFilename,image2);
    
    waitKey(0);
}