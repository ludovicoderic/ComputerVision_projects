#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, const char* argv[])
{
    if (argc!=2) {
        cerr << "Syntax: " << argv[0] << " <image>" << endl;
        exit(-1);
    }

    Mat img = imread(argv[1]);
    Mat img2, img3, imgCanny, imgCannyf, imgBF, imgCanny3c, result, resultf;


/** BORDES **/
    // Aplicamos un filtro de mediana de tamaño 7x7 para quitar ruido.
    medianBlur(img, img2, 7);

    // Detectamos los bordes con Canny, umbral inferior 50 y superior 150.
    int lowThreshold = 30;
    int ratio = 3;

    Canny (img2, img3, 50, 50*3); // Filtro canny con los umbrales minimo y maximo (hysteresis) proporcionados

    // Dilatamos los bordes mediante un filtro cuadrado de 2x2
    int erosion_type = MORPH_RECT; // Forma del filtro
    int erosion_size = 2;             // Tamaño del filtro (2x2)

    Mat element = getStructuringElement(erosion_type,Size(erosion_size, erosion_size));
    dilate(img3, imgCanny, element);

    // Escalamos los valores resultantes a 1 y los invertimos
    imgCanny = 1 - (imgCanny / 255);

    // Convertimos la imagen a float para permitir multiplicaciones con valores entre 0 y 1
    imgCanny.convertTo(imgCannyf, CV_32FC3);

    // Aplicamos un filtro gaussiano de 5x5 pixels con desviacion tipica 0
    GaussianBlur(imgCannyf, imgCannyf, Size(5,5), 0); // Realiza un filtrado gaussiano con un kernel de 5x5 píxeles y desviación típica 0

/** COLOR **/
    // Sobre la imagen original, aplicamos un filtro bilateral de diametro 9 con umbrales 150 y 150. 
    bilateralFilter(img, imgBF, 9, 150, 150); // Aplica un filtro bilateral con un diámetro de 9 pixeles vecinos y una intensidad mínima 150.

    // Truncamos los colores. En este caso usamos un valor de 25 (cuanto mas alto mas "cartoonizado")
    result = imgBF / 25;
    result = result * 25;

    // Convertimos la imagen a float, igual que hemos hecho con los bordes
    result.convertTo(resultf, CV_32FC3);

/** UNIMOS COLOR Y BORDES **/
    Mat cannyChannels[] = { imgCannyf, imgCannyf, imgCannyf };
    merge(cannyChannels, 3, imgCanny3c);

    // Multiplicamos las matrices de color (resultf) y bordes (imgCanny3c)
    multiply(resultf,imgCanny3c,resultf);
    // convertimos el resultado a una imagen estandar (color de 8 bits)
    resultf.convertTo(result, CV_8UC3);

    // Mostramos el resultado
    imshow("Input", img);
    imshow("Result", result);

    imwrite("cartoonized.jpg",result);

    waitKey(0);

    return 0;
}