#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;


Mat gradient_u(Mat image, int rows, int cols) {

    Mat kernel = (Mat_<float>(rows,cols) << -1, 0, 1,-1, 0, 1,-1, 0, 1);

    Mat image2;
    filter2D(image, image2, -1, kernel);

    return image2;
}

Mat gradient_v(Mat image, int rows, int cols) {

    Mat kernel = (Mat_<float>(rows,cols) << -1, -1, -1,0, 0, 0,1, 1, 1);

    Mat image2;
    filter2D(image, image2, -1, kernel);

    return image2;
}

Mat gx(Mat &src) {
    return gradient_u(src,3,3);
}

Mat gy(Mat &src) {
    return gradient_v(src,3,3);
}

int main(int argc, const char* argv[]) {
    Mat image = imread((argv[1]), IMREAD_GRAYSCALE);

    // Calculamos gradiente horizontal y vertical
    Mat dx = gx(image);
    Mat dy = gy(image);

    // Pasamos a float para que los calculos no saturen en la funcion magnitude
    dx.convertTo(dx, CV_32F);
    dy.convertTo(dy, CV_32F);

    // Calculamos la magnitud
    Mat magnitud;
    magnitude(dx, dy, magnitud); // magnitude hace la operación sqrt(dx^2 + dy^2)

    // Convertimos de float a uchar para poder visualizar la imagen
    magnitud.convertTo(magnitud, CV_8UC1);
    //imshow("Magnitud", magn);
    imwrite("lena_prewitt.jpg",magnitud);
    waitKey(0);
}
