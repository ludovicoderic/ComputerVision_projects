#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char *argv[]) {
 
    if (argc!=3) {
      cerr << "Sintaxis: " << argv[0] << " <obtenida> <solucion>" << endl;
      return -1;
    }

    Mat obtenida = imread(argv[1],IMREAD_GRAYSCALE);
    Mat solucion = imread(argv[2],IMREAD_GRAYSCALE);
    
    if (!obtenida.data || !solucion.data) {
        cerr << "Error en alguna de las imagenes" << endl;
        return -1;
    }
  
    if (obtenida.size() != solucion.size()) {
        cerr << "Error: Las imagenes deben tener el mismo tamanyo" << endl;
        return -1;
    }

    // Calculamos intersection over union (IoU), es el porcentaje de acierto de nuestro programa 
    float intersectionAB = countNonZero(obtenida & solucion);
    float unionAB = countNonZero(obtenida | solucion);
    float IoU = intersectionAB / unionAB;
    
    cout << IoU << endl;
}
