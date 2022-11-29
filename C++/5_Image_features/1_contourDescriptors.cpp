#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

struct Descriptors {
     vector<Point> SC;
     float perimetro, compactacion, elongacion, rectangularidad, areaCierreConvexo;
     Point centroide;
     float orientacion;
     double Hu[7];
};

// Funcion que recibe los contornos de una imagen y extrae de ellos n puntos aleatorios para calcular SC
vector<Point> SCPoints(const vector<vector<Point> > &contours, int n=300)
{
    // Cada contorno dentro de contours es un vector de puntos. Sacamos todos los puntos del contorno y los guardamos en allPointsFromImage
    vector<Point> allPointsFromImage;
    for (unsigned j=0; j<contours.size(); j++)
        for (int k=0; k< contours[j].size(); k++)
            allPointsFromImage.push_back(contours[j][k]);

    // "Barajamos" los puntos aleatoriamente, y nos quedamos como maximo con n=300 para usarlos con SC
    random_shuffle(allPointsFromImage.begin(), allPointsFromImage.end());
    vector<Point> points;
    for (int i=0; i<n && i<allPointsFromImage.size(); i++)
        points.push_back(allPointsFromImage[i]);

    return points;
}

Descriptors extractDescriptors(const Mat &image) {
   Descriptors imgDescriptors;

   // Calculamos todos los contornos de la imagen
   vector<vector<Point> > contours;
   findContours(image, contours, RETR_LIST, CHAIN_APPROX_NONE);

   // Extraemos puntos aleatorios de estos contornos para SC y los guardamos en el descriptor
   imgDescriptors.SC = SCPoints(contours);

   // Extraemos el mayor contorno de la imagen (para esto puedes usar la función contourArea), y de este contorno calculamos y guardamos en el descriptor:
        int maxContour ;
        double areaMax = 0.0;
        for(int i=0; i<contours.size();i++)  {
            if(contourArea(contours[i],false) > areaMax){
                maxContour = i;
                areaMax = contourArea(contours[i], false);
            }
        }
    // - Perimetro
        imgDescriptors.perimetro = arcLength(contours[maxContour],true);
    // - Compactacion
        imgDescriptors.compactacion = imgDescriptors.perimetro / areaMax;
    // - Elongacion (usando caja de Feret)
        Rect rectang = boundingRect(contours[maxContour]);
        imgDescriptors.elongacion = float(rectang.width)/rectang.height;
    // - Rectangularidad (usando caja de Feret)
        float areaRectang = rectang.width*rectang.height;
        imgDescriptors.rectangularidad = areaMax / areaRectang;
    // - Area del cierre convexo (pista: funcion convexHull) 
        vector<Point> salida;
        convexHull( contours[maxContour], salida );
        imgDescriptors.areaCierreConvexo = contourArea(salida);
    // - Centroide (X,Y) y orientacion (usando los momentos)
        // Extraemos los momentos para cada contorno
        Moments momentos = moments(contours[maxContour], false);
        imgDescriptors.centroide.x = momentos.m10/momentos.m00;
        imgDescriptors.centroide.y = momentos.m01/momentos.m00;
        imgDescriptors.orientacion = 0.5*atan2(2*momentos.mu11,momentos.mu20-momentos.mu02);
    // - Momentos de Hu
        //Moments momentos2 = moments(contours[0]);
        double hu[7];
        HuMoments(momentos, imgDescriptors.Hu); // El array hu contiene los 7 momentos.
        
       return imgDescriptors;
}

void calcularDistancias(const Descriptors &queryDescriptors, const Descriptors &imgDescriptors) {

   float dSC, dPer, dComp, dElong, dRect, dCierre, dCent, dOr, dHu;
   dSC = dPer = dComp = dElong = dRect = dCierre = dCent = dOr = dHu=0;

   // Calcular y devolver la distancia de:
   // - SC
        // Creamos una instancia de este descriptor
        Ptr <ShapeContextDistanceExtractor> mysc = createShapeContextDistanceExtractor();
        dSC = mysc->computeDistance(queryDescriptors.SC,imgDescriptors.SC);
   // - Perimetro
    dPer = abs(queryDescriptors.perimetro-imgDescriptors.perimetro);
   // - Compactacion
    dComp = abs(queryDescriptors.compactacion-imgDescriptors.compactacion);
   // - Elongacion
    dElong = abs(queryDescriptors.elongacion-imgDescriptors.elongacion);
   // - Rectangularidad
    dRect = abs(queryDescriptors.rectangularidad-imgDescriptors.rectangularidad);
   // - Area del cierre convexo
    dCierre = abs(queryDescriptors.areaCierreConvexo-imgDescriptors.areaCierreConvexo);
   // - Orientacion
    dOr = abs(queryDescriptors.orientacion-imgDescriptors.orientacion);
   // - Centroide (distancia Euclidea entre los dos puntos con coordenadas u,v)
    dCent = sqrt(pow(queryDescriptors.centroide.x-imgDescriptors.centroide.x,2)+pow(queryDescriptors.centroide.y-imgDescriptors.centroide.y,2));
   // - Hu (formula explicada en los siguientes parrafos del libro)
    double signA, signB;
    double ma, mb;
    int k = 0;
    float aux = 0;

    while(k<7){
        if(queryDescriptors.Hu[k] > 0)
            signA = 1;
        else if (queryDescriptors.Hu[k] < 0)
            signA = -1;
        else if (queryDescriptors.Hu[k] == 0)
            signA = 0;

        if(imgDescriptors.Hu[k] > 0)
            signB = 1;
        else if (imgDescriptors.Hu[k] < 0)
            signB = -1;
        else if (imgDescriptors.Hu[k] == 0)
            signB = 0;

        if(abs(queryDescriptors.Hu[k])>1*exp(-5) && abs(imgDescriptors.Hu[k])>1*exp(-5)){
            ma = signA * log10(abs(queryDescriptors.Hu[k]));
            mb = signB * log10(abs(imgDescriptors.Hu[k]));
            
            aux = aux + abs((1/mb) - (1/ma));
        }

        k++;
    }
    dHu = aux;
    
   cout << " dSC = " << dSC << endl;   
   cout << " dPer = " << dPer << endl;
   cout << " dComp = " << dComp << endl;
   cout << " dElong = " << dElong << endl;
   cout << " dRect = " << dRect << endl;
   cout << " dCierre = " << dCierre << endl;
   cout << " dCent = " << dCent << endl;
   cout << " dOr = " << dOr << endl;
   cout << " dHu = " << dHu << endl;
}

int main(int argc, char* argv[])
{
    // Procesamos parámetros de entrada
    string path = "shape_sample/";

    int indexQuery = 5;
    if (argc==2)
        indexQuery = atoi(argv[1]);

    // Leemos imagen query
    stringstream queryName;
    queryName << path << indexQuery << ".png";
    Mat query=imread(queryName.str(), IMREAD_GRAYSCALE);

    // Calculamos sus descriptores
    Descriptors queryDescriptors = extractDescriptors(query);

    // Para las otras imágenes, calculamos sus descriptores y los comparamos con los de la query
    for (int i=0; i<20; i++) {
        int imageIndex = i+1; 

        if (imageIndex==indexQuery) continue;  // Ignoramos esta imagen si es la misma que la de referencia

        // Leemos la imagen
        stringstream img;
        img << path << imageIndex << ".png";
        Mat image=imread(img.str(), IMREAD_GRAYSCALE);

        // Extraemos sus descriptores y los comparamos con los de la query
        cout << "-----------" << endl;
        cout << "Imagen " << imageIndex << ":" << endl;
        Descriptors imgDescriptors = extractDescriptors(image);
        calcularDistancias(queryDescriptors,imgDescriptors);
    }

    return 0;
}