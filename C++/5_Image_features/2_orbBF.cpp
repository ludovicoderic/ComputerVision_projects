#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <vector>

using namespace cv;
using namespace std;

const vector<string> labelNames= {"book", "cat", "chair", "dog", "glass", "laptop", "pen", "remote", "cellphone", "tv"};

// Lee las imagenes especificadas en un fichero de texto y las guarda en images, junto con sus etiquetas en labels.
// El ultimo parametro (max) podemos indicarlo cuando queramos que no se cargen todos los datos sino como mucho max imagenes (para hacer pruebas mas rapido)
void readData(string filename, vector<Mat> &images, vector<int> &labels, int max=-1)
{
   ifstream fi(filename.c_str());

   if (!fi.is_open()) {
     cerr << "Fichero " << filename << " no encontrado" << endl;
     exit(-1);
   }

   cout << "Cargando fichero " << filename << "..." << endl;

   string imagefn;
   string label;

   for (int i=0; i!=max && fi >> imagefn >> label; i++) {
       Mat image = imread(imagefn, IMREAD_GRAYSCALE);
       int label_u = distance(labelNames.begin(), find(labelNames.begin(), labelNames.end(), label));

       images.push_back(image); 
       labels.push_back(label_u);
   }
   fi.close();
}

// Funcion para extraer las caracteristicas ORB de una imagen
vector<Mat> extractORBFeatures(const vector<Mat> &data) 
{
     vector<Mat> features;
     // Recorremos el vector data, y para cada imagen (Mat) detectamos y extraemos caracteristicas ORB con 100 puntos como máximo por imagen.
     // Al final añadir todos todos los descriptores obtenidos de una imagen en un elemento del vector "features" (usando push_back).

    Ptr<FeatureDetector> detector = ORB::create(100);
    vector<KeyPoint> keypoints;
    Ptr<DescriptorExtractor> descriptor = detector;     //Descriptor
    Mat descriptors;

    for (int i = 0; i < data.size(); i++ ){
        detector->detect(data[i],keypoints); 
        descriptor->compute(data[i], keypoints, descriptors);
        features.push_back(descriptors);
    }
    
    return features;
}

// Extraemos y guardamos todas las caracteristicas en el fichero trainORBDescriptors.yml, para poder usarlas luego en la fase de reconocimiento
void storeTrainORBDescriptors(const vector<Mat> &train, const vector<int> &labelsTrain) {

        cout << "Extrayendo descriptores..." << endl;
        vector<Mat> features = extractORBFeatures(train);

        cout << "Guardando los descriptores en el fichero trainORBDescriptors.yml" << endl;

        FileStorage fs("trainORBDescriptors.yml", FileStorage::WRITE);
        fs << "ORBFeatures" << "[";
        for (unsigned i=0; i<train.size(); i++)  {
           fs << "{" << "label" << labelsTrain[i] << "descriptor" << features[i] << "}";
        }
        fs << "]";

        fs.release();
}

// Esta es la fase de reconocimiento de imagenes no vistas durante el entrenamiento. Para esto usamos el conjunto test.
void testORB(const vector<Mat> &test, const vector<int> &labelsTest)
{
       // Cargar fichero trainORBDescriptors.yml, guardando en trainORBDescriptors los descriptores de las imagenes de entrenamiento y en labelsTrain sus etiquetas.
       cout << "Cargando descriptores de entrenamiento" << endl;
       FileStorage fs("trainORBDescriptors.yml", FileStorage::READ); 

       vector<Mat> trainORBDescriptors;
       vector<int> labelsTrain;

       FileNode features = fs["ORBFeatures"];
       for (FileNodeIterator it = features.begin(); it!= features.end(); it++) {
           int label = (*it)["label"];
           Mat desc;
           (*it)["descriptor"] >> desc;
           trainORBDescriptors.push_back(desc);
           labelsTrain.push_back(label);
       }

       // Calculamos descriptores ORB del conjunto de test
       cout << "Calculando descriptores de test" << endl;
       vector<Mat> testORBDescriptors = extractORBFeatures(test);

       // Comparamos cada imagen de test con todas las de train para obtener su etiqueta. Ojo, a pesar de ser un descriptor binario veras que es lento.
       cout << "Matching..." << endl;
       vector<DMatch> matches;
       BFMatcher matcher(NORM_HAMMING);


       // Comparar los descriptores de cada imagen de test con todas las imagenes de entrenamiento.
       
       int ok=0;
       int predicted;

       int puntosSimilares;
       int maximoPuntosSimilares = 0;
       double distMin = 90;

       for (unsigned i = 0; i<testORBDescriptors.size(); i++) {
         for (unsigned j=0; j<trainORBDescriptors.size(); j++) {
            if (testORBDescriptors[i].cols!=0 && trainORBDescriptors[j].cols!=0) {  // Solo si el descriptor no es vacio
               // Solo consideramos que dos puntos son similares si su distancia es menor de 90. 
               // La imagen mas similar sera la que tiene mas keypoints coincidentes. Hay que extraer su etiqueta y guardarla en "predicted".
               matcher.match(testORBDescriptors[i], trainORBDescriptors[j], matches);

                
                for (unsigned k = 0; k < matches.size(); k++){
                    if(distMin >= matches[k].distance) //si supera la distancia minima añadimos un punto similar
                        puntosSimilares ++;
                }

                if(maximoPuntosSimilares <= puntosSimilares ){
                    predicted =  labelsTrain[j];
                    maximoPuntosSimilares = puntosSimilares;
                }
                puntosSimilares = 0;
            }
         }
         maximoPuntosSimilares = 0;
         cout << i << endl;
         // Si la etiqueta de la imagen de test coincide con la que deberia dar, entonces la prediccion se considera correcta..
         if (predicted == labelsTest[i])
           ok++; 
       }

       float accuracy = (float)ok/test.size();
       cout << "Accuracy=" << accuracy << endl;
}

int main(int argc, char *argv[])
{
    if (argc!=2 || (string(argv[1])!="train" && string(argv[1])!="test")) {
       cout << "Sintaxis: " << argv[0] << " <train/test>" << endl;
       exit(-1);
    }

    vector<Mat> train, test;
    vector<int> labelsTrain, labelsTest;

    if (string(argv[1]) == "train") {
        readData("train.txt", train, labelsTrain);
        storeTrainORBDescriptors(train, labelsTrain);
    }

    else {
        readData("test.txt", test, labelsTest);
        testORB(test, labelsTest);
    }
}