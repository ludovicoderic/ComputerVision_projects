#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <vector>

using namespace cv;
using namespace std;
using namespace cv::ml;

const vector<string> labelNames = {"book", "cat", "chair", "dog", "glass", "laptop", "pen", "remote", "cellphone", "tv"};

// Lee las imagenes especificadas en un fichero de texto y las guarda en images, junto con sus etiquetas en labels.
// El ultimo parametro (max) podemos indicarlo cuando queramos que no se cargen todos los datos sino solo hasta max imagenes (para hacer pruebas mas rapido)
void readData(string filename, vector<Mat> &images, vector<int> &labels, int max=-1)
{
   ifstream fi(filename.c_str());

   if (!fi.is_open()) {
     cerr << "Fichero " << filename << " no encontrado" << endl;
     exit(-1);
   }

   cout << "Cargando " << filename << "..." << endl;

   string imagefn;
   string label;

   for (int i=0; i!=max && fi >> imagefn >> label; i++) {
       Mat image = imread(imagefn,IMREAD_GRAYSCALE);
       int label_u = distance(labelNames.begin(), find(labelNames.begin(), labelNames.end(), label));

       images.push_back(image); 
       labels.push_back(label_u);
   }
   fi.close();
}

// Extraemos los descriptores HOG (tendremos un vector<float> por cada imagen) del conjunto recibido por parametro
vector<vector<float> > extractHOGFeatures(const vector<Mat> &data)
{
     vector<vector<float> > features;
     HOGDescriptor hog;

     Ptr<FeatureDetector> detector = ORB::create(100);
	 vector<Point> location;
	 vector<float> descriptor;

     for (unsigned i = 0; i<data.size(); i++) {
           Mat image = data[i];

           // Para que todas las imagenes tengan el mismo tamanyo de descriptor, las debemos escalar a un tamanyo 128x128 
            resize(image, image, Size(128,128)); // El ultimo parametro es opcional
           // Ahora debemos calcular el descriptor HOG con un stride de 128x128 (asumimos que el objeto ocupa toda la imagen) y un padding (0,0).
            hog.compute(image, descriptor, Size(128,128), Size(0,0), location);
           // Anyadimos el descriptor obtenido de la imagen al vector features con push_back.
            features.push_back(descriptor);
     }
     return features;
}

// Funcion para convertir un vector<vector<float> > a una matriz Mat, ya que nos hace falta este formato para nuestro clasificador
Mat convertToMat(vector<vector<float> > features)
{
    int descriptor_size = features[0].size();
    Mat trainMat(features.size(), descriptor_size, CV_32F);
    for(int i=0; i<features.size(); i++){
        for(int j = 0; j<descriptor_size; j++){
           trainMat.at<float>(i,j) = features[i][j];
        }
    }
    return trainMat;
}

// Entrenamiento de nuestro clasificador, en este caso un SVM (support vector machine).
void trainHOGSVM(const vector<Mat> &train, const vector<int> &labelsTrain) {

    cout << "Extrayendo descriptores..." << endl;
    vector<vector<float> > features = extractHOGFeatures(train);

    // Convertimos a Mat
    Mat trainMat = convertToMat(features);
    Mat dataLabel(labelsTrain);

    // Configuramos el clasificador SVM
    cout << "Entrenando..." << endl;
    Ptr<SVM> svm= SVM::create();
    // Debemos poner el clasificador con el tipo C_SVC
    svm->setType(100);
    // kernel LINEAR
    svm->setKernel(0);
    // El criterio de finalización debe ser MAX_ITER con 100 iteraciones maximas y EPS=1e-6.
    svm->setTermCriteria(TermCriteria(1, 100, 0.000001));
    // Ayuda para los puntos anteriores: https://docs.opencv.org/3.3.0/d1/d2d/classcv_1_1ml_1_1SVM.html

    // Entrenamos SVM con trainMat y dataLabel
    Ptr<TrainData> data= TrainData::create(trainMat, ROW_SAMPLE, dataLabel);
    svm->train(data);

    // Guardamos el modelo en un fichero.
    svm->save("modelSVM.xml");
}

// Probamos el modelo SVM con el conjunto de test
void testHOGSVM(const vector<Mat> &test, const vector<int> &labelsTest)
{
    cout << "Extrayendo descriptores..." << endl;
    vector<vector<float> > features = extractHOGFeatures(test);
    Mat testMat = convertToMat(features);
    Mat dataLabel(labelsTest);

    // Cargamos modelo
    cout << "Cargando modelo..." << endl;
    Ptr<SVM> svm = Algorithm::load<SVM>("modelSVM.xml");

    // Hacemos el reconocimiento
    cout << "Clasificando muestras..." << endl;
    Mat testResponse;
    svm->predict(testMat, testResponse);

    // Calculamos accuracy
    int ok=0;
    for(int i=0; i<testResponse.rows; i++) {
        int predicted = testResponse.at<float>(i,0);
        if (predicted==labelsTest[i]){
            ok++;
        }
    }
    float accuracy = ((float)ok/testResponse.rows);
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
        trainHOGSVM(train, labelsTrain);
    }
    else {
        readData("test.txt", test, labelsTest);
        testHOGSVM(test, labelsTest); 
    }
}