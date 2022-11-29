#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
#include <vector>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <boost/algorithm/string.hpp>

using namespace cv;
using namespace std;

int main(int argc, char **argv){

	vector<Point2f>corners;
    Mat rotation_vector;
    Mat translation_vector;
	VideoCapture capture(0); //la de la webcam
	Mat view;
	Mat Img;

	cv::Scalar AZUL = cv::Scalar(126,58,5);
	cv::Scalar NARANJA = cv::Scalar(0,108,255);

	float cuadrosDeAncho = 9;//nº de cuadrados internos de ancho
	float cuadrosDeAlto = 6;//nº de cuadrados internos de alto
	Size patternSize(9,6);
	vector<Point3f>  objectPoints;
	bool found;
 	vector<Point2d> point2D;
	
	
	///*
	//RMS : 0.314 tiembla mucho mas , es mas inestable   //LA BUENA
	// Copiamos la matriz de valores intrínsecos del resultado de la calibración en el fichero out_camera_data.xml
   	Mat camera_matrix = (Mat_<double>(3,3) << 6.1860957395569835e+02, 0, 320, 0,
   	6.1860957395569835e+02, 240, 0, 0, 1.);
   	Mat dist_coeffs = (Mat_<double>(5,1) << 8.9999637566697214e-03, -1.4757930458773030e-01, 0, 0,
  	-4.0042983080322191e-02);
	//*/

	
	//Creamos y guardamos la nube de puntos
	std::string pcdFile;
	if(argc > 1)
	{
		pcdFile = std::string(argv[1]);
    }

	pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud (new pcl::PointCloud<pcl::PointXYZ>);

	//si al cargar la nube, da un error, se acaba el programa
	if(pcl::io::loadPCDFile<pcl::PointXYZ> (pcdFile, *pointCloud) == -1){
		std::cerr<<"Error al cargar la nube de puntos "+pcdFile+"\n"<<std::endl;
		cout<<"No se ha cargado la nube de putos."<<endl<<"El comando de ejecucion ha de ser: ./projection nombreNubePuntos.pcd"<<endl;
		return -1;
	}
	vector<int> indices1;

	//eliminamos NaN's
	pcl::removeNaNFromPointCloud(*pointCloud, *pointCloud, indices1);


	//Creamos un vector 3D y le pasamos a ese vector las coordenadas de los 
	//puntos de la nube. 
	vector<Point3d> pts;
  	pts.resize (pointCloud->points.size());

	//transformaciones necesarias para la proyeccion dependiendo de que nube de puntos se haya cargado
	if(string(argv[1]) == "conejo.pcd"){
   		for (size_t i=0; i<pointCloud->points.size(); i++) { 
    		pts[i].x = (pointCloud->points[i].x/3.5)+15;   //los +X del final es para desplazar en su eje
       		pts[i].y = -(pointCloud->points[i].y/3.5)+10;  // los /X es para reducir el tamaño
			pts[i].z = -(pointCloud->points[i].z/3.5);    //y los -() son para invertir los puntos sobre su eje
		}
	}else if(string(argv[1]) == "digi.pcd"){
		for (size_t i=0; i<pointCloud->points.size(); i++){
    		pts[i].x = (pointCloud->points[i].x/3.5)+8;       													
       		pts[i].y = (pointCloud->points[i].z/3.5)+1;
			pts[i].z = -(pointCloud->points[i].y/3.5);      	
		}
		
	}else{
		cout<<"No existe la nube de puntos: " + string(argv[1]) <<endl;
		return -1;
	}


	//Para obtener el objectPoints (5)
	float tamCuadrados = 4;//side of a square length. Es orientativo, es para poner lo que mide el lado del tablero, un numero bajo es mas comodo que la distancia original
	//Son el nº de unidades en 2d que tendra de lado el cuadrado
	for (int j=0; j<cuadrosDeAlto;j++)
		 for( int i=0; i < cuadrosDeAncho;i++)
		   objectPoints.push_back(Point3f(i*tamCuadrados,j*tamCuadrados,0));


	
	while(1){
		
		capture >> view;

		//pasamos a escala de grises la imagen capturada por la camara y lo guardamos
		//en Img. Necesario para las siguientes funciones. (5)
		cvtColor(view,Img,CV_BGR2GRAY);

		 
		//Busca la posicion de las esquinas internas del tablero y las guarda en
		//corners (5) 
		found=findChessboardCorners(Img, patternSize,  corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
        	+ CALIB_CB_FAST_CHECK);//This will detect pattern
			//La imagen de la camara, nºde esquinas internas y externas, vector de salida

    	if(found){  //si detecta las esquinas del tablero

			
			//CornerSubpix necesita la imagen en grises (5)
			//Te "refina" / mejora la posicion de las esquinas del tablero
		    cornerSubPix(Img, corners, Size(11, 11), Size(-1, -1),
        	TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

			//Devuelve los vectores de rotacion y traslacion tras transformar
			//un punto 3D a una coordenada de la camara en 2D
		    cv::solvePnP(objectPoints, corners, camera_matrix, dist_coeffs, rotation_vector, translation_vector);//Gives you rotation_vector, translation_vector

		    //Con los vectores de traslacion y rotacion obtenidos anteriormente podemos
        	//(3)projectar estos puntos en 3D en el plano de 2D de la imagen . Le pasamos las coordenadas de la nube
			cv::projectPoints(pts, rotation_vector, translation_vector, camera_matrix, dist_coeffs, point2D);
							  //pts                                                                   puntos de salida tras transformacion
			

			//Dependiendo de la nube de puntos se representa de una manera u otra
			//Pero en general para cada punto del vector 2D se le dibuja un circulo sobre dicho punto
			if(string(argv[1]) == "conejo.pcd"){
				for(signed i = 0; i<point2D.size(); i++){
					circle(view,point2D[i], 1, NARANJA, 1, 8, 0); //Azul 126,58,5
				}
			}else if(string(argv[1]) == "digi.pcd"){
				for(signed i = 0; i<point2D.size(); i++){
					circle(view,point2D[i], 1, NARANJA, 1, 8, 0); //Azul 126,58,5
				}
			}
		}

		//Mostrar en ventana emergente la salida de la camara con los 
		//patrones de lineas y circulos previamente definidos (1)
		cv::flip(view, view, 1);  //flip vertical de la imagen
    	//cv::namedWindow("Display frame", WINDOW_NORMAL);
		cv::imshow("Display frame", view);

		//Para mostrar la ventana hasta que el usuario presione una tecla, si
		//no lo pusieramos acabaria muy rapido el programa, cero significa esperar
		//para siempre.
		cv::waitKey(1);
  	}

}

