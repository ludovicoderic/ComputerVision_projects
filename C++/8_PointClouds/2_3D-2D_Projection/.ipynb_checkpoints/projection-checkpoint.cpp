#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
#include <vector>
#include <chrono>

using namespace cv;
using namespace std;
using namespace chrono;

int main(int argc, char **argv)
{
	int red = 0;
	int green = 260;
	int blue = 0;

	vector<Point2f>corners;
    Mat rotation_vector;
    Mat translation_vector;
	VideoCapture capture(0);
	Mat view;
	Mat Img;

	float cuadrosDeAncho = 9;//nº de cuadrados internos de ancho
	float cuadrosDeAlto = 6;//nº de cuadrados internos de alto
	Size patternSize(9,6);
	vector<Point3f>  objectPoints;
	bool found;
	vector<Point3d> point3D;
 	vector<Point2d> point2D;
	
	//RMS : 0.288 tiembla mucho mas , es mas inestable   //LA BUENA
	// Copiamos la matriz de valores intrínsecos del resultado de la calibración en el fichero out_camera_data.xml
   	Mat camera_matrix = (Mat_<double>(3,3) << 6.0792607673721375e+02, 0, 320, 0,
   	6.0792607673721375e+02, 240, 0, 0, 1.);
   	Mat dist_coeffs = (Mat_<double>(5,1) << 7.0294911765856227e-02, -1.7503463345050715e-01, 0, 0,
  	-3.0096854234150250e-01);
	//  //RMS:3.16


	//Para obtener el objectPoints (5)
	float tamCuadrados = 4;//side of a square length. Lo ponemos a 4 por comodidad, pero puede ser el que se quiera
	//Son el nº de unidades en 2d que tendra de lado el cuadrado
	for (int j=0; j<cuadrosDeAlto;j++)
		 for( int i=0; i < cuadrosDeAncho;i++)
		   objectPoints.push_back(Point3f(i*tamCuadrados,j*tamCuadrados,0));

	

	//Para nuestra función de calcular el RMS manual
	/*
	point3D.push_back(Point3d(0, 0, -1)); //10,5  0 0 -1
  	point3D.push_back(Point3d(4, 0, -1)); //20,5  4 0 -1
	point3D.push_back(Point3d(0, 4, -1)); //10,15 0 4 -1
  	point3D.push_back(Point3d(4, 4, -1)); //20,15 4 4 -1
	*/

	///*Puntos en el espacio para cada una de las orientaciones de nuestra figura
	point3D.push_back(Point3d(15, 10, 0));
	point3D.push_back(Point3d(15, 10, -18));
	point3D.push_back(Point3d(21, 10, -9));
	point3D.push_back(Point3d(16.86, 4.3, -9));
	point3D.push_back(Point3d(10.14, 6.46, -9));
	point3D.push_back(Point3d(10.14, 13.54, -9));
	point3D.push_back(Point3d(16.86, 15.7, -9));

	point3D.push_back(Point3d(18.84, 7.15, -9));
	point3D.push_back(Point3d(13.41, 5.38, -9));
	point3D.push_back(Point3d(10.14, 10, -9));
	point3D.push_back(Point3d(13.5, 14.62, -9));
	point3D.push_back(Point3d(18.93, 12.85, -9));

	point3D.push_back(Point3d(19.92, 8.575, -9));
	point3D.push_back(Point3d(15.045, 4.84, -9));
	point3D.push_back(Point3d(10.14, 8.23, -9));
	point3D.push_back(Point3d(11.82, 14.08, -9));
	point3D.push_back(Point3d(17.895, 14.275, -9));

	point3D.push_back(Point3d(17.85, 5.725, -9));
	point3D.push_back(Point3d(11.77, 5.92, -9));
	point3D.push_back(Point3d(10.14, 11.77, -9));
	point3D.push_back(Point3d(15.74, 15.16, -9));
	point3D.push_back(Point3d(19.965, 11.425, -9));
	//*/

	int romper = 0;
	int cambio = 0;
	/*
	cout<<endl;
	cout<<"==========================================================================="<<endl;
	float media1, media2, media3, media4, mediaF, mediaG;
	mediaG = 0;
	*/

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

    	if(found){  //si encontramos las esquinas del tablero

			
			//CornerSubpix necesita la imagen en grises (5)
			//Te "refina" / mejora la posicion de las esquinas del tablero
		    cornerSubPix(Img, corners, Size(11, 11), Size(-1, -1),
        	TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30,  0.1));




			//Devuelve los vectores de rotacion y traslacion tras transformar
			//un punto 3D a una coordenada de la camara en 2D
		    cv::solvePnP(objectPoints, corners, camera_matrix, dist_coeffs, rotation_vector, translation_vector);//Gives you rotation_vector, translation_vector

		    //Con los vectores de traslacion y rotacion obtenidos anteriormente podemos y las matrices intrinsecas de la cmara
        	//(3)projectar estos puntos en 3D en el plano de 2D de la imagen 
			cv::projectPoints(point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, point2D);
							  //pts                                                                   puntos de salida tras transformacion
			
			
			/*
			if(romper <= 20){
				
				media1 = sqrt(pow((corners[0].x)-(point2D[0].x),2)+pow((corners[0].y)-(point2D[0].y),2));
				media2 = sqrt(pow((corners[1].x)-(point2D[1].x),2)+pow((corners[1].y)-(point2D[1].y),2));
				media3 = sqrt(pow((corners[9].x)-(point2D[2].x),2)+pow((corners[9].y)-(point2D[2].y),2));
				media4 = sqrt(pow((corners[10].x)-(point2D[3].x),2)+pow((corners[10].y)-(point2D[3].y),2));
				mediaF = (media1+media2+media3+media4)/4;

				cout<<"Punto 0,0 :  ";
				cout<<"Projectado->  "<<point2D[0]<<"  Real->  "<<corners[0]<<endl;

				cout<<"Punto 4,0 :  ";
				cout<<"Projectado->  "<<point2D[1]<<"  Real->  "<<corners[1]<<endl;
				
				cout<<"Punto 0,4 :  ";
				cout<<"Projectado->  "<<point2D[2]<<"  Real->  "<<corners[9]<<endl;
				
				cout<<"Punto 4,4 :  ";
				cout<<"Projectado->  "<<point2D[3]<<"  Real->  "<<corners[10]<<endl;
				cout<<endl<<"Root Mean Square:  "<<mediaF<<endl;
				
				mediaG += mediaF;

				cout<<"==========================================================================="<<endl;
				
				romper++;

				
			}
			
			if(romper == 21){
				mediaG = mediaG/21;
				cout<<endl<<"RMS: "<<mediaG<<endl;
				romper++;
			}
			

			circle(view,point2D[0],10, cv::Scalar(142,68,173), -1, 8, 0);
			circle(view,point2D[1], 10, cv::Scalar(142,68,173), -1, 8, 0);
			circle(view,point2D[2], 10, cv::Scalar(142,68,173), -1, 8, 0);
			circle(view,point2D[3], 10, cv::Scalar(142,68,173), -1, 8, 0);
			*/

			///* Comprobaciones para el cambio gradual del color
			if(green <= 60){
				cambio = 1;
			}else if(green >= 255){
				cambio = 0;
			}
			if(cambio == 0){
				green = green - 10;
			}else if(cambio == 1){
				green = green + 10;
			}

			//Si se ha ejecutado ./projection movil el programa te va mostrando cada 2 iteraciones del buble distintas orientaciones 
			if(argc == 2 && string(argv[1]) == "movil"){
				if(romper == 0){
					line(view,point2D[0],point2D[2], cv::Scalar(blue, green, red),6);
					line(view,point2D[0],point2D[3], cv::Scalar(blue, green, red),6);
					line(view,point2D[0],point2D[4], cv::Scalar(blue, green, red),6);
					line(view,point2D[0],point2D[5], cv::Scalar(blue, green, red),6);
					line(view,point2D[0],point2D[6], cv::Scalar(blue, green, red),6);
					line(view,point2D[1],point2D[2], cv::Scalar(blue, green, red),6);
					line(view,point2D[1],point2D[3], cv::Scalar(blue, green, red),6);
					line(view,point2D[1],point2D[4], cv::Scalar(blue, green, red),6);
					line(view,point2D[1],point2D[5], cv::Scalar(blue, green, red),6);
					line(view,point2D[1],point2D[6], cv::Scalar(blue, green, red),6);
					line(view,point2D[2],point2D[3], cv::Scalar(blue, green, red),6);
					line(view,point2D[3],point2D[4], cv::Scalar(blue, green, red),6);
					line(view,point2D[4],point2D[5], cv::Scalar(blue, green, red),6);
					line(view,point2D[5],point2D[6], cv::Scalar(blue, green, red),6);
					line(view,point2D[6],point2D[2], cv::Scalar(blue, green, red),6);
					romper++;

				}else if(romper <= 2){
				
					line(view,point2D[0],point2D[12], cv::Scalar(blue, green, red),6);
					line(view,point2D[0],point2D[13], cv::Scalar(blue, green, red),6);
					line(view,point2D[0],point2D[14], cv::Scalar(blue, green, red),6);
					line(view,point2D[0],point2D[15], cv::Scalar(blue, green, red),6);
					line(view,point2D[0],point2D[16], cv::Scalar(blue, green, red),6);
					line(view,point2D[1],point2D[12], cv::Scalar(blue, green, red),6);
					line(view,point2D[1],point2D[13], cv::Scalar(blue, green, red),6);
					line(view,point2D[1],point2D[14], cv::Scalar(blue, green, red),6);
					line(view,point2D[1],point2D[15], cv::Scalar(blue, green, red),6);
					line(view,point2D[1],point2D[16], cv::Scalar(blue, green, red),6);
					line(view,point2D[12],point2D[13], cv::Scalar(blue, green, red),6);
					line(view,point2D[13],point2D[14], cv::Scalar(blue, green, red),6);
					line(view,point2D[14],point2D[15], cv::Scalar(blue, green, red),6);
					line(view,point2D[15],point2D[16], cv::Scalar(blue, green, red),6);
					line(view,point2D[16],point2D[12], cv::Scalar(blue, green, red),6);
					romper++;

				}else if(romper <= 4){
					line(view,point2D[0],point2D[7], cv::Scalar(blue, green, red),6);
					line(view,point2D[0],point2D[8], cv::Scalar(blue, green, red),6);
					line(view,point2D[0],point2D[9], cv::Scalar(blue, green, red),6);
					line(view,point2D[0],point2D[10], cv::Scalar(blue, green, red),6);
					line(view,point2D[0],point2D[11], cv::Scalar(blue, green, red),6);
					line(view,point2D[1],point2D[7], cv::Scalar(blue, green, red),6);
					line(view,point2D[1],point2D[8], cv::Scalar(blue, green, red),6);
					line(view,point2D[1],point2D[9], cv::Scalar(blue, green, red),6);
					line(view,point2D[1],point2D[10], cv::Scalar(blue, green, red),6);
					line(view,point2D[1],point2D[11], cv::Scalar(blue, green, red),6);
					line(view,point2D[7],point2D[8], cv::Scalar(blue, green, red),6);
					line(view,point2D[8],point2D[9], cv::Scalar(blue, green, red),6);
					line(view,point2D[9],point2D[10], cv::Scalar(blue, green, red),6);
					line(view,point2D[10],point2D[11], cv::Scalar(blue, green, red),6);
					line(view,point2D[11],point2D[7], cv::Scalar(blue, green, red),6);
					romper++;

				}else if(romper <= 6){

					line(view,point2D[0],point2D[17], cv::Scalar(blue, green, red),6);
					line(view,point2D[0],point2D[18], cv::Scalar(blue, green, red),6);
					line(view,point2D[0],point2D[19], cv::Scalar(blue, green, red),6);
					line(view,point2D[0],point2D[20], cv::Scalar(blue, green, red),6);
					line(view,point2D[0],point2D[21], cv::Scalar(blue, green, red),6);
					line(view,point2D[1],point2D[17], cv::Scalar(blue, green, red),6);
					line(view,point2D[1],point2D[18], cv::Scalar(blue, green, red),6);
					line(view,point2D[1],point2D[19], cv::Scalar(blue, green, red),6);
					line(view,point2D[1],point2D[20], cv::Scalar(blue, green, red),6);
					line(view,point2D[1],point2D[21], cv::Scalar(blue, green, red),6);
					line(view,point2D[17],point2D[18], cv::Scalar(blue, green, red),6);
					line(view,point2D[18],point2D[19], cv::Scalar(blue, green, red),6);
					line(view,point2D[19],point2D[20], cv::Scalar(blue, green, red),6);
					line(view,point2D[20],point2D[21], cv::Scalar(blue, green, red),6);
					line(view,point2D[21],point2D[17], cv::Scalar(blue, green, red),6);
					romper++;

					if(romper == 7){
						romper = 0;
					}

				} 
				//si es el fijo, constantemente te muestra la misma proyección
			}else if(argc == 1 || string(argv[1]) == "fijo"){
				line(view,point2D[0],point2D[2], cv::Scalar(blue, green, red),6);
				line(view,point2D[0],point2D[3], cv::Scalar(blue, green, red),6);
				line(view,point2D[0],point2D[4], cv::Scalar(blue, green, red),6);
				line(view,point2D[0],point2D[5], cv::Scalar(blue, green, red),6);
				line(view,point2D[0],point2D[6], cv::Scalar(blue, green, red),6);
				line(view,point2D[1],point2D[2], cv::Scalar(blue, green, red),6);
				line(view,point2D[1],point2D[3], cv::Scalar(blue, green, red),6);
				line(view,point2D[1],point2D[4], cv::Scalar(blue, green, red),6);
				line(view,point2D[1],point2D[5], cv::Scalar(blue, green, red),6);
				line(view,point2D[1],point2D[6], cv::Scalar(blue, green, red),6);
				line(view,point2D[2],point2D[3], cv::Scalar(blue, green, red),6);
				line(view,point2D[3],point2D[4], cv::Scalar(blue, green, red),6);
				line(view,point2D[4],point2D[5], cv::Scalar(blue, green, red),6);
				line(view,point2D[5],point2D[6], cv::Scalar(blue, green, red),6);
				line(view,point2D[6],point2D[2], cv::Scalar(blue, green, red),6);
			}
			
			// line: pinta una linea sobre la imagen, de un punto a otro de color
			//Scalar y grosor 6
			//     imagen, puntos      			 color,            grosor 
			//*/


		
		}

		//Mostrar en ventana emergente la salida de la camara con los 
		//patrones de lineas y circulos previamente definidos (1)
		cv::flip(view, view, 1); //hacemos flip vertical de la imagen
    	cv::imshow("Output", view);

		//Para mostrar la ventana hasta que el usuario presione una tecla, si
		//no lo pusieramos acabaria muy rapido el programa, cero significa esperar
		//para siempre.
		cv::waitKey(1);	
  	}
	
}
