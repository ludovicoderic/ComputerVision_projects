#include <iostream>
#include <vector>
#include <boost/thread/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <math.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/keypoints/susan.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/console/parse.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/search/kdtree.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/eigen.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/keypoints/iss_3d.h>
#include <chrono>

using namespace std;
using namespace pcl;
using namespace chrono;

//Funcion para Voxelizar una nube
void applyVoxelGrid(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& inputCloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& outputCloud, const double& leaf)
{
	pcl::VoxelGrid<pcl::PointXYZRGBA> voxelgrid_filter;
	voxelgrid_filter.setInputCloud (inputCloud);			
	voxelgrid_filter.setLeafSize (leaf, leaf, leaf);		
	voxelgrid_filter.filter (*outputCloud);		
}

//Generar matriz de escalado
Eigen::Affine3f generateScaleMatrix(const Eigen::Vector3f& scale)
{
	Eigen::Affine3f transform(Eigen::Affine3f::Identity());
	transform = Eigen::Scaling(scale);
	return transform;
}

//Generar matriz de Rotacion
Eigen::Affine3f generateRotationMatrix(const Eigen::Vector3f& rotation)
{
	Eigen::Affine3f transform(Eigen::Affine3f::Identity());
	transform.rotate (Eigen::AngleAxisf (rotation[0], Eigen::Vector3f::UnitZ())); 
	transform.rotate (Eigen::AngleAxisf (rotation[1], Eigen::Vector3f::UnitY()));
	transform.rotate (Eigen::AngleAxisf (rotation[2], Eigen::Vector3f::UnitX()));
	return transform;
}

//Funcion para pasar el algoritmo ransac
void RANSAC (PointCloud<PointXYZRGBA>::Ptr& pointCloud, float threshold)
{

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  	pcl::PointIndices::Ptr inliners (new pcl::PointIndices ());

    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
	pcl::SACSegmentation<pcl::PointXYZRGBA> segm; //creamos el objeto de segmentacion
	segm.setOptimizeCoefficients (true);
    segm.setMethodType (pcl::SAC_RANSAC);   //seleccionamos el metodo
	segm.setModelType (pcl::SACMODEL_PLANE);  //seleccionamos el modelo
	segm.setMaxIterations (1000);
    segm.setDistanceThreshold (threshold);
	segm.setInputCloud (pointCloud);
    segm.segment (*inliners, *coefficients);

    extract.setInputCloud (pointCloud); //extraer inliers
    extract.setIndices (inliners);
    extract.setNegative (true);
    extract.filter (*pointCloud);
}

//Calcular la resolucion de la escena
double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pointCloud)
{
	double resolution = 0.0;
	int numberOfPoints = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> squaredDistances(2);
	pcl::search::KdTree<pcl::PointXYZ> tree;
	tree.setInputCloud(pointCloud);

	for (size_t i = 0; i < pointCloud->size(); ++i)
	{
		// Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
		if (nres == 2)
		{
			resolution += sqrt(squaredDistances[1]);
			++numberOfPoints;
		}
	}
	if (numberOfPoints != 0)
		resolution /= numberOfPoints;

	return resolution;
}


int main (int argc, char** argv)
{
	auto begin = high_resolution_clock::now();
	string pcdFile = "../scenes/snap_0point.pcd";
	string pcdFile2 = "../objects/s0_plant_corr.pcd";
	string pcdFile3 = "../objects/s0_piggybank_corr.pcd";
	string pcdFile4 = "../objects/s0_plc_corr.pcd";
	string pcdFile5 = "../objects/s0_mug_corr.pcd";

	// Cargamos las nubes de puntos
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloudScene (new pcl::PointCloud<pcl::PointXYZRGBA>);
	
	if(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (pcdFile, *pointCloudScene) == -1)
	{
		std::cerr<<"Error al cargar la nube de puntos "+pcdFile+"\n"<<std::endl;
		return -1;
	}

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr visualizerPointCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

	copyPointCloud(*pointCloudScene, *visualizerPointCloud);

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloudObject (new pcl::PointCloud<pcl::PointXYZRGBA>);
	
	if(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (pcdFile2, *pointCloudObject) == -1)
	{
		std::cerr<<"Error al cargar la nube de puntos "+pcdFile2+"\n"<<std::endl;
		return -1;
	}

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloudObject2 (new pcl::PointCloud<pcl::PointXYZRGBA>);
	
	if(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (pcdFile3, *pointCloudObject2) == -1)
	{
		std::cerr<<"Error al cargar la nube de puntos "+pcdFile3+"\n"<<std::endl;
		return -1;
	}

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloudObject3 (new pcl::PointCloud<pcl::PointXYZRGBA>);
	
	if(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (pcdFile4, *pointCloudObject3) == -1)
	{
		std::cerr<<"Error al cargar la nube de puntos "+pcdFile4+"\n"<<std::endl;
		return -1;
	}

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloudObject4 (new pcl::PointCloud<pcl::PointXYZRGBA>);
	
	if(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (pcdFile5, *pointCloudObject4) == -1)
	{
		std::cerr<<"Error al cargar la nube de puntos "+pcdFile5+"\n"<<std::endl;
		return -1;
	}


    //ekiminamos los NaN de las escena para evitar problemas
	vector<int> indices1;
	vector<int> indices2;
	vector<int> indices3;
	vector<int> indices4;
	vector<int> indices5;

	pcl::removeNaNFromPointCloud(*pointCloudScene, *pointCloudScene, indices1);
	pcl::removeNaNFromPointCloud(*pointCloudObject, *pointCloudObject, indices2);
	pcl::removeNaNFromPointCloud(*pointCloudObject2, *pointCloudObject2, indices3);
	pcl::removeNaNFromPointCloud(*pointCloudObject3, *pointCloudObject3, indices4);
	pcl::removeNaNFromPointCloud(*pointCloudObject4, *pointCloudObject4, indices5);

	//Eliminamos los planos dominantes de la escena
	RANSAC(pointCloudScene,0.09);
	RANSAC(pointCloudScene,0.04);
	RANSAC(pointCloudScene,0.02);
	
	//Hacemos copias de la escena en formatos distintos para poder pasarles los distintos metodos
    //XYZ ISS/SIFT XYZRGBA SUSAN XYZI HARRIS
	pcl::PointCloud<pcl::PointXYZ>::Ptr copyPointCloudSceneXYZ (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr copyPointCloudObjectXYZ (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr copyPointCloudObject2XYZ (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr copyPointCloudObject3XYZ (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr copyPointCloudObject4XYZ (new pcl::PointCloud<pcl::PointXYZ>);

    //copiamos las nubes ya que al copiarlas se modifica solo el formato
	copyPointCloud(*pointCloudScene, *copyPointCloudSceneXYZ);
	copyPointCloud(*pointCloudObject, *copyPointCloudObjectXYZ);
	copyPointCloud(*pointCloudObject2, *copyPointCloudObject2XYZ);
	copyPointCloud(*pointCloudObject3, *copyPointCloudObject3XYZ);
	copyPointCloud(*pointCloudObject4, *copyPointCloudObject4XYZ);  


	//Extraemos los keypoints de la escena

	//Variables de la resolucion de la snubes
	double resolutionScene = computeCloudResolution(copyPointCloudSceneXYZ);
	double resolutionObject = computeCloudResolution(copyPointCloudObjectXYZ);
	double resolutionObject2 = computeCloudResolution(copyPointCloudObject2XYZ);
	double resolutionObject3 = computeCloudResolution(copyPointCloudObject3XYZ);
	double resolutionObject4 = computeCloudResolution(copyPointCloudObject4XYZ);

	//Variables para los keypoints de ISS Son XYZ porque está puesto para ISS
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsScene (new pcl::PointCloud<pcl::PointXYZ>);    
  	pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsObject (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsObject2 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsObject3 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsObject4 (new pcl::PointCloud<pcl::PointXYZ>);		

	

	//SUSAN

	//Keypoints para SUSAN. Si quieres probarlo, comenta los anteriores
	/*
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keypointsScene (new pcl::PointCloud<pcl::PointXYZRGBA> ());
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keypointsObject (new pcl::PointCloud<pcl::PointXYZRGBA> ());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keypointsObject2 (new pcl::PointCloud<pcl::PointXYZRGBA> ());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keypointsObject3 (new pcl::PointCloud<pcl::PointXYZRGBA> ());
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keypointsObject4 (new pcl::PointCloud<pcl::PointXYZRGBA> ());
    */

	/*
	pcl::SUSANKeypoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA>* susan3D = new pcl::SUSANKeypoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA>;
	susan3D->setInputCloud(copyPointCloudSceneXYZ);
	susan3D->setNonMaxSupression(false);
	susan3D->compute(*keypointsScene);

	pcl::SUSANKeypoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA>* susan3D2 = new pcl::SUSANKeypoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA>;
	susan3D2->setInputCloud(copyPointCloudObjectXYZ);
	susan3D2->setNonMaxSupression(false);
	susan3D2->compute(*keypointsObject);
    
    pcl::SUSANKeypoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA>* susan3D3 = new pcl::SUSANKeypoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA>;
	susan3D3->setInputCloud(copyPointCloudObject2XYZ);
	susan3D3->setNonMaxSupression(false);
	susan3D3->compute(*keypointsObject2);

    pcl::SUSANKeypoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA>* susan3D4 = new pcl::SUSANKeypoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA>;
	susan3D4->setInputCloud(copyPointCloudObject3XYZ);
	susan3D4->setNonMaxSupression(false);
	susan3D4->compute(*keypointsObject3);

    pcl::SUSANKeypoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA>* susan3D5 = new pcl::SUSANKeypoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA>;
	susan3D5->setInputCloud(copyPointCloudObject4XYZ);
	susan3D5->setNonMaxSupression(false);
	susan3D5->compute(*keypointsObject4);
    */



	//ISS algoritmo

	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>); //cremos el KdTree
	pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ>  detector; //Creamos el objeto para ISS

	detector.setInputCloud(copyPointCloudSceneXYZ);
	detector.setSearchMethod(kdtree); //metodo de busqueda
	detector.setSalientRadius(6 * resolutionScene); //radio del saliente
	detector.setNonMaxRadius(4 * resolutionScene); //radio del non max supresion
	detector.setMinNeighbors(5); //minimo numero de vecinos
	detector.setThreshold21(0.45); //Umbrales 
	detector.setThreshold32(0.45);
	detector.setNumberOfThreads(4); //numeor de hilos
  	detector.compute(*keypointsScene);

	//Objetos
	detector.setInputCloud(copyPointCloudObjectXYZ);
	detector.setSearchMethod(kdtree);
	detector.setSalientRadius(6 * resolutionObject);
	detector.setNonMaxRadius(4 * resolutionObject);
	detector.setMinNeighbors(5); //10 Planta
	detector.setThreshold21(0.45); //0.45 
	detector.setThreshold32(0.45); //0.45 
	detector.setNumberOfThreads(4); //4 
	detector.compute(*keypointsObject);
	//objeto 2
	detector.setInputCloud(copyPointCloudObject2XYZ);
	detector.setSearchMethod(kdtree);
	detector.setSalientRadius(6 * resolutionObject2);
	detector.setNonMaxRadius(4 * resolutionObject2);
	detector.setMinNeighbors(5); //10 piggybank
	detector.setThreshold21(0.45); //0.45 piggybank
	detector.setThreshold32(0.45); //0.45 piggybank
	detector.setNumberOfThreads(4); //4 piggybank
	detector.compute(*keypointsObject2);
	//Objetos 3
	detector.setInputCloud(copyPointCloudObject3XYZ);
	detector.setSearchMethod(kdtree);
	detector.setSalientRadius(6 * resolutionObject3);
	detector.setNonMaxRadius(4 * resolutionObject3);
	detector.setMinNeighbors(1); //10 plc
	detector.setThreshold21(0.45); //0.45 plc
	detector.setThreshold32(0.45); //0.45 plc
	detector.setNumberOfThreads(4); //4 
	detector.compute(*keypointsObject3);
	//objeto 4
	detector.setInputCloud(copyPointCloudObject4XYZ);
	detector.setSearchMethod(kdtree);
	detector.setSalientRadius(6 * resolutionObject4);
	detector.setNonMaxRadius(4 * resolutionObject4);
	detector.setMinNeighbors(2); //10 taza
	detector.setThreshold21(0.45); //0.45 taza
	detector.setThreshold32(0.45); //0.45 
	detector.setNumberOfThreads(4); //4 
	detector.compute(*keypointsObject4);
	


	//HARRIS 3D
	/*
	pcl::PointCloud<pcl::PointXYZI>::Ptr keypointsScene (new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr keypointsObject (new pcl::PointCloud<pcl::PointXYZI>());

	//Escena
	pcl::HarrisKeypoint3D <PointXYZ, PointXYZI> scenedetector; //PointXYZ,PointXYZI

	scenedetector.setNonMaxSupression (true);
	scenedetector.setRadius(0.02);
	scenedetector.setRadiusSearch(0.02);
	scenedetector.setInputCloud (copyPointCloudSceneXYZ);
	scenedetector.compute (*keypointsScene);

	//Objeto
	pcl::HarrisKeypoint3D <PointXYZ, PointXYZI> objectdetector2; //PointXYZ,PointXYZI

 	objectdetector2.setNonMaxSupression (true);
	objectdetector2.setRadius(0.01);
	objectdetector2.setRadiusSearch(0.01);
 	objectdetector2.setInputCloud (copyPointCloudObjectXYZ);
 	objectdetector2.compute (*keypointsObject);

     //Objeto
	pcl::HarrisKeypoint3D <PointXYZ, PointXYZI> objectdetector3; //PointXYZ,PointXYZI

 	objectdetector3.setNonMaxSupression (true);
	objectdetector3.setRadius(0.01);
	objectdetector3.setRadiusSearch(0.01);
 	objectdetector3.setInputCloud (copyPointCloudObject2XYZ);
 	objectdetector3.compute (*keypointsObject2);

     //Objeto
	pcl::HarrisKeypoint3D <PointXYZ, PointXYZI> objectdetector4; //PointXYZ,PointXYZI

 	objectdetector4.setNonMaxSupression (true);
	objectdetector4.setRadius(0.01);
	objectdetector4.setRadiusSearch(0.01);
 	objectdetector4.setInputCloud (copyPointCloudObject3XYZ);
 	objectdetector4.compute (*keypointsObject3);

     //Objeto
	pcl::HarrisKeypoint3D <PointXYZ, PointXYZI> objectdetector5; //PointXYZ,PointXYZI

 	objectdetector5.setNonMaxSupression (true);
	objectdetector5.setRadius(0.01);
	objectdetector5.setRadiusSearch(0.01);
 	objectdetector5.setInputCloud (copyPointCloudObject4XYZ);
 	objectdetector5.compute (*keypointsObject4);
	*/

	/*

	if(keypointsScene->points.size() != 0)
		cout <<endl<< "The scene has " << copyPointCloudSceneXYZ->size() <<" points. "<< "Detected keypoints: " << keypointsScene->size() <<endl;
    
	if(keypointsObject->points.size() != 0)
		cout << "Object Plant has " << copyPointCloudObjectXYZ->size() <<" points. "<< "Detected keypoints: " << keypointsObject->size() << endl;

	if(keypointsObject2->points.size() != 0)
		cout << "Object Piggy bank has " << copyPointCloudObject2XYZ->size() <<" points. "<< "Detected keypoints: " << keypointsObject2->size() << endl;

	if(keypointsObject3->points.size() != 0)
		cout << "Object PLC has " << copyPointCloudObject3XYZ->size() <<" points."<< "Detected keypoints: " << keypointsObject3->size() << endl;

	if(keypointsObject4->points.size() != 0)
		cout << "Object Mug has " << copyPointCloudObject4XYZ->size() <<" points."<< "Detected keypoints: " << keypointsObject4->size() << endl<<endl;
	*/
	
    
    //Para ver los keypoints sobre los objetos originales
	/*
	pcl::visualization::PCLVisualizer viewer2("PCL Viewer");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler2 (keypointsScene, 255, 0, 0);
	viewer2.setBackgroundColor( 0, 0, 0);
	viewer2.addPointCloud(copyPointCloudSceneXYZ, "cloud");
	viewer2.addPointCloud(keypointsScene, keypoints_color_handler2, "keypoints");
	viewer2.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "keypoints");
	viewer2.spin();

	pcl::visualization::PCLVisualizer viewer3("PCL Viewer");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler3 (keypointsObject, 0, 255, 0);
	viewer3.setBackgroundColor( 0, 0, 0 );
	viewer3.addPointCloud(copyPointCloudObjectXYZ, "cloud");
	viewer3.addPointCloud(keypointsObject, keypoints_color_handler3, "keypoints");
	viewer3.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "keypoints");
	viewer3.spin();

	pcl::visualization::PCLVisualizer viewer4("PCL Viewer");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler4 (keypointsObject2, 0, 255, 0);
	viewer4.setBackgroundColor( 0, 0, 0 );
	viewer4.addPointCloud(copyPointCloudObject2XYZ, "cloud");
	viewer4.addPointCloud(keypointsObject2, keypoints_color_handler4, "keypoints");
	viewer4.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "keypoints");
	viewer4.spin();

	pcl::visualization::PCLVisualizer viewer5("PCL Viewer");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler5 (keypointsObject3, 0, 255, 0);
	viewer5.setBackgroundColor( 0, 0, 0 );
	viewer5.addPointCloud(copyPointCloudObject3XYZ, "cloud");
	viewer5.addPointCloud(keypointsObject3, keypoints_color_handler5, "keypoints");
	viewer5.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "keypoints");
	viewer5.spin();

	pcl::visualization::PCLVisualizer viewer6("PCL Viewer");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler6 (keypointsObject4, 0, 255, 0);
	viewer6.setBackgroundColor( 0, 0, 0 );
	viewer6.addPointCloud(copyPointCloudObject4XYZ, "cloud");
	viewer6.addPointCloud(keypointsObject4, keypoints_color_handler6, "keypoints");
	viewer6.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "keypoints");
	viewer6.spin();
	

*/

	//DESCRIPTORES SHOT Y FPFH

	//SHOT
	pcl::PointCloud<pcl::SHOT352>::Ptr descriptorsScene(new pcl::PointCloud<pcl::SHOT352>());
	pcl::PointCloud<pcl::SHOT352>::Ptr descriptorsObject(new pcl::PointCloud<pcl::SHOT352>());
	pcl::PointCloud<pcl::SHOT352>::Ptr descriptorsObject2(new pcl::PointCloud<pcl::SHOT352>());
	pcl::PointCloud<pcl::SHOT352>::Ptr descriptorsObject3(new pcl::PointCloud<pcl::SHOT352>());
	pcl::PointCloud<pcl::SHOT352>::Ptr descriptorsObject4(new pcl::PointCloud<pcl::SHOT352>());
	
    //Variables para las normales de SHOT
	pcl::PointCloud<pcl::Normal>::Ptr normalsScene(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr normalsObject(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr normalsObject2(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr normalsObject3(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr normalsObject4(new pcl::PointCloud<pcl::Normal>);
	
	/*
  	//FPFH
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptorsScene(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptorsObject(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptorsObject2(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptorsObject3(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptorsObject4(new pcl::PointCloud<pcl::FPFHSignature33>());
	*/

    //Kdtrees para los descriptores XYZ para SHOT y ISS
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtreeScene(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtreeObject(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtreeObject2(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtreeObject3(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtreeObject4(new pcl::search::KdTree<pcl::PointXYZ>);


	//Obtener normales
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation; //objeto para las normales
	normalEstimation.setSearchMethod(kdtreeScene);  //metodo de busqueda
	normalEstimation.setInputCloud(copyPointCloudSceneXYZ);
	normalEstimation.setRadiusSearch(0.02); //radio de busqueda
	normalEstimation.compute(*normalsScene);

	normalEstimation.setSearchMethod(kdtreeObject);
	normalEstimation.setInputCloud(copyPointCloudObjectXYZ);
	normalEstimation.setRadiusSearch(0.02); 
	normalEstimation.compute(*normalsObject);

	normalEstimation.setSearchMethod(kdtreeObject2);
	normalEstimation.setInputCloud(copyPointCloudObject2XYZ); 
	normalEstimation.setRadiusSearch(0.02); 
	normalEstimation.compute(*normalsObject2);

	
	normalEstimation.setSearchMethod(kdtreeObject3);
	normalEstimation.setInputCloud(copyPointCloudObject3XYZ); 
	normalEstimation.setRadiusSearch(0.02);
	normalEstimation.compute(*normalsObject3);

	
	normalEstimation.setSearchMethod(kdtreeObject4);
	normalEstimation.setInputCloud(copyPointCloudObject4XYZ); 
	normalEstimation.setRadiusSearch(0.02); 
	normalEstimation.compute(*normalsObject4);



	//SHOT

	//Escena
	pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot; //objeto para pasar shot
	shot.setInputCloud(keypointsScene); //pasamos los keypoints como entrada 
	shot.setInputNormals(normalsScene); //estblecemos las normales de la escena
	shot.setSearchSurface(copyPointCloudSceneXYZ); 
	shot.setRadiusSearch(0.02); //radio de busqueda
	shot.compute(*descriptorsScene);

	//Objetos
	shot.setInputCloud(keypointsObject);
	shot.setInputNormals(normalsObject);
	shot.setSearchSurface(copyPointCloudObjectXYZ);
	shot.setRadiusSearch(0.02); //0.02
	shot.compute(*descriptorsObject);
										
	shot.setInputCloud(keypointsObject2);
	shot.setInputNormals(normalsObject2);
	shot.setSearchSurface(copyPointCloudObject2XYZ);
	shot.setRadiusSearch(0.02); //0.02
	shot.compute(*descriptorsObject2);

	shot.setInputCloud(keypointsObject3);
	shot.setInputNormals(normalsObject3);
	shot.setSearchSurface(copyPointCloudObject3XYZ);
	shot.setRadiusSearch(0.02); //0.02
	shot.compute(*descriptorsObject3);
										
	shot.setInputCloud(keypointsObject4);
	shot.setInputNormals(normalsObject4);
	shot.setSearchSurface(copyPointCloudObject4XYZ);
	shot.setRadiusSearch(0.02); //0.02
	shot.compute(*descriptorsObject4);


	//FPFH
    
    /*
	//ESCENA
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_escena; //objeto para el descriptor fpfh
	fpfh_escena.setInputCloud(keypointsScene);
	fpfh_escena.setInputNormals(normalsScene);
	fpfh_escena.setSearchMethod(kdtreeScene); //metodo de busqueda
	fpfh_escena.setRadiusSearch(0.05);

	fpfh_escena.compute(*descriptorsScene);

	//OBJETOS
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_objeto;
	fpfh_objeto.setInputCloud(keypointsObject);
	fpfh_objeto.setInputNormals(normalsObject);
	fpfh_objeto.setSearchMethod(kdtreeObject);
	fpfh_objeto.setRadiusSearch(0.05);
	fpfh_objeto.compute(*descriptorsObject);


	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_objeto2;
	fpfh_objeto2.setInputCloud(keypointsObject2);
	fpfh_objeto2.setInputNormals(normalsObject2);
	fpfh_objeto2.setSearchMethod(kdtreeObject2);
	fpfh_objeto2.setRadiusSearch(0.05);
	fpfh_objeto2.compute(*descriptorsObject2);


	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_objeto3;
	fpfh_objeto3.setInputCloud(keypointsObject3);
	fpfh_objeto3.setInputNormals(normalsObject3);
	fpfh_objeto3.setSearchMethod(kdtreeObject3);
	fpfh_objeto3.setRadiusSearch(0.05);
	fpfh_objeto3.compute(*descriptorsObject3);


	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_objeto4;
	fpfh_objeto4.setInputCloud(keypointsObject4);
	fpfh_objeto4.setInputNormals(normalsObject4);
	fpfh_objeto4.setSearchMethod(kdtreeObject4);
	fpfh_objeto4.setRadiusSearch(0.05);
	fpfh_objeto4.compute(*descriptorsObject4);

	*/

    /*//mostramos por pantalla el numero de descriptores
	cout << endl << "Found " << descriptorsScene->size() << " descriptors for the scene." << endl;
	cout << "Found " << descriptorsObject->size() << " descriptors for the object Plant." << endl;
	cout << "Found " << descriptorsObject2->size() << " descriptors for the object PiggyBank." << endl;
	cout << "Found " << descriptorsObject3->size() << " descriptors for the object PLC." << endl;
	cout << "Found " << descriptorsObject4->size() << " descriptors for the object Mug." << endl<<endl;

*/

	//Emparejamientos
    //Objeto 1
  	pcl::CorrespondencesPtr correspondencesScene (new pcl::Correspondences ()); //objeto para las correspondencias

  	pcl::KdTreeFLANN<pcl::SHOT352> matching; //Para shot
	//pcl::KdTreeFLANN<pcl::FPFHSignature33> matching; //Para fpfh

  	matching.setInputCloud (descriptorsObject);

  	for (size_t i = 0; i < descriptorsScene->size(); ++i) //para todos los descriptores 
  	{
        std::vector<int> neighIndices (1);
        std::vector<float> squareDistance (1);

        int neighborCount = matching.nearestKSearch (descriptorsScene->at (i), 1, neighIndices, squareDistance); //busca en todos los descriptores del objeto se busca el mas cercano

        if(neighborCount == 1 && squareDistance[0] < 0.5)  //si el mas cercano tiene como distancia menos de 0.5 unidades se acepta como correspondencia
        {                                                   
            pcl::Correspondence corresp (neighIndices[0], static_cast<int> (i), squareDistance[0]);
            correspondencesScene->push_back(corresp);
        }
  	}
    
  	//cout <<endl<< "Correspondences found in Plant: " << correspondencesScene->size() << endl;


    //lo mismo de arriba con el resto
	//Objeto 2

	pcl::CorrespondencesPtr correspondencesScene2 (new pcl::Correspondences ());
  	pcl::KdTreeFLANN<pcl::SHOT352> matching2;
	//pcl::KdTreeFLANN<pcl::FPFHSignature33> matching2;
	
  	matching2.setInputCloud (descriptorsObject2);

  	for (size_t i = 0; i < descriptorsScene->size(); ++i)
  	{
        std::vector<int> neighIndices2 (1);
        std::vector<float> squareDistance2 (1);

        int neighborCount2 = matching2.nearestKSearch (descriptorsScene->at (i), 1, neighIndices2, squareDistance2);

        if(neighborCount2 == 1 && squareDistance2[0] < 0.5)
        {                                                   
            pcl::Correspondence corresp2 (neighIndices2[0], static_cast<int> (i), squareDistance2[0]);
            correspondencesScene2->push_back(corresp2);
        }
  	}
    
  //	cout << "Correspondences found in PiggyBank: " << correspondencesScene2->size() << endl;



	//Objeto 3

	pcl::CorrespondencesPtr correspondencesScene3 (new pcl::Correspondences ());
  	pcl::KdTreeFLANN<pcl::SHOT352> matching3;
	//pcl::KdTreeFLANN<pcl::FPFHSignature33> matching3;
	
  	matching3.setInputCloud (descriptorsObject3);

  	for (size_t i = 0; i < descriptorsScene->size(); ++i)
  	{
        vector<int> neighIndices3 (1);
        vector<float> squareDistance3 (1);

        int neighborCount3 = matching3.nearestKSearch (descriptorsScene->at (i), 1, neighIndices3, squareDistance3);

        if(neighborCount3 == 1 && squareDistance3[0] < 0.5) 
        {                                                   
            pcl::Correspondence corresp3 (neighIndices3[0], static_cast<int> (i), squareDistance3[0]);
            correspondencesScene3->push_back(corresp3);
        }
  	}
    
  	//cout << "Correspondences found in PLC: " << correspondencesScene3->size() << endl;


	//Objeto 4

	pcl::CorrespondencesPtr correspondencesScene4 (new pcl::Correspondences ());
  	pcl::KdTreeFLANN<pcl::SHOT352> matching4;
	//pcl::KdTreeFLANN<pcl::FPFHSignature33> matching4;

  	matching4.setInputCloud (descriptorsObject4);

  	for (size_t i = 0; i < descriptorsScene->size(); ++i)
  	{
        vector<int> neighIndices4 (1);
        vector<float> squareDistance4 (1);

        int neighborCount4 = matching4.nearestKSearch (descriptorsScene->at (i), 1, neighIndices4, squareDistance4);

        if(neighborCount4 == 1 && squareDistance4[0] < 0.5) 
        {                                                   
            pcl::Correspondence corresp4 (neighIndices4[0], static_cast<int> (i), squareDistance4[0]);
            correspondencesScene4->push_back(corresp4);
        }
  	}
    
  	//cout << "Correspondences found in Mug: " << correspondencesScene4->size() << endl<<endl;

	

	//Buenas correspondencias
    //Objeto 1
	pcl::CorrespondencesPtr bestCorrespondence (new pcl::Correspondences);
    
    //de todas las correspondencias se cogen como buenas las que tengan menos distancia de 0.4, si es asi se guardan en un vector con las mejores correspondencias
	for (pcl::Correspondences::iterator bestCorresp (correspondencesScene->begin()); bestCorresp != correspondencesScene->end (); ++bestCorresp)
	{
		if (bestCorresp->distance <= 0.4) 
			bestCorrespondence->push_back(*bestCorresp);
	}
	//std::cout << "Good correspondences found in Plant: " << bestCorrespondence->size () << std::endl;


	//Objeto2
	pcl::CorrespondencesPtr bestCorrespondence2 (new pcl::Correspondences);
	
	for (pcl::Correspondences::iterator bestCorresp2 (correspondencesScene2->begin()); bestCorresp2 != correspondencesScene2->end (); ++bestCorresp2)
	{
		if (bestCorresp2->distance <= 0.4) 
			bestCorrespondence2->push_back(*bestCorresp2);
	}
	//std::cout << "Good correspondences found in PiggyBank: " << bestCorrespondence2->size () << std::endl;

	//objeto 3
	pcl::CorrespondencesPtr bestCorrespondence3 (new pcl::Correspondences);
	
	for (pcl::Correspondences::iterator bestCorresp3 (correspondencesScene3->begin()); bestCorresp3 != correspondencesScene3->end (); ++bestCorresp3)
	{
		if (bestCorresp3->distance <= 0.4) 
			bestCorrespondence3->push_back(*bestCorresp3);
	}
	//std::cout << "Good correspondences found in PLC: " << bestCorrespondence3->size () << std::endl;


	//Objeto 4
	pcl::CorrespondencesPtr bestCorrespondence4 (new pcl::Correspondences);
	
	for (pcl::Correspondences::iterator bestCorresp4 (correspondencesScene4->begin()); bestCorresp4 != correspondencesScene4->end (); ++bestCorresp4)
	{
		if (bestCorresp4->distance <= 0.4) 
			bestCorrespondence4->push_back(*bestCorresp4);
	}
	//std::cout << "Good correspondences found in Mug: " << bestCorrespondence4->size () <<endl<<endl;



	//Transformaciones
	//  Actual Clustering
	vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transformation;
	vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transformation2;
	vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transformation3;
	vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transformation4;
	
	vector<pcl::Correspondences> clusteredCorrespondences;
	vector<pcl::Correspondences> clusteredCorrespondences2;
	vector<pcl::Correspondences> clusteredCorrespondences3;
	vector<pcl::Correspondences> clusteredCorrespondences4;
	
    //Objeto 1
	pcl::GeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ> grouping;
	grouping.setGCSize(0.02); 
	grouping.setGCThreshold(3); //numero minimo de correspondencias para hacer las transformaciones
    grouping.setInputCloud(keypointsObject);
	grouping.setSceneCloud(keypointsScene);
	grouping.setModelSceneCorrespondences(bestCorrespondence);
	grouping.recognize(transformation, clusteredCorrespondences);

	pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointCloud (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::transformPointCloud(*copyPointCloudObjectXYZ, *transformPointCloud, transformation[0]);

	//Objeto2
	grouping.setInputCloud(keypointsObject2);
	grouping.setSceneCloud(keypointsScene);
	grouping.setModelSceneCorrespondences(bestCorrespondence2);
	grouping.recognize(transformation2, clusteredCorrespondences2);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointCloud2 (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::transformPointCloud(*copyPointCloudObject2XYZ, *transformPointCloud2, transformation2[0]);


	pcl::GeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ> grouping2;
	grouping2.setGCSize(0.02); 
	grouping2.setGCThreshold(2);
	
    //Objeto 4
    grouping2.setInputCloud(keypointsObject4);
	grouping2.setSceneCloud(keypointsScene);
	grouping2.setModelSceneCorrespondences(bestCorrespondence4);
	grouping2.recognize(transformation4, clusteredCorrespondences4);

	pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointCloud4 (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::transformPointCloud(*copyPointCloudObject4XYZ, *transformPointCloud4, transformation4[0]);
		
	//Objeto 3
	grouping2.setInputCloud(keypointsObject3);
	grouping2.setSceneCloud(keypointsScene);
	grouping2.setModelSceneCorrespondences(bestCorrespondence3);
	grouping2.recognize(transformation3, clusteredCorrespondences3);

	pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointCloud3 (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::transformPointCloud(*copyPointCloudObject3XYZ, *transformPointCloud3, transformation3[0]);


	

	

	//ICP
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZ>); //nube final tras pasar ICP
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> registration;
	registration.setInputSource(transformPointCloud);
	registration.setInputTarget(copyPointCloudSceneXYZ);

	registration.align(*finalCloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> registration2;
	registration2.setInputSource(transformPointCloud2);
	registration2.setInputTarget(copyPointCloudSceneXYZ);

	registration2.align(*finalCloud2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud3(new pcl::PointCloud<pcl::PointXYZ>);

	registration.setInputSource(transformPointCloud3);
	registration.setInputTarget(copyPointCloudSceneXYZ);

	registration.align(*finalCloud3);

	pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud4(new pcl::PointCloud<pcl::PointXYZ>);

	registration.setInputSource(transformPointCloud4);
	registration.setInputTarget(copyPointCloudSceneXYZ);

	registration.align(*finalCloud4);
	
	Eigen::Matrix4f tmp;

	tmp = registration.getFinalTransformation();
	pcl::transformPointCloud(*finalCloud4,*finalCloud4,tmp);

	auto end = high_resolution_clock::now();
	cout<<"Tiempo CPU(ms): "<<duration_cast<milliseconds>(end-begin).count()<<endl;


    //Visualizar todos los objetos
    //Escena
	pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
	viewer.addPointCloud<pcl::PointXYZRGBA>(visualizerPointCloud, "scene_cloud");

	//Objeto 1
	std::stringstream ss_cloud;
	ss_cloud << "instance" << 1;

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorCloud(transformPointCloud, 255, 0, 0);
	viewer.addPointCloud(transformPointCloud, colorCloud, ss_cloud.str ());


	//Objeto 2
	std::stringstream ss_cloud2;
	ss_cloud2 << "instance" << 2;

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorCloud2(transformPointCloud2, 255, 0, 0);
	viewer.addPointCloud(transformPointCloud2, colorCloud2, ss_cloud2.str ());


    //Objeto 3
	std::stringstream ss_cloud4;
	ss_cloud4 << "instance" << 4;

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorCloud4(finalCloud4, 255, 0, 0);
	viewer.addPointCloud(finalCloud4, colorCloud4, ss_cloud4.str ());


    //Objeto 4
	std::stringstream ss_cloud3;
	ss_cloud3 << "instance" << 3;

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorCloud3(finalCloud3, 255, 0, 0);
	viewer.addPointCloud(finalCloud3, colorCloud3, ss_cloud3.str ());


	viewer.spin();

	return 0;
}
