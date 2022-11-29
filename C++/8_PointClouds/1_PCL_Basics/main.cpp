#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <boost/algorithm/string.hpp>
#include <math.h>


using namespace std;


void spawnViewer(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->addCoordinateSystem (1.0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(pointCloud);
    viewer->addPointCloud<pcl::PointXYZRGBA> (pointCloud, rgb, "id0");
    while(!viewer->wasStopped ()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	viewer->close();
}

//Mostar nube
void showCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud)
{
	// Abrir un visualizador que muestre la nube filtrada
	boost::thread visualizer = boost::thread(spawnViewer, cloud);
	cout << "Pulsa para continuar" << endl;
	cin.get();
	visualizer.interrupt();
	visualizer.join();
}

//Aplicar voxelgrid de tamaño leaf en las 3 dimensiones
void applyVoxelGrid(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& inputCloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& outputCloud, const double& leaf)
{
	// Aplicar voxel grid con un tamanyo de hoja "leaf"
	cout << "Aplicando VoxelGrid..." << endl;
	pcl::VoxelGrid<pcl::PointXYZRGBA> voxelgrid_filter;
	voxelgrid_filter.setInputCloud (inputCloud);			// nube de origen
	voxelgrid_filter.setLeafSize (leaf, leaf, leaf);		// tamanyo del voxel
	voxelgrid_filter.filter (*outputCloud);		// ejecutar el filtrado y guardar el resultado
	// Obtener el numero de puntos de la nube filtrada
	cout << "Nube tras aplicar voxel grid " << outputCloud->points.size() << endl;
}

//Generar matriz de traslación
Eigen::Affine3f generateTranslationMatrix(const Eigen::Vector3f& trans)
{
	//Transformación de translación
	Eigen::Affine3f transform(Eigen::Affine3f::Identity());
	//Construir translación
	transform.translation() = trans;
	return transform;
}

//Aplicar translación
void applyTranslation(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& inputCloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& outputCloud,
	const Eigen::Vector3f& trans)
{
	//Transformación de translación
	Eigen::Affine3f transform = generateTranslationMatrix(trans);	
	std::cout << "Aplicando traslación..." << endl;
	std::cout<<"Aplicando transformación sobre la superficie: \n"<<transform.matrix()<<std::endl;
	//Aplicar transformación a la nube de puntos
	pcl::transformPointCloud (*inputCloud, *outputCloud, transform);
}

//Aplicar translación inversa
void applyTranslationInv(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& inputCloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& outputCloud,
	const Eigen::Vector3f& trans)
{
	//Transformación de translación
	Eigen::Affine3f transform = generateTranslationMatrix(trans);	
	transform = transform.inverse();
	
	std::cout << "Aplicando traslación inversa..." << endl;
	std::cout<<"Aplicando transformación sobre la superficie: \n"<<transform.matrix()<<std::endl;
	//Aplicar transformación a la nube de puntos
	pcl::transformPointCloud (*inputCloud, *outputCloud, transform);
}

//Generar matriz de escala
Eigen::Affine3f generateScaleMatrix(const Eigen::Vector3f& scale)
{
	// Construir transformación de escala
	Eigen::Affine3f transform(Eigen::Affine3f::Identity());
	transform = Eigen::Scaling(scale);
	return transform;
}

//Aplicar escala
void applyScaling(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& inputCloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& outputCloud,
	const Eigen::Vector3f& scale)
{
	// Construir transformación de escala
	Eigen::Affine3f transform = generateScaleMatrix(scale);
	//Mostrar transformación
	std::cout << "Aplicando escala..." << endl;
	std::cout<<"Aplicando transformación sobre la superficie: \n"<<transform.matrix()<<std::endl;
	//Aplicar transformación a la nube de puntos
	pcl::transformPointCloud (*inputCloud, *outputCloud, transform);
}

//Aplicar escala inversa
void applyScalingInv(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& inputCloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& outputCloud,
	const Eigen::Vector3f& scale)
{
	// Construir transformación de escala
	Eigen::Affine3f transform = generateScaleMatrix(scale);
	transform = transform.inverse();
	
	//Mostrar transformación
	std::cout << "Aplicando escala inversa..." << endl;
	std::cout<<"Aplicando transformación sobre la superficie: \n"<<transform.matrix()<<std::endl;
	//Aplicar transformación a la nube de puntos
	pcl::transformPointCloud (*inputCloud, *outputCloud, transform);
}

//Generar matriz de rotación
Eigen::Affine3f generateRotationMatrix(const Eigen::Vector3f& rotation)
{
	// Aplicar transformación de rotación
	Eigen::Affine3f transform(Eigen::Affine3f::Identity());
	//Rotaciones en los 3 ejes (Euler ZYX -> yaw, pitch, roll)
	transform.rotate (Eigen::AngleAxisf (rotation[0], Eigen::Vector3f::UnitZ())); 
	transform.rotate (Eigen::AngleAxisf (rotation[1], Eigen::Vector3f::UnitY()));
	transform.rotate (Eigen::AngleAxisf (rotation[2], Eigen::Vector3f::UnitX()));
	return transform;
}

//Aplicar rotación
void applyRotation(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& inputCloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& outputCloud,
	const Eigen::Vector3f& rotation)
{
	// Aplicar transformación de rotación
	Eigen::Affine3f transform = generateRotationMatrix(rotation);
	
	std::cout << "Aplicando rotación..." << endl;
	std::cout<<"Ángulos de rotación (Z,Y,X): ("<<rotation[0]<<","<<rotation[1]<<","<<rotation[2]<<")\n";
	std::cout<<"Aplicando transformación sobre la superficie: \n"<<transform.matrix()<<std::endl;
	//Aplicar transformación a la nube de puntos
	pcl::transformPointCloud (*inputCloud, *outputCloud, transform);
}

//Aplicar rotación invertida
void applyRotationInv(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& inputCloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& outputCloud,
	const Eigen::Vector3f& rotation)
{
	// Aplicar transformación de rotación
	Eigen::Affine3f transform = generateRotationMatrix(rotation);
	transform = transform.inverse();
	
	std::cout << "Aplicando rotación inversa..." << endl;
	std::cout<<"Ángulos de rotación (Z,Y,X): ("<<rotation[0]<<","<<rotation[1]<<","<<rotation[2]<<")\n";
	std::cout<<"Aplicando transformación sobre la superficie: \n"<<transform.matrix()<<std::endl;
	//Aplicar transformación a la nube de puntos
	pcl::transformPointCloud (*inputCloud, *outputCloud, transform);
}

//Aplicar filtro passthrough sobre una dimensión
void applyPassThrough(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& inputCloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& outputCloud, const std::string& dimension,
	const double& min, const double& max)
{
	// Aplicar filtro passthrough en el eje "dimension" con min = "min", max="max"
	cout << "Aplicando PassThrough..." << endl;
	pcl::PassThrough<pcl::PointXYZRGBA> passthorugh_filter;
	passthorugh_filter.setInputCloud (inputCloud);	// nube de origen
	passthorugh_filter.setFilterFieldName (dimension);				// eje que filtra
	passthorugh_filter.setFilterLimits (min, max);				// excluimos lo que este fuera del intervalo [2.0, 6.0]
	passthorugh_filter.filter (*outputCloud);			// ejecutar el filtrado y guardar el resultado
}

int main (int argc, char** argv)
{
	boost::thread visualizer;
	
	std::string pcdFile;
	//Comprobar si se ha recibido parámetro
	if(argc > 1)
	{
		//Nombre de fichero introducido
		pcdFile = std::string(argv[1]);
	}
	else
	{
		// Escena de Kinect
		pcdFile = "../scenes/cloud_291.pcd";
	}

	// Cargar una nube de puntos
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	if(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (pcdFile, *pointCloud) == -1)
	{
		std::cerr<<"Error al cargar la nube de puntos "+pcdFile+"\n"<<std::endl;
		return -1;
	}

	// Obtener el numero de puntos de la nube
	cout << "Esta nube tiene " << pointCloud->points.size() << endl;

	// Obtener el centroide de la nube
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid 	(*pointCloud, centroid);
	cout << "El centroide es \n" << centroid << endl;

	// Abrir un visualizador que muestre la nube cargada
	showCloud(pointCloud);
	
	//Aplicar voxelgrid con tamaño de hoja 0.05
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud_voxelized (new pcl::PointCloud<pcl::PointXYZRGBA>);
	applyVoxelGrid(pointCloud, pointCloud_voxelized, 0.05);
	// Obtener el numero de puntos de la nube filtrada
	cout << "Esta nube tiene " << pointCloud_voxelized->points.size() << endl;
	// Abrir un visualizador que muestre la nube cargada
	showCloud(pointCloud_voxelized);

	//Aplicar traslación
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud_trans(new pcl::PointCloud<pcl::PointXYZRGBA>);
	applyTranslation(pointCloud_voxelized, pointCloud_trans, Eigen::Vector3f(10,5,-3));
	// Obtener el numero de puntos de la nube
	cout << "Esta nube tiene " << pointCloud_trans->points.size() << endl;
	// Abrir un visualizador que muestre la nube cargada
	showCloud(pointCloud_trans);
	
	//Aplicar transformación de escala
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud_scale(new pcl::PointCloud<pcl::PointXYZRGBA>);
	applyScaling(pointCloud_trans, pointCloud_scale, Eigen::Vector3f(4,4,4));
	// Obtener el numero de puntos de la nube
	cout << "Esta nube tiene " << pointCloud_scale->points.size() << endl;
	// Abrir un visualizador que muestre la nube cargada
	showCloud(pointCloud_scale);
	
	//Aplicar transformación de rotación
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud_rotation(new pcl::PointCloud<pcl::PointXYZRGBA>);
	applyRotation(pointCloud_scale, pointCloud_rotation, Eigen::Vector3f(M_PI/4.0,M_PI,M_PI/2.0));
	// Obtener el numero de puntos de la nube
	cout << "Esta nube tiene " << pointCloud_rotation->points.size() << endl;
	// Abrir un visualizador que muestre la nube cargada
	showCloud(pointCloud_rotation);
	
	//Aplicar transformación inversa de rotación
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud_rotation_inv(new pcl::PointCloud<pcl::PointXYZRGBA>);
	applyRotationInv(pointCloud_rotation, pointCloud_rotation_inv, Eigen::Vector3f(M_PI/4.0,M_PI,M_PI/2.0));
	// Obtener el numero de puntos de la nube
	cout << "Esta nube tiene " << pointCloud_rotation_inv->points.size() << endl;
	// Abrir un visualizador que muestre la nube cargada
	showCloud(pointCloud_rotation_inv);
	
	//Aplicar transformación de escala inversa
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud_scale_inv(new pcl::PointCloud<pcl::PointXYZRGBA>);
	applyScalingInv(pointCloud_rotation_inv, pointCloud_scale_inv, Eigen::Vector3f(4,4,4));
	// Obtener el numero de puntos de la nube
	cout << "Esta nube tiene " << pointCloud_scale_inv->points.size() << endl;
	// Abrir un visualizador que muestre la nube cargada
	showCloud(pointCloud_scale_inv);
	
	//Aplicar traslación inversa
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud_trans_inv(new pcl::PointCloud<pcl::PointXYZRGBA>);
	applyTranslationInv(pointCloud_scale_inv, pointCloud_trans_inv, Eigen::Vector3f(10,5,-3));
	// Obtener el numero de puntos de la nube
	cout << "Esta nube tiene " << pointCloud_trans_inv->points.size() << endl;
	// Abrir un visualizador que muestre la nube cargada
	showCloud(pointCloud_trans_inv);
	
	// Aplicar filtro passthrough en el eje Z con min = 2, max=6
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
	applyPassThrough(pointCloud_trans_inv, pointCloud_filtered, "z", 2.0, 6.0);
	// Obtener el numero de puntos de la nube filtrada
	cout << "Esta nube tiene " << pointCloud_filtered->points.size() << endl;
	// Abrir un visualizador que muestre la nube cargada
	showCloud(pointCloud_filtered);
	
	//Guardar en disco el resultado
	std::string output;
	//Comprobar si se ha recibido parámetro
	if(argc > 2)
	{
		//Nombre de fichero de salida introducido
		output = std::string(argv[2]);
	}
	else
	{
		//Salida en el mismo directorio
		output = "output.pcd";
	}
	
	//Guardar fichero
	if(pcl::io::savePCDFileASCII (output, *pointCloud_filtered) == -1)
	{
		std::cerr<<"Error al almacenar la nube de puntos en "+output<<std::endl;
		return -1;
	}
	else
	{
		std::cout<<"Nube de puntos guardada en "<<output<<std::endl;
	}


	return 0;
}
