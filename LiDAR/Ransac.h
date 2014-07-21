#pragma once 

#include "PointCloudAccessor.h"
#include "PointCloudAccessorImpl.h"
#include "PointCloudDataDescriptor.h"
#include "PointCloudDataRequest.h"
#include "PointCloudElement.h"
#include "PointCloudView.h"
#include "ProgressTracker.h"
#include "PseudocolorLayer.h"
#include "RasterElement.h"
#include "RasterUtilities.h"
#include "SpatialDataView.h"
#include "SpatialDataWindow.h"
#include "Statistics.h"
#include "switchOnEncoding.h" 
#include "Undo.h"
#include <limits>
#include <boost/random/uniform_int.hpp>
#include <boost/random/random_device.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#include <Eigen/Core> //http://eigen.tuxfamily.org/index.php?title=Visual_Studio
#include <Eigen/Eigenvalues>
#include "StringUtilities.h"
#include <iostream>
#include <fstream>
// needed by the Raster generator
#include "DataRequest.h"
#include "AlgorithmShell.h"
#include "DesktopServices.h"
#include "DataAccessor.h"
#include "DataAccessorImpl.h"
//#include <gdal/gdal.h>
//#include <gdal/gdal_priv.h>
//#include <gdal/gdal_alg.h>
// needed for buildings outline segmentation
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/core/eigen.hpp>
//#include <opencv2/opencv.hpp>
//#include <opencv2/features2d/features2d.hpp>
//#include <opencv2\features2d\features2d.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>


namespace
{
template<typename T>
void assign(T* pData, float val)
{
   *pData = static_cast<T>(val);
}
}

// these methods are needed for eigen values
template<typename Matrix, typename Roots> inline void
computeRoots (const Matrix& m, Roots& roots);
template<typename Scalar, typename Roots> void
computeRoots2 (const Scalar& b, const Scalar& c, Roots& roots);
template<typename Matrix, typename Vector> inline void
eigen33 ( Matrix& mat, typename Matrix::Scalar& eigenvalue, Vector& eigenvector);

class WatershedSegmenter
{
private:
    cv::Mat markers;
public:
    void setMarkers(cv::Mat& markerImage)
    {
        markerImage.convertTo(markers, CV_32S);
    }

    cv::Mat process(cv::Mat &image)
    {
        cv::watershed(image, markers);
        markers.convertTo(markers,CV_8U);
        return markers;
    }
};


class Ransac
{
public:
    
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> prova33;
	 
	std::string msg2;
	std::string msg1; // statistics message
	std::ofstream myfile;
	std::string path;// ="C:/Users/Roberta/Desktop/Universita/GSoC_2014_Opticks/SampleData/";

	//* RANSAC ALGORITHM VARIABLES
	Eigen::VectorXd  model_coefficients;
	std::vector<int> random_selected_indices;
    const PointCloudDataDescriptor* pDesc;
	std::vector<int> inliers; // it contains the inliers indexes (their ID) for the single iterations: maybe this must be private
	int nr_p;//NUMBER OF THE INLIERS for the single iteration
	Eigen::VectorXd optimized_coefficients;
	std::vector<int> final_inliers;// the inliers found
	Eigen::VectorXd final_model_coefficients; // the coefficients corrispondent to the max number of inliers
	int n_best_inliers_count;


	//* VARIABLES NEEDED FOR SEGMENTATION
	int k_for_process_all_point_cloud;
    std::vector<cv::Mat> tiles_array; // stores the tiles in which the original raster is divided (maybe this mustn'be a global variable)
	std::vector<cv::Mat> result_tiles_array; // stores the result of the segmentation algorithm
	
	//* ORIGINAL_TILES_MERGED is the real raster input for all the processing methods of the Plug-In,
	//* since the way I use to tile the original raster makes lose some pixels
	//* (however they are all on the last columns and/or last rows, so the original raster conformation is preserved - no problems for the image coordinates of the pixels)
	//* for example, original_tiles_merged is needed for the mask application (method use to retrieve the z coordinate of every pixel identified as belonging to a building)
	cv::Mat original_tiles_merged;

	//* blobs is a std::vector of cv::Point
	//* each row of this std::vector stores the image coordinates of all the pixels identified as belonging to a specific building
	std::vector < std::vector<cv::Point2i> > blobs; 

	//* buildingS is a std::vector of Eigen matrixes
	//* each row of this std::vector is an Eigen matrix with rows = number of points belonging to the specific building and columns = 3 (x,y,z coordinates)
	//* first row stores the x,y and z coordinates of all the points belonging to first identified building
	//* second row stores the x,y and z coordinates of all the points belonging to second identified building...
	std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> buildingS;

	std::vector<std::vector<int>> buildingS_inliers;// stores the inliers for all the buildings
	std::vector<Eigen::VectorXd> buildingS_plane_coefficients;// stores the plane parameters for all the buildings

	Ransac(void);
	~Ransac(void);
	bool Ransac::ComputeModel(PointCloudElement* pElement, double ransac_threshold);
	bool Ransac::getSamples (int model_points);
	bool Ransac::computeModelCoefficients (PointCloudAccessor acc);
	bool Ransac::countWithinDistance(double threshold, PointCloudAccessor acc);
	bool Ransac::optimizeModelCoefficients(PointCloudAccessor acc);

	// ALL THE METHOD WITH THE 2 NUMBER CAN BE APPLIED TO AN EIGEN MATRIX(AND NOT THE PELEMENT)
	bool Ransac::ComputeModel2(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> data, double ransac_threshold);
	bool Ransac::getSamples2 (int model_points,int size_array);
	bool Ransac::computeModelCoefficients2 ( Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> data);
	bool Ransac::countWithinDistance2(double threshold,  Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> data);
	bool Ransac::optimizeModelCoefficients2(Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> data);
	bool Ransac::Ransac_for_buildings(float dem_spacing, PointCloudElement* pElement, double ransac_threshold);

	bool Ransac::generate_DEM (PointCloudElement* pElement, float post_spacing, int n_rows_tiles, int n_cols_tiles);
	bool Ransac::generate_raster_from_intensity (PointCloudElement* pElement, float post_spacing);
	bool Ransac::generate_point_cloud_statistics (PointCloudElement* pElement);
	bool Ransac::draw_raster_from_eigen_mat (std::string name, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> median_Eigen, PointCloudElement* pElement);
	bool Ransac::draw_raster_from_openCV_mat (std::string name, cv::Mat image, PointCloudElement* pElement);
	bool Ransac::watershed_segmentation(std::string image_name,PointCloudElement* pElement);
	bool Ransac::n_x_n_tile_generator(cv::Mat image, int n);
	bool Ransac::n_x_m_tile_generator(cv::Mat image, int n_rows, int n_cols, PointCloudElement* pElement);
	cv::Mat Ransac::merge_tiles(std::vector<cv::Mat> tiles_array, int n_rows, int n_cols);
	bool Ransac::process_all_point_cloud_with_watershed(int n_rows, int n_cols, PointCloudElement* pElement);
	bool Ransac::process_all_point_cloud_with_pca(int n_rows, int n_cols, PointCloudElement* pElement);
	cv::Scalar Ransac::cv_matrix_mode (cv::Mat image);
	double Ransac::getOrientation(std::vector<cv::Point> &pts, cv::Mat &img);
	bool Ransac::pca_segmentation(std::string image_name, PointCloudElement* pElement);
	void Ransac::FindBlobs(const cv::Mat &binary, std::vector < std::vector<cv::Point2i> > &blobs);
	bool Ransac::connected_components(std::string image_name, PointCloudElement* pElement);
	bool Ransac::draw_buildings_contours(cv::Mat image);
    std::string Ransac::type_of_CVMat_2_str(int type);
	bool Ransac::print_result();
};

