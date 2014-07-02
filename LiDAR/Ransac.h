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
#include <gdal/gdal.h>
#include <gdal/gdal_priv.h>
#include <gdal/gdal_alg.h>
// needed for buildings outline segmentation
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/core/eigen.hpp>
//#include <opencv2/opencv.hpp>

//#include <sstream>   



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
	
	Eigen::VectorXd  model_coefficients;
	 
	std::vector<int> random_selected_indices;
    const PointCloudDataDescriptor* pDesc;
	std::vector<int> inliers; // it contains the inliers indexes (their ID) for the single iterations
	int nr_p;//NUMBER OF THE INLIERS for the single iterations
	Eigen::VectorXd optimized_coefficients;

	int k_for_process_all_point_cloud;
	std::string msg2;
	std::string msg1; // statistics message
	std::ofstream myfile;
	std::string path;// ="C:/Users/Roberta/Desktop/Universita/GSoC_2014_Opticks/SampleData/";
    std::vector<cv::Mat> tiles_array;
	std::vector<cv::Mat> result_tiles_array;


	Ransac(void);
	~Ransac(void);
	bool Ransac::ComputeModel(PointCloudElement* pElement);
	bool Ransac::getSamples (int model_points);
	bool Ransac::computeModelCoefficients (PointCloudAccessor acc);
	bool Ransac::countWithinDistance(double threshold, PointCloudAccessor acc);
	bool Ransac::optimizeModelCoefficients(PointCloudAccessor acc);
	bool Ransac::computeRootsdouble (const Eigen::Matrix3d m, Eigen::Vector3d roots);
	bool Ransac::computeRoots2double (double b, double c, Eigen::Vector3d roots);
	bool Ransac::generate_DEM (PointCloudElement* pElement, float post_spacing);
	bool Ransac::generate_raster_from_intensity (PointCloudElement* pElement, float post_spacing);
	bool Ransac::generate_point_cloud_statistics (PointCloudElement* pElement);
	bool Ransac::draw_raster_from_eigen_mat (std::string name, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> median_Eigen, PointCloudElement* pElement);
	bool Ransac::draw_raster_from_openCV_mat (std::string name, cv::Mat image, PointCloudElement* pElement);
	bool Ransac::standalone_opencv(std::string image_name,PointCloudElement* pElement);
	bool Ransac::n_x_n_tile_generator(cv::Mat image, int n);
	bool Ransac::n_x_m_tile_generator(cv::Mat image, int n_rows, int n_cols, PointCloudElement* pElement);
	bool Ransac::process_all_point_cloud(int n_rows, int n_cols, PointCloudElement* pElement);
	cv::Scalar Ransac::cv_matrix_mode (cv::Mat image);
	double getOrientation(std::vector<cv::Point> &pts, cv::Mat &img);
};

