/*
 * The information in this file is
 * Copyright(c) 2007 Ball Aerospace & Technologies Corporation
 * and is subject to the terms and conditions of the
 * GNU Lesser General Public License Version 2.1
 * The license text is available from   
 * http://www.gnu.org/licenses/lgpl.html
 */

#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core_c.h>
#include <Eigen/Core> //http://eigen.tuxfamily.org/index.php?title=Visual_Studio
#include "Ransac.h"

class Segmentation
{
private:
	int k_for_process_all_point_cloud;
	void Segmentation::FindBlobs(const cv::Mat &binary, std::vector < std::vector<cv::Point2i> > &blobs);// needed for connected components
	double Segmentation::getOrientation(std::vector<cv::Point> &pts, cv::Mat &img); //needed for PCA segmentation

public:
	
	std::string segmentation_msg;
	std::string path;
	
	std::vector<cv::Mat> result_tiles_array;// stores the result of the segmentation algorithm
	
	//* blobs is a std::vector of cv::Point
	//* each row of this std::vector stores the image coordinates of all the pixels identified as belonging to a specific building
	std::vector < std::vector<cv::Point2i> > blobs; 


	Ransac Ransac_buildings;
	//* buildingS is a std::vector of Eigen matrixes
	//* each row of this std::vector is an Eigen matrix with rows = number of points belonging to the specific building and columns = 3 (x,y,z coordinates)
	//* first row stores the x,y and z coordinates of all the points belonging to first identified building
	//* second row stores the x,y and z coordinates of all the points belonging to second identified building...
	std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> buildingS;

	std::vector<std::vector<int>> buildingS_inliers;// stores the inliers for all the buildings (only first iteration - iteration number 0)
	std::vector<std::vector<int>> buildingS_outliers;// stores the outliers for all the buildings (only first iteration - iteration number 0)
	std::vector<Eigen::VectorXd> buildingS_plane_coefficients;// stores the plane parameters for all the buildings (only first iteration - iteration number 0)
	std::vector<int> buldingS_number_inliers; // every row stores the number of inliers for each building (only first iteration - iteration number 0)


	Segmentation(void);
	~Segmentation(void);
	std::vector<cv::Mat>  Segmentation::n_x_m_tile_generator(cv::Mat image, int n_rows, int n_cols);
	cv::Mat Segmentation::merge_tiles(std::vector<cv::Mat> tiles_array, int n_rows, int n_cols);
	
	
	bool Segmentation::watershed_segmentation(std::string image_name);
	bool Segmentation::pca_segmentation(std::string image_name);

	cv::Mat Segmentation::process_all_point_cloud_with_watershed(int n_rows, int n_cols);
	bool Segmentation::process_all_point_cloud_with_pca(int n_rows, int n_cols);

	cv::Scalar Segmentation::cv_matrix_mode (cv::Mat image);
	bool Segmentation::connected_components(cv::Mat input);
	bool Segmentation::draw_buildings_contours(cv::Mat image);
	bool Segmentation::Ransac_for_buildings(float dem_spacing, double ransac_threshold, cv::Mat original_tiles_merged);
	bool Segmentation::print_result();
};

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

