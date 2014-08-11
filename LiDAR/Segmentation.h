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


class Segmentation
{
private:
	int k_for_process_all_point_cloud;
	void Segmentation::FindBlobs(const cv::Mat &binary, std::vector < std::vector<cv::Point2i> > &blobs);// needed for connected components
	double Segmentation::getOrientation(std::vector<cv::Point> &pts, cv::Mat &img); //needed for PCA segmentation

public:
	
	std::string segmentation_msg;
	std::string path;
	
	std::vector<cv::Mat> result_tiles_array;
	
	//* blobs is a std::vector of cv::Point
	//* each row of this std::vector stores the image coordinates of all the pixels identified as belonging to a specific building
	std::vector < std::vector<cv::Point2i> > blobs; 

	Segmentation(void);
	~Segmentation(void);
	std::vector<cv::Mat>  Segmentation::n_x_m_tile_generator(cv::Mat image, int n_rows, int n_cols);
	cv::Mat Segmentation::merge_tiles(std::vector<cv::Mat> tiles_array, int n_rows, int n_cols);
	
	
	bool Segmentation::watershed_segmentation(std::string image_name);
	bool Segmentation::pca_segmentation(std::string image_name);

	cv::Mat Segmentation::process_all_point_cloud_with_watershed(cv::Mat original_tiles_merged, int n_rows, int n_cols);
	bool Segmentation::process_all_point_cloud_with_pca(int n_rows, int n_cols);

	cv::Scalar Segmentation::cv_matrix_mode (cv::Mat image);
	bool Segmentation::connected_components(cv::Mat input);
	bool Segmentation::draw_buildings_contours(cv::Mat image);
};

class WatershedSegmenter2
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

