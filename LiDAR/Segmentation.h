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
//#include <opencv2/core/eigen.hpp>
//#include <Eigen/Core> //http://eigen.tuxfamily.org/index.php?title=Visual_Studio
//#include <Eigen/Eigenvalues>

class Segmentation
{
private:
	int k_for_process_all_point_cloud;

public:

	std::string path;
	
	
	

	Segmentation(void);
	~Segmentation(void);
	std::vector<cv::Mat>  Segmentation::n_x_m_tile_generator(cv::Mat image, int n_rows, int n_cols);
	cv::Mat Segmentation::merge_tiles(std::vector<cv::Mat> tiles_array, int n_rows, int n_cols);
	bool Segmentation::watershed_segmentation(std::string image_name);
};

