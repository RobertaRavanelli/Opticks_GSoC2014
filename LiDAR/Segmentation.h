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
public:

	std::string path;
	int k_for_process_all_point_cloud;
	std::vector<cv::Mat> tiles_array; // stores the tiles in which the original raster is divided, these tiles are the input for the segmentation algorhithm 
	

	Segmentation(void);
	~Segmentation(void);
	bool Segmentation::n_x_m_tile_generator(cv::Mat image, int n_rows, int n_cols);
};

