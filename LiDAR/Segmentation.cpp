#include "Segmentation.h"


Segmentation::Segmentation(void)
{
	path = "C:/Users/Roberta/Desktop/Results/";
	k_for_process_all_point_cloud = 0;
}


Segmentation::~Segmentation(void)
{
}


bool Segmentation::n_x_m_tile_generator(cv::Mat image, int n_rows, int n_cols)
{
	//* in this way I lose some pixels (original DEM raster has width and height greater than those of tiles merged)
	//* however they are all on the last columns and/or last rows, so the original raster conformation is preserved 
	//* (no problems for the image coordinates of the pixels)
	
	int unitWidth = image.cols / n_cols; 
    int unitHeight = image.rows / n_rows; 
	int k = 0;
	
	tiles_array.resize(n_rows * n_cols);

	//This for loop generates n_rows X n_cols tiles 
	for(int i = 0; i < n_rows; i++) 
	{  //i is row index
    // inner loop added so that more than one row of tiles written
       for(int j = 0; j < n_cols; j++)
	   { // j is col index
       
        cv::Mat subImage = image(cv::Rect(j * unitWidth, i * unitHeight, unitWidth, unitHeight));

		tiles_array[k] = subImage;
        std::ostringstream oss;
        oss << i << "_" << j << ".png";
        std::string name = oss.str();
		cv::Mat CVtile32FC1;
        imwrite(path + "Tiles/"+ name, subImage);
		k++;
        }
	}

	return true;
}
