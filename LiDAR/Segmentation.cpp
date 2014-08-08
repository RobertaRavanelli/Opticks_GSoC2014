/*
 * The information in this file is
 * Copyright(c) 2007 Ball Aerospace & Technologies Corporation
 * and is subject to the terms and conditions of the
 * GNU Lesser General Public License Version 2.1
 * The license text is available from   
 * http://www.gnu.org/licenses/lgpl.html
 */

#include "Segmentation.h"

Segmentation::Segmentation(void)
{
	path = "C:/Users/Roberta/Desktop/Results/";
	k_for_process_all_point_cloud = 0;
}


Segmentation::~Segmentation(void)
{
}


std::vector<cv::Mat>  Segmentation::n_x_m_tile_generator(cv::Mat image, int n_rows, int n_cols)
{
	//* in this way I lose some pixels (original DEM raster has width and height greater than those of tiles merged)
	//* however they are all on the last columns and/or last rows, so the original raster conformation is preserved 
	//* (no problems for the image coordinates of the pixels)
	std::vector<cv::Mat> tiles_array; // stores the tiles in which the original raster is divided, these tiles are the input for the segmentation algorhithm 

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
	return tiles_array;
}

cv::Mat Segmentation::merge_tiles(std::vector<cv::Mat> tiles_array, int n_rows, int n_cols)
{
	std::vector<cv::Mat> rows_array;
	rows_array.resize(n_rows);
	cv::Mat merged_image;
	for(int i = 0; i < n_rows; i++) 
	{  //i is row index
       bool break_row = false;
	   cv::hconcat(tiles_array[i * n_cols], tiles_array[i * n_cols + 1], rows_array[i]);
	  /* std::ostringstream oss2;
       oss2 << i;
	   std::string name_i = oss2.str();*/
	   for(int j = 2; j < n_cols; j ++)
	   { // j is col index; // n is the index for the result_array
		   int n = i * n_cols + j;
		   if (n < n_rows * n_cols)
		   {
			  cv::hconcat(rows_array[i], tiles_array[n], rows_array[i]);
		   }
		   else
		   {
			   break_row = true;
			   break;
		   }
		   //cv::imshow("row_" + name_i, rows_array[i]);
        }

	   if(break_row)
	   {
		   //msg2 += "problem occurred in merging tiles\n\n";
		   //return false;
	   }
	}
	
	merged_image = rows_array[0];
	for (int z = 1; z < n_rows; z++)
	{
		cv::vconcat(merged_image, rows_array[z], merged_image);
	}

	//cv::imshow("image", merged_image);
	
	return merged_image;
}

bool Segmentation::watershed_segmentation(std::string image_name)//, PointCloudElement* pElement)
{
	//cv::Mat src;
	//cv::Mat median_image;
	//cv::Mat binary;
	//cv::Mat binary2;
	//cv::Mat image;
	//src = cv::imread(path + image_name, 1);
	////draw_raster_from_openCV_mat ("tile as read from file" + image_name, src,  pElement);
	////cv::imshow("src image read as seen by OpenCV", src);

	//cv::Mat CVtile32FC1;
	//cv::Mat CVtile8U;
	//src.convertTo(CVtile32FC1, CV_32FC1);
	//src.convertTo(CVtile8U, CV_8UC1);
	////draw_raster_from_openCV_mat ("tile float " + image_name, CVtile32FC1,  pElement);
	////draw_raster_from_openCV_mat ("tile uint" + image_name, CVtile8U,  pElement);
	////cv::imshow("src image as seen by OpenCV float", CVtile32FC1);
	////cv::imshow("src image as seen by OpenCV uint", CVtile8U);

	//cv::medianBlur (src, median_image, 5);

 //   image = median_image;
 //   //image = src;
 //   cv::cvtColor(image, binary, CV_BGR2GRAY);
	///*draw_raster_from_openCV_mat ("src image " + image_name, binary,  pElement);
	//cv::imshow("src image after gray conversion as seen by OpenCV", binary);*/
	////cv::normalize(binary, binary, 0, 255, cv::NORM_MINMAX, CV_8UC1);

	// cv::Scalar temp_mean;
	// cv::Scalar temp_std;
	// cv::Scalar temp_mode = cv_matrix_mode (binary); // on the test tile of the test point cloud is 109
	// cv::meanStdDev(binary, temp_mean, temp_std, cv::Mat());
	// double mean = temp_mean.val[0];
	// double std = temp_std.val[0];
	// double mode = temp_mode.val[0];
	// msg2 += "Tile " + StringUtilities::toDisplayString(k_for_process_all_point_cloud) + "\n" + "MEAN \n" + StringUtilities::toDisplayString(mean) + "\n"+ "STD DEV \n"+StringUtilities::toDisplayString(std)+ "\n" + "MODE \n"+StringUtilities::toDisplayString(mode)+ "\n\n";

	////http://stackoverflow.com/questions/17141535/how-to-use-the-otsu-threshold-in-opencv
	////cv::threshold(binary, binary, 180, 255, cv::THRESH_BINARY_INV + cv::THRESH_OTSU); // Currently, the Otsu’s method is implemented only for 8-bit images.
	//cv::threshold(binary, binary, mode + std::max(9.0, 1.1*(mean - mode)), 255, cv::THRESH_BINARY_INV); 
	//
	////cv::imshow("src image after gray conversion as seen by OpenCV", binary);

	////draw_raster_from_openCV_mat (image_name + " binary sa", binary,  pElement);

	//// Eliminate noise and smaller objects c++
 //    cv::Mat fg;
 //    cv::erode(binary, fg, cv::Mat(), cv::Point(-1,-1), 2);
	//
 //   // Identify image pixels without objects
 //   cv::Mat bg;
 //   cv::dilate(binary, bg, cv::Mat(), cv::Point(-1,-1), 3);
 //   cv::threshold(bg, bg, 1, 128, cv::THRESH_BINARY_INV);

 //   // Create markers image
 //   cv::Mat markers(binary.size(),CV_8U,cv::Scalar(0));
 //   markers = fg + bg;

 //   // Create watershed segmentation object 
 //   WatershedSegmenter segmenter;
 //   segmenter.setMarkers(markers);

 //   cv::Mat result = segmenter.process(image);
 //   
	// ////////////////// DISABLED WARNINGS AS ERRORS ///////////////////////
	//// these lines are needed to remove the tiles contours (watershed identifies as building countours also the tile contours, and I need to remove them otherwise there are problems with the connected component method)
	// result.col(1).copyTo(result.col(0)); // I disabled the warning treated as errors: to re-enable, add the enable warnings property sheet (see http://opticks.org/irclogs/%23opticks/%23opticks.2014-05-27-Tue.txt)
	// result.col(result.cols-2).copyTo(result.col(result.cols-1));//http://stackoverflow.com/questions/6670818/opencv-c-copying-a-row-column-in-a-mat-to-another
	// result.row(1).copyTo(result.row(0)); // I disabled the warning treated as errors: to re-enable, add the enable warnings property sheet
	// result.row(result.rows-2).copyTo(result.row(result.rows-1));

	//// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	// result_tiles_array[k_for_process_all_point_cloud] = result; // this line must be commented when using only one tile //
	//// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//
	//result.convertTo(result, CV_8U);

	///*draw_raster_from_openCV_mat (image_name + " final_result", result,  pElement);
	//draw_raster_from_openCV_mat (image_name + " markers sa", markers,  pElement);
	//draw_raster_from_openCV_mat (image_name + " bg sa", bg,  pElement);
	//draw_raster_from_openCV_mat (image_name + " fg sa", fg,  pElement);*/
	//
	//cv::waitKey(0);
	return true;
}