/*
 * The information in this file is
 * Copyright(c) 2007 Ball Aerospace & Technologies Corporation
 * and is subject to the terms and conditions of the
 * GNU Lesser General Public License Version 2.1
 * The license text is available from   
 * http://www.gnu.org/licenses/lgpl.html
 */
#include "Segmentation.h"
#include "StringUtilities.h"

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
		   segmentation_msg += "problem occurred in merging tiles\n\n";
		   //return false;
	   }
	}
	
	merged_image = rows_array[0];
	for (int z = 1; z < n_rows; z++)
	{
		cv::vconcat(merged_image, rows_array[z], merged_image);
	}
	
	return merged_image;
}

double Segmentation::getOrientation(std::vector<cv::Point> &pts, cv::Mat &img)
{    
	 if (pts.size() == 0) return false;

    //Construct a buffer used by the pca analysis
    cv::Mat data_pts = cv::Mat(pts.size(), 2, CV_64FC1);
    for (int i = 0; i < data_pts.rows; ++i)
    {
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }


    //Perform PCA analysis
    cv::PCA pca_analysis(data_pts, cv::Mat(), CV_PCA_DATA_AS_ROW);

    //Store the position of the object
    cv::Point2i pos = cv::Point2i(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
                     static_cast<int>(pca_analysis.mean.at<double>(0, 1)));


    //Store the eigenvalues and eigenvectors
    std::vector<cv::Point2d> eigen_vecs(2);
    std::vector<double> eigen_val(2);
    for (int i = 0; i < 2; ++i)
    {
        eigen_vecs[i] = cv::Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                                pca_analysis.eigenvectors.at<double>(i, 1));

        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
    }

    // Draw the principal components
    cv::circle(img, pos, 3, CV_RGB(255, 0, 255), 2);
    cv::line(img, pos, pos + 0.02 * cv::Point(static_cast<int>(eigen_vecs[0].x * eigen_val[0]),static_cast<int> (eigen_vecs[0].y * eigen_val[0])) , CV_RGB(255, 255, 0));
    cv::line(img, pos, pos + 0.02 * cv::Point(static_cast<int>(eigen_vecs[1].x * eigen_val[1]), static_cast<int>(eigen_vecs[1].y * eigen_val[1])) , CV_RGB(0, 255, 255));
	
    return atan2(eigen_vecs[0].y, eigen_vecs[0].x);
}

bool Segmentation::pca_segmentation(std::string image_name)//, PointCloudElement* pElement)
{
	// http://robospace.wordpress.com/2013/10/09/object-orientation-principal-component-analysis-opencv/
	cv::Mat bw, img = cv::imread(path + image_name); 
	 
    // Convert it to greyscale
    cv::cvtColor(img, bw, CV_BGR2GRAY);

	cv::equalizeHist(bw, bw);// for the test point cloud, it improves the result, for SD point cloud it improves the results for the superior tiles, it worsens the result for the central tiles (those with buildings of H shape)
	//cv::normalize(bw, bw, 0, 255, cv::NORM_MINMAX, CV_8UC1); // it doesn't improve but it also doesn' t worsen the result for the test point cloud; same behaviour for he SD point cloud; pratically it is useless

     cv::Scalar temp_mean;
	 cv::Scalar temp_std;
	 cv::Scalar temp_mode = cv_matrix_mode (bw); // on the test tile of the test point cloud is 109
	 cv::meanStdDev(bw, temp_mean, temp_std, cv::Mat());
	 double mean = temp_mean.val[0];
	 double std = temp_std.val[0];
	 double mode = temp_mode.val[0];
	 segmentation_msg += "Tile " + StringUtilities::toDisplayString(k_for_process_all_point_cloud) + "\n" + "MEAN \n"+StringUtilities::toDisplayString(mean) + "\n"+ "STD DEV \n"+StringUtilities::toDisplayString(std)+ "\n"  "MODE \n"+StringUtilities::toDisplayString(mode) + "\n" + "\n";	
	
	 // Apply thresholding
	 //cv::threshold(bw, bw, 150, 255, CV_THRESH_BINARY + cv::THRESH_OTSU);
	 cv::threshold(bw, bw, mode + 1.1*(mean - mode), 255, cv::THRESH_BINARY_INV); 

    // Find all objects of interest
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(bw, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

    // For each object
    for (size_t i = 0; i < contours.size(); ++i)
    {
        // Calculate its area
        double area = cv::contourArea(cv::Mat(contours[i]));
		
        // Ignore if too small or too large
        if (area < 1e2 || 1e5 < area) continue;

        // Draw the contour
        cv::drawContours(img, contours, i, CV_RGB(255, 0, 0), 2, 8, hierarchy, 0);

        // Get the object orientation
        Segmentation::getOrientation(contours[i], img);
    }

	////////////////// DISABLED WARNINGS AS ERRORS ///////////////////////
	// these lines are needed to remove the tiles contours (watershed al
    img.col(1).copyTo(img.col(0)); // I disabled the warning treated as errors: to re-enable, add the enable warnings property sheet (see http://opticks.org/irclogs/%23opticks/%23opticks.2014-05-27-Tue.txt)
	img.col(img.cols-2).copyTo(img.col(img.cols-1));//http://stackoverflow.com/questions/6670818/opencv-c-copying-a-row-column-in-a-mat-to-another
	img.row(1).copyTo(img.row(0)); // I disabled the warning treated as errors: to re-enable, add the enable warnings property sheet
	img.row(img.rows-2).copyTo(img.row(img.rows-1));


	// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	result_tiles_array[k_for_process_all_point_cloud] = img;     // this line must be commented when using only one tile //
	// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	/*img.convertTo(img, CV_32FC1);
	draw_raster_from_openCV_mat ("pca result", img,  pElement);*/
	cv::waitKey(0);


	return true;
}

bool Segmentation::watershed_segmentation(std::string image_name)//, PointCloudElement* pElement)
{
	cv::Mat src;
	cv::Mat median_image;
	cv::Mat binary;

	src = cv::imread(path + image_name, 1);
	cv::medianBlur (src, median_image, 5);
    cv::cvtColor(median_image, binary, CV_BGR2GRAY);
	
	 cv::Scalar temp_mean;
	 cv::Scalar temp_std;
	 cv::Scalar temp_mode = cv_matrix_mode (binary); // on the test tile of the test point cloud is 109
	 cv::meanStdDev(binary, temp_mean, temp_std, cv::Mat());
	 double mean = temp_mean.val[0];
	 double std = temp_std.val[0];
	 double mode = temp_mode.val[0];
	 segmentation_msg += "Tile " + StringUtilities::toDisplayString(k_for_process_all_point_cloud) + "\n" + "MEAN \n" + StringUtilities::toDisplayString(mean) + "\n"+ "STD DEV \n"+StringUtilities::toDisplayString(std)+ "\n" + "MODE \n"+StringUtilities::toDisplayString(mode)+ "\n\n";

	cv::threshold(binary, binary, mode + std::max(9.0, 1.1*(mean - mode)), 255, cv::THRESH_BINARY_INV); 

	// Eliminate noise and smaller objects c++
     cv::Mat fg;
     cv::erode(binary, fg, cv::Mat(), cv::Point(-1,-1), 2);
	
    // Identify image pixels without objects
    cv::Mat bg;
    cv::dilate(binary, bg, cv::Mat(), cv::Point(-1,-1), 3);
    cv::threshold(bg, bg, 1, 128, cv::THRESH_BINARY_INV);

    // Create markers image
    cv::Mat markers(binary.size(),CV_8U,cv::Scalar(0));
    markers = fg + bg;

    // Create watershed segmentation object 
    WatershedSegmenter2 segmenter;
    segmenter.setMarkers(markers);

    cv::Mat result = segmenter.process(median_image);
    
	 ////////////////// DISABLED WARNINGS AS ERRORS ///////////////////////
	// these lines are needed to remove the tiles contours (watershed identifies as building countours also the tile contours, and I need to remove them otherwise there are problems with the connected component method)
	 result.col(1).copyTo(result.col(0)); // I disabled the warning treated as errors: to re-enable, add the enable warnings property sheet (see http://opticks.org/irclogs/%23opticks/%23opticks.2014-05-27-Tue.txt)
	 result.col(result.cols-2).copyTo(result.col(result.cols-1));//http://stackoverflow.com/questions/6670818/opencv-c-copying-a-row-column-in-a-mat-to-another
	 result.row(1).copyTo(result.row(0)); // I disabled the warning treated as errors: to re-enable, add the enable warnings property sheet
	 result.row(result.rows-2).copyTo(result.row(result.rows-1));

	// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	 result_tiles_array[k_for_process_all_point_cloud] = result; // this line must be commented when using only one tile //
	// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	
	result.convertTo(result, CV_8U);

	cv::waitKey(0);
	return true;
}

cv::Scalar Segmentation::cv_matrix_mode (cv::Mat image)
{
	 // https://github.com/arnaudgelas/OpenCVExamples/blob/master/cvMat/Statistics/Mode/Mode.cpp
	  double m=(image.rows*image.cols)/2;
       int bin0=0, bin1=0, bin2=0;
      cv::Scalar mode;
       mode.val[0]=-1;
       mode.val[1]=-1;
       mode.val[2]=-1;
        int histSize = 256;
	    float range[] = { 0, 256 } ;
	    const float* histRange = { range };
	    bool uniform = true;
	    bool accumulate = false;
	    cv::Mat hist0, hist1, hist2;
		std::vector<cv::Mat> channels;
	    cv::split( image, channels );
	    cv::calcHist( &channels[0], 1, 0, cv::Mat(), hist0, 1, &histSize, &histRange, uniform, accumulate );
	/*	cv::calcHist( &channels[1], 1, 0, cv::Mat(), hist1, 1, &histSize, &histRange, uniform, accumulate );
		cv::calcHist( &channels[2], 1, 0, cv::Mat(), hist2, 1, &histSize, &histRange, uniform, accumulate );*/

		for (int i=0; i<256 ;i++)
		{
			if (bin0<cvRound(hist0.at<float>(i)))
			{
				bin0=cvRound(hist0.at<float>(i));
				mode.val[0]=i;

			}
		/*	if (bin1<cvRound(hist1.at<float>(i)))
			{
				bin1=cvRound(hist1.at<float>(i));
				mode.val[1]=i;
			}
			if (bin2<cvRound(hist2.at<float>(i)))
			{
				bin2=cvRound(hist2.at<float>(i));
				mode.val[2]=i;
			}*/
		}

		return mode;
}

cv::Mat Segmentation::process_all_point_cloud_with_watershed(cv::Mat original_tiles_merged, int n_rows, int n_cols)//, PointCloudElement* pElement)
{
	k_for_process_all_point_cloud = 0;
	result_tiles_array.resize(n_rows * n_cols);  
	
	 for(int i = 0; i < n_rows; i++) 
	    {  //i is row index
             for(int j = 0; j < n_cols; j++)
	         {
		       std::ostringstream oss;
               oss << i << "_" << j << ".png";
               std::string name = oss.str();
               Segmentation::watershed_segmentation("Tiles/"+ name);
			   //Segmentation::pca_segmentation("Tiles/"+ name);
			   k_for_process_all_point_cloud++;
	         }
         }
	
	cv::Mat merged_mat = Segmentation::merge_tiles(result_tiles_array, n_rows, n_cols);//this is a trinary image: we need to convert it in a binary image
	cv::imshow("merged result watershed", merged_mat);
	cv::imwrite(path + "Tiles/result_watershed.png", merged_mat);

	cv::Mat mask;
	cv::threshold(merged_mat, mask, 250.0f, 1.0f, cv::THRESH_BINARY_INV);// 255 is background; mask is a CV8U image, since merged image is of this type
	mask.convertTo(mask, CV_32FC1);//I need float image to multiply with another float image
	
	segmentation_msg += "\nWatershed raster:\nwidth "+ StringUtilities::toDisplayString(mask.rows) + "; height "+ StringUtilities::toDisplayString(mask.cols)+"\n\n";

    cv::waitKey(0);
    return mask;
}

bool Segmentation::process_all_point_cloud_with_pca(int n_rows, int n_cols)//, PointCloudElement* pElement)
{
	k_for_process_all_point_cloud = 0;
	result_tiles_array.resize(n_rows * n_cols);  
	
	 for(int i = 0; i < n_rows; i++) 
	 {  //i is row index
        for(int j = 0; j < n_cols; j++)
	    {
		std::ostringstream oss;
        oss << i << "_" << j << ".png";
        std::string name = oss.str();
		pca_segmentation("Tiles/" + name);
		k_for_process_all_point_cloud++;
	    }
      }

	cv::Mat merged_mat = Segmentation::merge_tiles(result_tiles_array, n_rows, n_cols);
	cv::imshow("merged result pca", merged_mat);
	cv::imwrite(path + "Tiles/result_pca.png", merged_mat);

	cv::waitKey(0);
	return true;
}

void Segmentation::FindBlobs(const cv::Mat &binary, std::vector < std::vector<cv::Point2i> > &blobs)
{
    blobs.clear();

    // Fill the label_image with the blobs
    // 0  - background
    // 1  - unlabelled foreground
    // 2+ - labelled foreground

    cv::Mat label_image;
	binary.convertTo(label_image, CV_32FC1);

    int label_count = 2; // starts at 2 because 0,1 are used already

	// http://areshopencv.blogspot.it/2011/12/blob-detection-connected-component-pure.html
	// this version works with openCV 220 (for the most recent version of OpenCV we need to change the code)
	for(int y=0; y < binary.rows; y++)
	{// y is row index
                for(int x=0; x < binary.cols; x++) 
				{// x is column index
                    float checker = label_image.at<float>(y,x); //need to look for float and not int as the scalar value is of type double
                    cv::Rect rect;
                    if(checker == 1) 
					{
                        //fill region from a point
                        cv::floodFill(label_image, cv::Point(x,y), cv::Scalar(label_count), &rect, cv::Scalar(0), cv::Scalar(0), 4);
                        label_count++;
                        
                        //a vector of all points in a blob
                        std::vector<cv::Point> blob;

                        for(int i=rect.y; i < (rect.y+rect.height); i++)
						{
                            for(int j=rect.x; j < (rect.x+rect.width); j++)
							{
                                float chk = label_image.at<float>(i,j);
                                if(chk == label_count-1) 
								{
                                    blob.push_back(cv::Point(j,i));
                                }                        
                            }
                        }
                        //place the points of a single blob in a grouping
                        //a vector of vector points
                        blobs.push_back(blob);
                    }
                }
            }
	       segmentation_msg +="\n"+ StringUtilities::toDisplayString(label_count) + " identified buildings with connected components\n\n";
}

bool Segmentation::connected_components(cv::Mat input)//, std::string image_name)//,  PointCloudElement* pElement)
{
   cv::Mat output = cv::Mat::zeros(input.size(), CV_8UC3);
   cv::Mat classified_buildings = cv::Mat::zeros(input.size(), CV_8UC1);

   cv::Mat binary;
   cv::threshold(input, binary, 0.0, 1.0, cv::THRESH_BINARY);

   //blobs is now a global variable
   Segmentation::FindBlobs(binary, blobs);

    // Randomy color the blobs
    for(int i = 0; i < blobs.size(); i++)
	{// i index is the building (blob) index
        unsigned char r = unsigned char(255 * (rand()/(1.0 + RAND_MAX)));
        unsigned char g = unsigned char(255 * (rand()/(1.0 + RAND_MAX)));
        unsigned char b = unsigned char(255 * (rand()/(1.0 + RAND_MAX)));
		
        for(int j = 0; j < blobs[i].size(); j++) 
		{// j index is the pixel index for the single building ()
            int pixel_column = blobs[i][j].x;
            int pixel_row = blobs[i][j].y;			

			classified_buildings.at<uchar>(pixel_row, pixel_column) = i;

            output.at<cv::Vec3b>(pixel_row, pixel_column)[0] = b;
            output.at<cv::Vec3b>(pixel_row, pixel_column)[1] = g;
            output.at<cv::Vec3b>(pixel_row, pixel_column)[2] = r;
        }
    }

	//draw_raster_from_openCV_mat ("classsified buildings", classified_buildings,  pElement);
	
    cv::imshow("RGB classsified buildings", output);
    
	cv::imwrite(path+"Tiles\result_con_comp.png", output);
	cv::waitKey(0);
	return true;
}

bool Segmentation::draw_buildings_contours(cv::Mat image)
{
	 image.convertTo(image, CV_8U);// conversion needed, otherwise findContours method doesn't work 
	 std::vector<std::vector<cv::Point>> contours; // Detected contours. Each contour is stored as a vector of points.
     std::vector<cv::Vec4i> hierarchy;
     cv:: findContours(image, contours, hierarchy, CV_RETR_TREE, CV_RETR_CCOMP, cv::Point2i(0, 0));

     cv::Mat drawing = cv::Mat::zeros(image.size(), CV_8UC3);  
     cv::RNG rng(12345);
     for( int i = 0; i< contours.size(); i++ )
     {
        cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0,cv::Point2i() );
     }
     segmentation_msg += "\n" + StringUtilities::toDisplayString(contours.size()) + " identified buildings with find contours method\n\n";
   
     // Show in a window
     cv::imshow("Building contours", drawing);

	 return true;
}