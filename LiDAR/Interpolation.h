//* CLASS USED TO GENERATE A DEM FROM THE ORIGINAL LAS FILE
//* FOR NOW IT SUPORTS ONLY THE NEARESTR NEIGHBOR INTERPOLATION METHOD, BUT IN FUTURE OTHER INTERPOLATION METHODS COULD BE ADDED


#pragma once
#include "PointCloudAccessor.h"
#include "PointCloudAccessorImpl.h"
#include "PointCloudDataDescriptor.h"
#include "PointCloudDataRequest.h"
#include "PointCloudElement.h"
#include "PointCloudView.h"
#include "ProgressTracker.h"
//#include "PseudocolorLayer.h"
#include "RasterElement.h"
#include "RasterUtilities.h"
#include "SpatialDataView.h"
#include "SpatialDataWindow.h"
#include "Statistics.h"
#include "switchOnEncoding.h" 
#include "Undo.h"
#include <limits>
//#include <boost/random/uniform_int.hpp>
//#include <boost/random/random_device.hpp>
//#include <boost/random/uniform_int_distribution.hpp>
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
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/core/core_c.h>
//#include <opencv2/core/eigen.hpp>
//#include <opencv2/opencv.hpp>
//#include <opencv2/features2d/features2d.hpp>
//#include <opencv2\features2d\features2d.hpp>
//#include <boost/numeric/ublas/matrix.hpp>
//#include <boost/numeric/ublas/io.hpp>


class Interpolation
{
public:
	Interpolation(void);
	~Interpolation(void);
	std::string interpolation_msg;
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Interpolation::generate_DEM(PointCloudElement* pElement, float post_spacing);
	bool Interpolation::print_DEM_on_file(std::string name_file, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> dem_RM);
};


