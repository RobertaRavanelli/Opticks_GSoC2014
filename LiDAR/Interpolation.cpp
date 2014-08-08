//* CLASS USED TO GENERATE A DEM FROM THE ORIGINAL LAS FILE
//* FOR NOW IT SUPORTS ONLY THE NEAREST NEIGHBOR INTERPOLATION METHOD, BUT IN FUTURE OTHER INTERPOLATION METHODS COULD BE ADDED

/*
 * The information in this file is
 * Copyright(c) 2007 Ball Aerospace & Technologies Corporation
 * and is subject to the terms and conditions of the
 * GNU Lesser General Public License Version 2.1
 * The license text is available from   
 * http://www.gnu.org/licenses/lgpl.html
 */

#include "Interpolation.h"
#include "Progress.h"
#include "ProgressResource.h"

Interpolation::Interpolation(void)
{
}


Interpolation::~Interpolation(void)
{
}

//bool Interpolation::generate_DEM(PointCloudElement* pElement, float post_spacing, int n_rows_tiles, int n_cols_tiles) //post_spacing is the pixel spacing of the dem matrix
Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Interpolation::generate_DEM(PointCloudElement* pElement, float post_spacing) // int n_rows_tiles, int n_cols_tiles) //post_spacing is the pixel spacing of the dem matrix
{
	   StepResource pStep("Generating DEM", "app", "ffe16048-1e58-11e4-b4be-b2227cce2b54");
	   
	   ProgressResource pResource("ProgressBar");
	   Progress *pProgress = pResource.get(); 
	   //pProgress-> setSettingAutoClose(false);
	   pProgress-> setSettingAutoClose(true);

	   Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> demRM;// dem stored in a Row major eigen matrix

	   /* Main processing loop */
	   FactoryResource<PointCloudDataRequest> req;
	   req->setWritable(true);
	   PointCloudAccessor acc(pElement->getPointCloudAccessor(req.release()));
	   if (!acc.isValid())
	   {
		   interpolation_msg += "Unable to write to point cloud for generating DEM.\n";
		   pProgress->updateProgress("Unable to write to point cloud for generating DEM.", 0, ERRORS);
		   pStep->finalize(Message::Abort, interpolation_msg);
		   return demRM.matrix(); // in this way it should return NULL matrix, see http://eigen.tuxfamily.org/dox/classEigen_1_1Matrix.html: Matrix(): For dynamic-size matrices, creates an empty matrix of size 0. Does not allocate any array. Such a matrix is called a null matrix. This constructor is the unique way to create null matrices: resizing a matrix to 0 is not supported.
	   }
	   const PointCloudDataDescriptor* pDesc = static_cast<const PointCloudDataDescriptor*>(pElement->getDataDescriptor());
	   double xMin = pDesc->getXMin() * pDesc->getXScale() + pDesc->getXOffset();
	   double xMax = pDesc->getXMax() * pDesc->getXScale() + pDesc->getXOffset();
	   double yMin = pDesc->getYMin() * pDesc->getYScale() + pDesc->getYOffset();
	   double yMax = pDesc->getYMax() * pDesc->getYScale() + pDesc->getYOffset();

	   int mDim = static_cast<int>(std::ceil((xMax - xMin) / post_spacing));  //columns
	   int nDim = static_cast<int>(std::ceil((yMax - yMin) / post_spacing)); //rows
	   xMax = xMin + mDim * post_spacing;
	   yMin = yMax - nDim * post_spacing;

	   const float badVal = -9999.f;
	   demRM.setConstant(nDim, mDim, badVal);

	   int prog = 0;
	   uint32_t adv = pDesc->getPointCount() / 100;
	   for (size_t idx = 0; idx < pDesc->getPointCount(); ++idx)
	   {
		  if (!acc.isValid())
		  {
			  interpolation_msg += "Unable to access data for generating DEM.\n";
			  pProgress->updateProgress("Unable to access data for generating DEM.", 0, ERRORS);
			  pStep->finalize(Message::Abort, interpolation_msg);
			  return demRM.matrix();// in this way it should return NULL matrix, see http://eigen.tuxfamily.org/dox/classEigen_1_1Matrix.html: Matrix(): For dynamic-size matrices, creates an empty matrix of size 0. Does not allocate any array. Such a matrix is called a null matrix. This constructor is the unique way to create null matrices: resizing a matrix to 0 is not supported.
		  }
		  if (idx % adv == 0)
		  {
			 //progress.report("Generating DEM", ++prog, NORMAL);
			  pProgress->updateProgress("Generating DEM", ++prog, NORMAL);
		  }
		  if (!acc->isPointValid())
		  {
			 acc->nextValidPoint();
			 continue;
		  }
		  double x = acc->getXAsDouble(true);
		  double y = acc->getYAsDouble(true);
		  float z = static_cast<float>(acc->getZAsDouble(true));
		  
		  // calculate nearest DEM point
		  int xIndex = std::max(0, static_cast<int>(std::floor((x - xMin) / post_spacing)));
		  int yIndex = std::max(0, static_cast<int>(std::floor((yMax - y) / post_spacing)));
		  
		  float demVal = demRM(yIndex, xIndex);
		  if (demVal == badVal || demVal < z)
		  {
			 demRM(yIndex, xIndex) = z; 
		  }

		  acc->nextValidPoint();
	   }
	   pProgress->updateProgress("DEM generation is complete.", 100, NORMAL);
	   pStep->finalize();

	//cv::Mat CVdemRM(static_cast<int>(demRM.rows()), static_cast<int>(demRM.cols()), CV_32FC1, demRM.data());
	return demRM;
}

bool Interpolation::print_DEM_on_file(std::string name_file, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> demRM)
{
	StepResource pStep("Writing DEM on a text file", "app", "c0c1c382-1f1e-11e4-b0cb-b2227cce2b54");
	ProgressResource pResource("ProgressBar");
	Progress *pProgress = pResource.get(); 
	//pProgress-> setSettingAutoClose(false);
    pProgress-> setSettingAutoClose(true);
	
	std::ofstream dem_file;
	dem_file.open (name_file);
    dem_file << "DEM generated from the input LAS file\n";
	dem_file << "i" << '\t' << "j" << '\t' << "z(i,j)" << '\n';  
	for (int row = 0; row < demRM.rows(); row++)
	{
			for (int col = 0; col <demRM.cols(); col++)
			{
				dem_file << row << '\t' << col << '\t' << demRM(row, col) << '\n';  
			}
			pProgress->updateProgress("Writing DEM on a text file", row * 100 / demRM.rows(), NORMAL);
	}
	dem_file.close();

    pProgress->updateProgress("DEM text file written.", 100, NORMAL);
	pStep->finalize();
	return true;
}