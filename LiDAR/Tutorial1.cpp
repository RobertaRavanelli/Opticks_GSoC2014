/*
 * The information in this file is
 * Copyright(c) 2007 Ball Aerospace & Technologies Corporation
 * and is subject to the terms and conditions of the
 * GNU Lesser General Public License Version 2.1
 * The license text is available from   
 * http://www.gnu.org/licenses/lgpl.html
 */
  
#include "AppVerify.h"
#include "PlugInArgList.h"
#include "PlugInManagerServices.h"
#include "PlugInRegistration.h"
#include "Progress.h"
#include "Tutorial1.h"

#include "DataAccessor.h"
#include "DataAccessorImpl.h"
#include "DataRequest.h"
#include "DesktopServices.h"
#include "ObjectFactory.h"

#include "PointCloudAccessor.h"
#include "PointCloudAccessorImpl.h"
#include "PointCloudDataDescriptor.h"
#include "PointCloudDataRequest.h"
#include "PointCloudElement.h"
#include "PointCloudView.h"
#include "ProgressTracker.h"
#include "PseudocolorLayer.h"
//#include "RasterElement.h"
//#include "RasterUtilities.h"
#include "SpatialDataView.h"
#include "SpatialDataWindow.h"
#include "Statistics.h"
#include "switchOnEncoding.h"
#include "Undo.h"
#include <limits>
#include "StringUtilities.h"
#include "Ransac.h"
#include <time.h>

REGISTER_PLUGIN_BASIC(LidarRoof, Tutorial1);

Tutorial1::Tutorial1()
{
   setDescriptorId("{732D7E3E-1CE0-4D3B-B3A9-7F7B5F6B11B0}");
   setName("LIDAR Roof Extraction");
   setDescription("Creating your first plug-in.");
   setCreator("Opticks Community");
   setVersion("Sample");
   setCopyright("Copyright (C) 2008, Ball Aerospace & Technologies Corp.");
   setProductionStatus(false);
   setType("Sample");
   setMenuLocation("[Point Cloud]/Roof Extraction");
   setAbortSupported(false);
}

Tutorial1::~Tutorial1()
{
}


bool Tutorial1::getInputSpecification(PlugInArgList*& pInArgList)
{
   VERIFY((pInArgList = Service<PlugInManagerServices>()->getPlugInArgList()) != NULL);
   pInArgList->addArg<Progress>(Executable::ProgressArg(), "Object for progress reporting.");
   pInArgList->addArg<PointCloudElement>(Executable::DataElementArg(), "The point cloud to process.");
   pInArgList->addArg<PointCloudView>(Executable::ViewArg(), NULL, "The view displaying the point cloud. If set, the displayed data will be set to classification.");
   return true;
}

bool Tutorial1::getOutputSpecification(PlugInArgList*& pOutArgList)
{
   VERIFY((pOutArgList = Service<PlugInManagerServices>()->getPlugInArgList()) != NULL);
   pOutArgList->addArg<double>("Minimum", "The minimum value");
   pOutArgList->addArg<double>("Maximum", "The maximum value");
   pOutArgList->addArg<unsigned int>("Count", "The number of points in the point cloud");
   return true;
}

bool Tutorial1::execute(PlugInArgList* pInArgList, PlugInArgList* pOutArgList)
{
   if (pInArgList == NULL)
   {
      return false;
   }
    
  ProgressTracker progress(pInArgList->getPlugInArgValue<Progress>(Executable::ProgressArg()),
      "Identifying buildings\n", "prova Roby", getDescriptorId());

  PointCloudElement* pElement = pInArgList->getPlugInArgValue<PointCloudElement>(Executable::DataElementArg());


   if (pElement == NULL)
   {
	  progress.report("A valid point cloud element must be provided.", 0, ERRORS, true);
      return false;
   }
  
  progress.report("Calculating", 0, NORMAL);

   double minX, maxX;
   int  count;

   pOutArgList->setPlugInArgValue("Minimum", &minX);
   pOutArgList->setPlugInArgValue("Maximum", &maxX);
   pOutArgList->setPlugInArgValue("Count", &count);
   
   Ransac prova = Ransac();
   
   //progress.report("Calculating point cloud statistics", 10, NORMAL);
   //prova.generate_point_cloud_statistics(pElement);
   
   progress.report("Generating DEM and raster", 20, NORMAL);
   //post_spacing is the pixel spacing of the dem matrix
   //static const float post_spacing = 0.1f;
   static const float post_spacing = 5.0f;//5.0f
   int n_rows = 8;// 4
   int n_cols = 10;// 5

   //prova.generate_raster_from_intensity(pElement, post_spacing);

   if(prova.generate_DEM(pElement, post_spacing, n_rows,n_cols) == true)
   {
	  progress.report("Segmenting buildings", 35, NORMAL);
	  //prova.watershed_segmentation("test_tile_float_opticks.png", pElement);
	  prova.process_all_point_cloud_with_watershed(n_rows, n_cols, pElement); //for example n_rows = 4 and n_cols = 5 : in this case the raster will be cut in 4 tiles horizontally and 5 vertically
	  //prova.pca_segmentation("prova0_4.png", pElement);
	  //prova.process_all_point_cloud_with_pca(n_rows, n_cols, pElement);
   }
    
   double RANSAC_threshold =  1.0; //0.00000001;0.02
   progress.report("Computing RANSAC", 40, NORMAL);
   prova.connected_components("buildings_for_connected_components.png", pElement);
   prova.Ransac_for_buildings(post_spacing, pElement, RANSAC_threshold, progress);
  

   
   //prova.ComputeModel(pElement, RANSAC_threshold);// treshold =0.02

  

   // this part is used o test the ransac implementation:
   // it must return a=0, b=0, c=1 and d=-10: the plane Z=10
   prova.msg2 += "___________________________\n\n";

   Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> matrix;
   matrix.setConstant(10000, 3, 0.0);


   for (int i =0; i< 9000; i++)
   {
	     matrix(i,0) = i*i;// % 100;
		 srand (time(NULL));
	     matrix(i,1) = 5*i+21;// rand() % 33;
	     matrix(i,2) = 10.0;
   }

   for (int i =9000; i< 10000; i++)
   {
	     matrix(i,0) = rand() % 10;
	     matrix(i,1) = rand() % 200;
	     matrix(i,2) = rand() % 700;
   }


   //prova.ComputeModel2(matrix, RANSAC_threshold);
   prova.ComputeModel2(matrix, RANSAC_threshold);
   prova.getSamples3(3, matrix.rows() - prova.n_best_inliers_count, prova.final_outliers);

   
   prova.print_result();
   progress.report("Printing messages, wait", 80, NORMAL);
   progress.report(prova.msg2, 90, WARNING);// only to see the message, it isn't a real warning
   progress.report(prova.msg1, 100, NORMAL);
   progress.upALevel();
   return true;
}
