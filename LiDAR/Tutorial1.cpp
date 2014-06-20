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
#include "RasterElement.h"
#include "RasterUtilities.h"
#include "SpatialDataView.h"
#include "SpatialDataWindow.h"
#include "Statistics.h"
#include "switchOnEncoding.h"
#include "Undo.h"
#include <limits>

#include <Eigen/Core> //http://eigen.tuxfamily.org/index.php?title=Visual_Studio
#include <Eigen/Eigenvalues>
//#include <Eigen/StdVector>
//#include <Eigen/Core>
//#include <Eigen/Geometry>
//#include <Eigen/SVD>
//#include <Eigen/LU>
//#include <Eigen/Dense>
#include <cmath>


#include <gdal/gdal.h>
#include <gdal/gdal_priv.h>
#include <gdal/gdal_alg.h>
//#include  <opencv2/>
#include "StringUtilities.h"
#include "Ransac.h"
#include <boost/random/uniform_int.hpp>
#include <boost/random/random_device.hpp>
#include <boost/random/uniform_int_distribution.hpp>

REGISTER_PLUGIN_BASIC(LidarRoof, Tutorial1);

namespace
{
template<typename T>
//void assign(T* pData, int bin)
//{
//   *pData = static_cast<T>(bin);
//}

void updateStatistics(T* pData, double& min, double& max, double& total)
   {
      min = std::min(min, static_cast<double>(*pData));
      max = std::max(max, static_cast<double>(*pData));
      total += *pData;
   }
}

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
      "Calculating pointcloud parameters\n", "prova Roby", getDescriptorId());

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
   static const float post_spacing = 5.0;
   prova.generate_DEM(pElement, post_spacing); 
   //prova.generate_raster_from_intensity(pElement, post_spacing);
   progress.report("Computing RANSAC", 40, NORMAL);
   //prova.ComputeModel(pElement);

   progress.report(prova.msg2, 90, WARNING);// only to see the message, it isn't a real warning
   progress.report(prova.msg1, 100, NORMAL);
   progress.upALevel();
   return true;
}
