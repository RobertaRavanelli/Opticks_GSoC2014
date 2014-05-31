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

//#include <gdal/gdal.h>
//#include <gdal/gdal_priv.h>
//#include <gdal/gdal_alg.h>
//#include  <opencv2/>
#include "StringUtilities.h"
#include "Ransac.h"
#include <boost/random/uniform_int.hpp>
#include <boost/random/random_device.hpp>
#include <boost/random/uniform_int_distribution.hpp>

REGISTER_PLUGIN_BASIC(OpticksTutorial, Tutorial1);

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
   setDescriptorId("{5D8F4DD0-9B20-42B1-A060-589DFBC85D00}");
   setName("Tutorial 1");
   setDescription("Creating your first plug-in.");
   setCreator("Opticks Community");
   setVersion("Sample");
   setCopyright("Copyright (C) 2008, Ball Aerospace & Technologies Corp.");
   setProductionStatus(false);
   setType("Sample");
   setMenuLocation("[Tutorial]/Tutorial 1");
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
   
   /* Main processing loop */
   FactoryResource<PointCloudDataRequest> req;
   req->setWritable(true);
   PointCloudAccessor acc = PointCloudAccessor (pElement->getPointCloudAccessor(req.release()));
   if (!acc.isValid())
   {
      progress.report("Unable to write to point cloud.", 0, ERRORS, true);
      return false;
   }
   const PointCloudDataDescriptor* pDesc = static_cast<const PointCloudDataDescriptor*>(pElement->getDataDescriptor());

   // inizializza il min e il max con il valore più alto e più basso possibile
   double minX, maxX, minY, maxY, minZ, maxZ;
   minX = minY = minZ = std::numeric_limits<double>::max(); //numero molto grande: facile trovarne uno più piccolo
   maxX = maxY = maxZ = -minX; //numero molto piccolo: facile trovarne uno più grande
   double sumX, sumY, sumZ; // servono per la media
   sumX = sumY = sumZ = 0;

   std::vector<double> x_coordinates  =  std::vector<double> (pDesc->getPointCount());
   std::vector<double> y_coordinates  =  std::vector<double> (pDesc->getPointCount());
   std::vector<double> z_coordinates  =  std::vector<double> (pDesc->getPointCount());
  

   // ciclo che accede a tutti i punti della point cloud
   int prog = 0;
   int points_number =0;//mi serve a verificare che il numero di punti della pointcloud sia uguale a quello del descriptor
   const uint32_t adv = pDesc->getPointCount() / 50;// serve per il progress report
   
   acc->toIndex(0);
   for (size_t idx = 0; idx < pDesc->getPointCount(); ++idx)
   {
	  if (!acc.isValid())
      {
         progress.report("Unable to access data.", 0, ERRORS, true);
         return false;
      }

	  if (idx % adv == 0)
      {
         progress.report("Calculating extents", ++prog, NORMAL);
      }

      if (!acc->isPointValid())
      {
         acc->nextValidPoint();
         continue;
      }

	  sumX += acc->getXAsDouble(true);
	  sumY += acc->getYAsDouble(true);
	  sumZ += acc->getZAsDouble(true);

	  minX = std::min(minX, acc->getXAsDouble(true));
      maxX = std::max(maxX, acc->getXAsDouble(true));

	  minY = std::min(minY, acc->getYAsDouble(true));
      maxY = std::max(maxY, acc->getYAsDouble(true));

	  minZ = std::min(minZ, acc->getZAsDouble(true));
      maxZ = std::max(maxZ, acc->getZAsDouble(true));
	  
	  points_number ++;
	  acc->nextValidPoint();//sposta l'accessor al punto successivo
   }
   acc->toIndex(0);// because I move the acccessor to the first element, in the case I need it once more

   double meanX = sumX / static_cast<double>(points_number);
   double meanY = sumY / static_cast<double>(points_number);
   double meanZ = sumZ / static_cast<double>(points_number);

   // From ASPRS Las Specification

   /*X, Y, and Z scale factors: The scale factor fields contain a double floating point value that is used to scale the corresponding X, Y, and Z long values within the point records. 
   The corresponding X, Y, and Z scale factor must be multiplied by the X, Y, or Z point record value to get the actual X, Y, or Z coordinate. 
   For example, if the X, Y, and Z coordinates are intended to have two decimal point values, then each scale factor will contain the number 0.01.*/

  /* X, Y, and Z offset: The offset fields should be used to set the overall offset for the point records. 
   In general these numbers will be zero, but for certain cases the resolution of the point data may not be large enough for a given projection system. 
   However, it should always be assumed that these numbers are used. So to scale a given X from the point record, take the point record X multiplied by the X scale factor, and then add the X offset.
   Xcoordinate = (Xrecord * Xscale) + Xoffset
   Ycoordinate = (Yrecord * Yscale) + Yoffset
   Zcoordinate = (Zrecord * Zscale) + Zoffset*/

   //Max and Min X, Y, Z: The max and min data fields are the actual unscaled extents of the LAS point file data, specified in the coordinate system of the LAS data.

   // VALORI DESUNTI DALL'HEADER DEL LAS FILE (sono immagazzinati nel descriptor): per verifica li confronterò con i valori calcolati considerando tutti i punti della nuvola (ciclo for)
   unsigned int count = pDesc->getPointCount();
   double xmin_header = pDesc->getXMin() * pDesc->getXScale() + pDesc->getXOffset();
   double xmax_header = pDesc->getXMax() * pDesc->getXScale() + pDesc->getXOffset();
   double ymin_header = pDesc->getYMin() * pDesc->getYScale() + pDesc->getYOffset();
   double ymax_header = pDesc->getYMax() * pDesc->getYScale() + pDesc->getYOffset();
   double zmin_header = pDesc->getZMin() * pDesc->getZScale() + pDesc->getZOffset();
   double zmax_header = pDesc->getZMax() * pDesc->getZScale() + pDesc->getZOffset();

    std::string msg = "\nScale along x: " + StringUtilities::toDisplayString(pDesc->getXScale()) +"\n"+
		              "Offset along x: " + StringUtilities::toDisplayString(pDesc->getXOffset()) + "\n"+

		              "Maximum value along x: " + StringUtilities::toDisplayString(maxX) + "\n"+
		              "Verify with the header: "+ StringUtilities::toDisplayString(xmax_header) + "\n"+
                      "Minimum value along x: " + StringUtilities::toDisplayString(minX) + "\n"+
					  "Verify with the header: "+ StringUtilities::toDisplayString(xmin_header) + "\n"+
					  "Avrage along x: "+ StringUtilities::toDisplayString(meanX)+ "\n"+"\n"+
					  
					  "Scale along y: " + StringUtilities::toDisplayString(pDesc->getYScale()) +"\n"+
		              "Offset along y: " + StringUtilities::toDisplayString(pDesc->getYOffset()) + "\n"+

					  "Maximum value along y: " + StringUtilities::toDisplayString(maxY) + "\n"+
		              "Verify with the header: "+ StringUtilities::toDisplayString(ymax_header) + "\n"+
                      "Minimum value along y: " + StringUtilities::toDisplayString(minY) + "\n"+
					  "Verify with the header: "+ StringUtilities::toDisplayString(ymin_header) + "\n"+
					  "Avrage along y: "+ StringUtilities::toDisplayString(meanY)+ "\n"+"\n"+
					  
					  "Scale along z: " + StringUtilities::toDisplayString(pDesc->getZScale()) +"\n"+
		              "Offset along z: " + StringUtilities::toDisplayString(pDesc->getZOffset()) + "\n"+

					  "Maximum value along z: " + StringUtilities::toDisplayString(maxZ) + "\n"+
		              "Verify with the header: "+ StringUtilities::toDisplayString(zmax_header) + "\n"+
					  "Minimum value along z: " + StringUtilities::toDisplayString(minZ) + "\n"+
					  "Verify with the header: "+ StringUtilities::toDisplayString(zmin_header) + "\n"+
					  "Average along z: "+ StringUtilities::toDisplayString(meanZ)+ "\n"+"\n"+
					  
					  "Number of points in the pont cloud: " + StringUtilities::toDisplayString(points_number)+"\n"+
					  "Verify with the header: "+ StringUtilities::toDisplayString(count);
	
   pOutArgList->setPlugInArgValue("Minimum", &minX);
   pOutArgList->setPlugInArgValue("Maximum", &maxX);
   pOutArgList->setPlugInArgValue("Count", &count);
   
   Ransac prova = Ransac();
   
   prova.ComputeModel(pElement);

   progress.report( prova.msg2, 90, WARNING);// only to see the message, it isn't a real warning
   progress.report(msg, 100, NORMAL);
   progress.upALevel();
   return true;
}
