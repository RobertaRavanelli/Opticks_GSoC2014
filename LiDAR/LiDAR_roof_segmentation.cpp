

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
#include "LiDAR_roof_segmentation.h"


#include "DataAccessor.h"
#include "DataAccessorImpl.h"
#include "DataRequest.h"
#include "DesktopServices.h"
#include "ObjectFactory.h"
//#include <ModelServices.h>
//#include "Window.h"
#include "MessageLogResource.h"

 
#include "PointCloudAccessor.h"
#include "PointCloudAccessorImpl.h"
#include "PointCloudDataDescriptor.h"
#include "PointCloudDataRequest.h"
#include "PointCloudElement.h"
#include "PointCloudView.h"
#include "ProgressTracker.h"
//#include "PseudocolorLayer.h"
//#include "RasterElement.h"
//#include "RasterUtilities.h"
//#include "SpatialDataView.h" 
//#include "SpatialDataWindow.h"
#include "Statistics.h"
#include "switchOnEncoding.h"
#include "Undo.h"
#include <limits>
#include "StringUtilities.h"
#include "Ransac.h"
#include <time.h>
#include <QtGui/QApplication>
#include <QtGui/QMessageBox>
#include "SessionItemSerializer.h"
#include "Interpolation.h"
#include "Segmentation.h"

REGISTER_PLUGIN_BASIC(LidarRoof, LiDAR_roof_segmentation);

LiDAR_roof_segmentation::LiDAR_roof_segmentation():
   mpGui(NULL)
{
   setDescriptorId("{09d9618e-1d57-11e4-ba18-b2227cce2b54}");
   setName("LIDAR Roof Extraction 2");
   setDescription("Creating your first plug-in.");
   setCreator("Opticks Community");
   setVersion("Sample");
   setCopyright("Copyright (C) 2008, Ball Aerospace & Technologies Corp.");
   setProductionStatus(false);
   setType("Sample");
   setMenuLocation("[Point Cloud]/Roof Extraction 2");
   setAbortSupported(false);
}

LiDAR_roof_segmentation::~LiDAR_roof_segmentation()
{
	 /*Service<DesktopServices> pDesktop;
     pDesktop->detach(SIGNAL_NAME(DesktopServices, AboutToShowPropertiesDialog),
     Slot(this, &Gui::updatePropertiesDialog));
     pDesktop->deleteWindow(mpDockWindow);*/
}


bool LiDAR_roof_segmentation::getInputSpecification(PlugInArgList*& pInArgList)
{
   VERIFY((pInArgList = Service<PlugInManagerServices>()->getPlugInArgList()) != NULL);
   pInArgList->addArg<Progress>(Executable::ProgressArg(), "Object for progress reporting.");
   pInArgList->addArg<PointCloudElement>(Executable::DataElementArg(), "The point cloud to process.");
   pInArgList->addArg<PointCloudView>(Executable::ViewArg(), NULL, "The view displaying the point cloud. If set, the displayed data will be set to classification.");
   return true;
}

bool LiDAR_roof_segmentation::getOutputSpecification(PlugInArgList*& pOutArgList)
{
  /* VERIFY((pOutArgList = Service<PlugInManagerServices>()->getPlugInArgList()) != NULL);
   pOutArgList->addArg<double>("Minimum", "The minimum value");
   pOutArgList->addArg<double>("Maximum", "The maximum value");
   pOutArgList->addArg<unsigned int>("Count", "The number of points in the point cloud");*/
   pOutArgList = NULL;
   return true;
}

bool LiDAR_roof_segmentation::execute(PlugInArgList* pInArgList, PlugInArgList* pOutArgList)
{

   if (pInArgList == NULL)
   {
      return false;
   }
  /*  
  ProgressTracker progress(pInArgList->getPlugInArgValue<Progress>(Executable::ProgressArg()),
      "Identifying buildings\n", "app", getDescriptorId());*/

  PointCloudElement* pElement = pInArgList->getPlugInArgValue<PointCloudElement>(Executable::DataElementArg());


   if (pElement == NULL)
   {
	  //progress.report("A valid point cloud element must be provided.", 0, ERRORS, true);
      return false;
   }
  
   showGui();

   //progress.report("ciao", 100, NORMAL);
   return true;
}

bool LiDAR_roof_segmentation::showGui()
{

	StepResource pStep( "Start", "app", "a520c744-1d6c-11e4-a3ec-b2227cce2b54" );
	
	Service<DesktopServices> pDesktop;

	mpGui = new Gui( pDesktop->getMainWidget(),"test", false);
	VERIFYNR(connect(mpGui, SIGNAL(finished(int)), this, SLOT(dialogClosed())));
	
	mpGui->show();
	pStep->finalize(Message::Success);
	return true;
}

void LiDAR_roof_segmentation::dialogClosed()
{
   abort();
}

  QWidget* LiDAR_roof_segmentation::getWidget() const
{
   return mpGui;
}

//bool LiDAR_roof_segmentation::serialize(SessionItemSerializer &serializer) const
//{
//   return serializer.serialize(NULL, 0); // force recreation on session load
//}
//
//bool LiDAR_roof_segmentation::deserialize(SessionItemDeserializer &deserializer)
//{
//   return showGui();
//}