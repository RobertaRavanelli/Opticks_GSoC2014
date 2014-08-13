/*
 * The information in this file is
 * Copyright(c) 2007 Ball Aerospace & Technologies Corporation
 * and is subject to the terms and conditions of the
 * GNU Lesser General Public License Version 2.1
 * The license text is available from   
 * http://www.gnu.org/licenses/lgpl.html
 */

#include "AppVerify.h"
#include "Gui.h"
#include <QtGui/QCheckBox>
#include <QtGui/QGridLayout>
#include <QtGui/QLabel>
#include <QtGui/QSpinBox>
#include <QtGui/qapplication.h>
#include <QtGui/QPushButton>
#include <Qt/qevent.h>
#include <QtGui/QMessageBox>
#include "Progress.h"
#include "ProgressResource.h"
#include "Interpolation.h"
#include "Segmentation.h"
#include "Ransac.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/core/eigen.hpp>


 Gui::Gui( QWidget* pParent, const char* pName, bool modal)
: QDialog(pParent)
{
	setModal( FALSE );
	setWindowTitle(QString::fromStdString("LiDAR roof extraction"));
	
	mpRunButton = new QPushButton( "Run the Plug-In", this );
	mpDEMspacing = new QDoubleSpinBox(this);
	mpRANSACthreshold = new QDoubleSpinBox(this);
	mpHorizontalTiles = new QDoubleSpinBox(this);
	mpVerticalTiles = new QDoubleSpinBox(this);
	mpLASListCombo = new QComboBox(this);

	mpLASListCombo -> setFixedSize(300,20);
	mpDEMspacing -> setFixedSize(50,20);
	mpHorizontalTiles -> setFixedSize(50,20);
	mpVerticalTiles -> setFixedSize(50,20);
    mpRANSACthreshold -> setFixedSize(50,20);

	QLabel *SelectLAS = new QLabel("Select the input .las file");
	QLabel *dem_spacing_unit = new QLabel("m/ft");//it depends from the reference system used in the file
	QLabel *SelectDEMspacing = new QLabel("Select the spacing for the DEM");
	QLabel *SelectTILEnumber = new QLabel("Select the number of tiles in which to divide the DEM raster");
	QLabel *xlabel = new QLabel("X");
	QLabel *SelectRANSACthreshold = new QLabel("Select the value of the RANSAC threshold");
	QLabel *RANSAC_threshold_unit = new QLabel("m/ft");//it depends from the reference system used in the file
	
	// Layout
    QGridLayout* pGrid = new QGridLayout(this);
	pGrid->addWidget(SelectLAS, 0, 0, 1, 6);
	pGrid->addWidget(mpLASListCombo, 1, 0, 1, 6);
	pGrid->addWidget(SelectDEMspacing, 2, 0, 1, 1);
	pGrid->addWidget(mpDEMspacing, 3, 0, 1, 1);
    pGrid->addWidget(dem_spacing_unit, 3, 1);
	pGrid->addWidget(SelectTILEnumber, 4, 0);
	pGrid->addWidget(mpHorizontalTiles, 5, 0, 1, 1);
	pGrid->addWidget(xlabel, 5, 1);
	pGrid->addWidget(mpVerticalTiles, 5, 2, 1, 1);
	pGrid->addWidget(SelectRANSACthreshold, 6, 0);
	pGrid->addWidget(mpRANSACthreshold, 7, 0);
	pGrid->addWidget(RANSAC_threshold_unit, 7, 1);
	pGrid->addWidget(mpRunButton, 8, 8);
	
	// Connections
	VERIFYNRV(connect(mpRunButton, SIGNAL( clicked() ), this, SLOT( RunApplication() )));

	init();
}


/*
*  Destroys the object and frees any allocated resources
*/
Gui::~Gui()
{
	/* Service<DesktopServices> pDesktop;
	 SpatialDataWindow* pScaledWindow = dynamic_cast<SpatialDataWindow*>( pDesktop->getWindow(
      "prova", SPATIAL_DATA_WINDOW ) );
   if ( pScaledWindow != NULL )
   {
      pDesktop->deleteWindow( pScaledWindow );
      pScaledWindow = NULL;
   }*/
}


void Gui::init()
{

   mPointCloudNames = pModel->getElementNames("PointCloudElement");
   
   int ii=0;
   for (unsigned int i = 0; i < mPointCloudNames.size(); i++)
   {
	  size_t pos;
	  std::string file_ext;      	  
	  pos = mPointCloudNames[i].find_last_of(".");
	  file_ext = mPointCloudNames[i].substr(pos);	  

	  if (file_ext.compare(".las") ==0) 
	  {
		  mpLASListCombo->insertItem(ii, QString::fromStdString(mPointCloudNames[i]));	
		  ii++;
	  }
   }
   button_cont = 0;

   mpDEMspacing ->setMinimum(1);
   mpDEMspacing ->setMaximum(10);
   mpDEMspacing ->setValue(5);


   mpRANSACthreshold ->setMinimum(0.1);
   mpRANSACthreshold ->setMaximum(5);
   mpRANSACthreshold ->setValue(1);

   mpHorizontalTiles ->setMinimum(1);
   mpHorizontalTiles ->setMaximum(20);
   mpHorizontalTiles ->setValue(8);

   mpVerticalTiles ->setMinimum(1);
   mpVerticalTiles ->setMaximum(20);
   mpVerticalTiles ->setValue(10);
}


void Gui::RunApplication()
{
	StepResource pStep( "LiDAR roof extraction", "app", "4c607288-2331-11e4-ae9f-b2227cce2b54" );
	ProgressResource pResource("ProgressBar");
	Progress *pProgress = pResource.get(); 
	pProgress-> setSettingAutoClose(true);
	
	button_cont++;
	mpRunButton->setEnabled(false);
	
	if (mPointCloudNames.empty() == true) // this is like to check if if (pElement == NULL)
    {
	   pProgress->updateProgress("A valid point cloud element must be provided.", 0, ERRORS);
	   pStep->finalize(Message::Abort, "A valid point cloud element must be provided.");
	   Gui::init();// needed to reload las files
	   mpRunButton->setEnabled(true);
    }
	else
	{
		std::string las_name = mPointCloudNames.at(mpLASListCombo->currentIndex());
		pElement = dynamic_cast<PointCloudElement*> (pModel->getElement(las_name, "", NULL ));
	
		Interpolation interp = Interpolation();
		Segmentation seg = Segmentation();
		float dem_spacing = mpDEMspacing->value();
		int n_rows_tiles = mpHorizontalTiles->value();
		int n_cols_tiles = mpVerticalTiles->value();
		Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> dem;
		dem = interp.generate_DEM( pElement, dem_spacing) ;
		double RANSAC_threshold =   mpRANSACthreshold->value();
	
		std::string path = "C:/Users/Roberta/Desktop/Results/";

		if (dem.size() != -1) // check if DEM  matrix is null
		{
		   interp.print_DEM_on_file( std::string(path) + "dem_" + StringUtilities::toDisplayString(button_cont) + "_from_gui.txt", dem);
		   draw_raster_from_eigen_mat ("dem "+ StringUtilities::toDisplayString(button_cont), dem, pElement);
		   cv::Mat CVdemRM(static_cast<int>(dem.rows()), static_cast<int>(dem.cols()), CV_32FC1, dem.data());
		   std::vector<cv::Mat> tiles = seg.n_x_m_tile_generator(CVdemRM, n_rows_tiles, n_cols_tiles);
		   //* ORIGINAL_TILES_MERGED is the real raster input for all the processing methods of the Plug-In,
		   //* since the way I use to tile the original raster makes lose some pixels
		   //* (however they are all on the last columns and/or last rows, so the original raster conformation is preserved - no problems for the image coordinates of the pixels)
		   //* for example, original_tiles_merged is needed for the mask application (method use to retrieve the z coordinate of every pixel identified as belonging to a building)
		   cv::Mat original_tiles_merged = seg.merge_tiles(tiles, n_rows_tiles, n_cols_tiles);
		   draw_raster_from_openCV_mat ("original tiles merged " + StringUtilities::toDisplayString(button_cont), original_tiles_merged, pElement);
		   cv::Mat result_watershed = seg.process_all_point_cloud_with_watershed(n_rows_tiles, n_cols_tiles);
		   cv::Mat buildings =  original_tiles_merged.mul(result_watershed);
		   draw_raster_from_openCV_mat ("buildings " + StringUtilities::toDisplayString(button_cont), buildings, pElement);
		   seg.connected_components(result_watershed);
		   seg.draw_buildings_contours(result_watershed);
		   seg.process_all_point_cloud_with_pca(n_rows_tiles, n_cols_tiles);
		   seg.Ransac_for_buildings(dem_spacing, RANSAC_threshold, original_tiles_merged);
		   seg.print_result();
		}
	
			// RANSAC test: it must return a=0, b=0, c=1 and d=-10: the plane Z=10
			Ransac Ransac_test = Ransac();
			Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> matrix;
			matrix.setConstant(10000, 3, 0.0);
			for (int i = 0; i< 9000; i++)
			{
				 matrix(i,0) = i*i;// % 100;
				 srand (time(NULL));
				 matrix(i,1) = 5*i+21;// rand() % 33;
				 matrix(i,2) = 10.0;
			}
			for (int i = 9000; i< 10000; i++)
			{
				 matrix(i,0) = rand() % 10;
				 matrix(i,1) = rand() % 200;
				 matrix(i,2) = rand() % 700;
			}
			Ransac_test.ComputeModel(matrix, RANSAC_threshold);
			std::string ciao = Ransac_test.ransac_msg;
	
		mpRunButton->setEnabled(true);
		pStep->finalize(Message::Success);
	}
}

bool Gui::draw_raster_from_eigen_mat (std::string name, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> eigen_matrix, PointCloudElement* pElement)
{
	StepResource pStep("Drawing DEM raster", "app", "8d9c42c6-1e5a-11e4-b4be-b2227cce2b54");
	ProgressResource pResource("ProgressBar");
	Progress *pProgress = pResource.get(); 
	//pProgress-> setSettingAutoClose(false);
	pProgress-> setSettingAutoClose(true);

	const float badVal = -9999.f;
	
	RasterElement* pDemOut = RasterUtilities::createRasterElement(name,  static_cast<int>(eigen_matrix.rows()), static_cast<int>(eigen_matrix.cols()), FLT4BYTES, true, pElement);
	if (pDemOut == NULL)
    {
		   warning_msg += "Unable to create DEM raster ("+ name +").\n";
		   pProgress->updateProgress("Unable to create DEM raster ("+ name +").", 0, ERRORS);
		   pStep->finalize(Message::Abort, "Unable to create DEM raster ("+ name +").");
		   return false;
    }
	   pDemOut->getStatistics()->setBadValues(std::vector<int>(1, (int)badVal));
	   FactoryResource<DataRequest> pReq;
	   pReq->setWritable(true);
	   DataAccessor racc(pDemOut->getDataAccessor(pReq.release()));
	
	   for (int row = 0; row < eigen_matrix.rows(); row++)
	   {
			for (int col = 0; col < eigen_matrix.cols(); col++)
			{
			if (!racc.isValid())
			{
				warning_msg += "Error writing output raster(" + name + ").";
				pProgress->updateProgress("Error writing output raster(" + name + ").", 0, ERRORS);
		        pStep->finalize(Message::Abort, "Error writing output raster(" + name + ").");
				return false;
			}
			pProgress->updateProgress("Drawing DEM raster ("+ name +")", row * 100 / eigen_matrix.rows(), NORMAL);
			*reinterpret_cast<float*>(racc->getColumn()) = eigen_matrix(row, col);
			racc->nextColumn();
			}
			racc->nextRow();
	   }
	   pDemOut->updateData();
	 
	   SpatialDataView* pView = ((SpatialDataWindow*)Service<DesktopServices>()->createWindow(name, SPATIAL_DATA_WINDOW))->getSpatialDataView();
       pView->setPrimaryRasterElement(pDemOut);
       {
         UndoLock lock(pView);
         pView->createLayer(RASTER, pDemOut);
	   }
	   pProgress->updateProgress("DEM raster is drawn.", 100, NORMAL);
	   pStep->finalize();

	return true;
}

bool Gui::draw_raster_from_openCV_mat (std::string name, cv::Mat image, PointCloudElement* pElement)
{
	image.convertTo(image, CV_32FC1);
	Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> image_Eigen(image.ptr<float>(), image.rows, image.cols);
	Gui::draw_raster_from_eigen_mat (name, image_Eigen, pElement);
	return true;
}