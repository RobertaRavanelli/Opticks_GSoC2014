/*
 * The information in this file is
 * Copyright(c) 2007 Ball Aerospace & Technologies Corporation
 * and is subject to the terms and conditions of the
 * GNU Lesser General Public License Version 2.1
 * The license text is available from   
 * http://www.gnu.org/licenses/lgpl.html
 */

#ifndef GUI_H
#define GUI_H

#include "ViewerShell.h"
#include <QtGui/QWidget>
#include "AttachmentPtr.h"
#include <QtGui/QAction>
#include <QtGui/QDialog>
#include <QtGui/QGroupBox>
#include <QtGui/QRadioButton>
#include <QtGui/QComboBox>
#include <QtGui/QDoubleSpinBox>

#include "PointCloudAccessor.h"
#include "PointCloudAccessorImpl.h"
#include "PointCloudDataDescriptor.h"
#include "PointCloudDataRequest.h"
#include "PointCloudElement.h"
#include "PointCloudView.h"

#include <Eigen/Core> //http://eigen.tuxfamily.org/index.php?title=Visual_Studio
#include <Eigen/Eigenvalues>
#include "StringUtilities.h"

class Gui: public QDialog//,public ViewerShell 
{
	Q_OBJECT

public:
	
	Gui(QWidget* pParent = 0, const char* pName = 0, bool modal = FALSE);
	virtual ~Gui();
	std::vector<std::string> mPointCloudNames;

public slots:

	void RunApplication();

private:

    Gui(const Gui& rhs);
    Gui& operator=(const Gui& rhs);
    QPushButton* mpRunButton;
    QComboBox* mpLASListCombo;
	QDoubleSpinBox* mpDEMspacing;
	QDoubleSpinBox* mpRANSACthreshold;
	QDoubleSpinBox* mpHorizontalTiles;
	QDoubleSpinBox* mpVerticalTiles;
	Service<ModelServices> pModel;
    PointCloudElement* pElement;
	void init();
	bool Gui::draw_raster_from_eigen_mat (std::string name, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> median_Eigen, PointCloudElement* pElement);
	std::string warning_msg;
	int button_cont;
};
#endif