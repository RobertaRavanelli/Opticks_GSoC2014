#pragma once

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
#include <boost/random/uniform_int.hpp>
#include <boost/random/random_device.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#include <Eigen/Core> //http://eigen.tuxfamily.org/index.php?title=Visual_Studio
#include <Eigen/Eigenvalues>
#include "StringUtilities.h"


// these methods are needed for eigen values
template<typename Matrix, typename Roots> inline void
computeRoots (const Matrix& m, Roots& roots);
template<typename Scalar, typename Roots> void
computeRoots2 (const Scalar& b, const Scalar& c, Roots& roots);
template<typename Matrix, typename Vector> inline void
eigen33 ( Matrix& mat, typename Matrix::Scalar& eigenvalue, Vector& eigenvector);

class Ransac
{
public:
	
	Eigen::VectorXd  model_coefficients;
	 
	std::vector<int> random_selected_indices;
    const PointCloudDataDescriptor* pDesc;
	std::vector<int> inliers; // it contains the inliers indexes (their ID) for the single iterations
	int nr_p;//NUMBER OF THE INLIERS for the single iterations
	Eigen::VectorXd optimized_coefficients;

	std::string msg2;

	Ransac(void);
	~Ransac(void);
	bool Ransac::ComputeModel(PointCloudElement* pElement);
	bool Ransac::getSamples (int model_points);
	bool Ransac::computeModelCoefficients (PointCloudAccessor acc);
	bool Ransac::countWithinDistance(double threshold, PointCloudAccessor acc);
	bool Ransac::optimizeModelCoefficients(PointCloudAccessor acc);
	bool Ransac::computeRootsdouble (const Eigen::Matrix3d m, Eigen::Vector3d roots);
	bool Ransac::computeRoots2double (double b, double c, Eigen::Vector3d roots);
	
	

};

