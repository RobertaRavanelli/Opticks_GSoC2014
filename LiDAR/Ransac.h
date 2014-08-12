/*
 * The information in this file is
 * Copyright(c) 2007 Ball Aerospace & Technologies Corporation
 * and is subject to the terms and conditions of the
 * GNU Lesser General Public License Version 2.1
 * The license text is available from   
 * http://www.gnu.org/licenses/lgpl.html
 */

#pragma once

#include "PointCloudAccessor.h"
#include "PointCloudAccessorImpl.h"
#include "PointCloudDataDescriptor.h"
#include "PointCloudDataRequest.h"
#include "PointCloudElement.h"
#include "PointCloudView.h"
#include <Eigen/Core> //http://eigen.tuxfamily.org/index.php?title=Visual_Studio


	// these methods are needed for eigen values
	template<typename Matrix, typename Roots> inline void
	computeRoots (const Matrix& m, Roots& roots);
	template<typename Scalar, typename Roots> void
	computeRoots2 (const Scalar& b, const Scalar& c, Roots& roots);
	template<typename Matrix, typename Vector> inline void
	eigen33 ( Matrix& mat, typename Matrix::Scalar& eigenvalue, Vector& eigenvector);


class Ransac
{
private:
	std::vector<int> inliers; // it contains the inliers indexes (their ID) for the single iteration: maybe this must be private
	std::vector<int> outliers; // it contains the outliers indexes (their ID) for the single iteration: maybe this must be private
	int nr_p;//NUMBER OF THE INLIERS for the single iteration
	int nr_o;//NUMBER OF THE outliers for the single iteration
	Eigen::VectorXd  model_coefficients;
	std::vector<int> random_selected_indices;
	Eigen::VectorXd optimized_coefficients;

public:
	std::string path;
	std::string ransac_msg;
    const PointCloudDataDescriptor* pDesc;
	std::vector<int> final_inliers;// the inliers found after ALL the iterations
	std::vector<int> final_outliers;// the outiers found after ALL the iterations
	Eigen::VectorXd final_model_coefficients; // the coefficients corrispondent to the max number of inliers
	int n_best_inliers_count;

	//* buildingS is a std::vector of Eigen matrixes
	//* each row of this std::vector is an Eigen matrix with rows = number of points belonging to the specific building and columns = 3 (x,y,z coordinates)
	//* first row stores the x,y and z coordinates of all the points belonging to first identified building
	//* second row stores the x,y and z coordinates of all the points belonging to second identified building...
	std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> buildingS;

	std::vector<std::vector<int>> buildingS_inliers;// stores the inliers for all the buildings (only first iteration - iteration number 0)
	std::vector<std::vector<int>> buildingS_outliers;// stores the outliers for all the buildings (only first iteration - iteration number 0)
	std::vector<Eigen::VectorXd> buildingS_plane_coefficients;// stores the plane parameters for all the buildings (only first iteration - iteration number 0)
	std::vector<int> buldingS_number_inliers; // every row stores the number of inliers for each building (only first iteration - iteration number 0)

	Ransac(void);
	~Ransac(void);

	// these methods are needed to apply the RANSAC algorhitm directly to the .las file (PointCloudElement)
	bool Ransac::ComputeModel(PointCloudElement* pElement, double ransac_threshold);
	bool Ransac::getSamples (int model_points);
	bool Ransac::computeModelCoefficients (PointCloudAccessor acc);
	bool Ransac::countWithinDistance(double threshold, PointCloudAccessor acc);
	bool Ransac::optimizeModelCoefficients(PointCloudAccessor acc);

    // these methods BE APPLIED TO AN EIGEN MATRIX(AND NOT THE PELEMENT)
	bool Ransac::ComputeModel(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> data, double ransac_threshold);
	bool Ransac::getSamples(int model_points,int size_array);
	bool Ransac::computeModelCoefficients( Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> data);
	bool Ransac::countWithinDistance(double threshold,  Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> data);
	bool Ransac::optimizeModelCoefficients(Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> data);
};

