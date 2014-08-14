/*
 * The information in this file is
 * Copyright(c) 2007 Ball Aerospace & Technologies Corporation
 * and is subject to the terms and conditions of the
 * GNU Lesser General Public License Version 2.1
 * The license text is available from   
 * http://www.gnu.org/licenses/lgpl.html
 
 * derived from PCL http://pointclouds.org/
	 * Software License Agreement (BSD License)
	 *
	 *  Point Cloud Library (PCL) - www.pointclouds.org
	 *  Copyright (c) 2009, Willow Garage, Inc.
	 *  Copyright (c) 2012-, Open Perception, Inc.
	 *
	 *  All rights reserved.
	 *
	 *  Redistribution and use in source and binary forms, with or without
	 *  modification, are permitted provided that the following conditions
	 *  are met:
	 *
	 *   * Redistributions of source code must retain the above copyright
	 *     notice, this list of conditions and the following disclaimer.
	 *   * Redistributions in binary form must reproduce the above
	 *     copyright notice, this list of conditions and the following
	 *     disclaimer in the documentation and/or other materials provided
	 *     with the distribution.
	 *   * Neither the name of the copyright holder(s) nor the names of its
	 *     contributors may be used to endorse or promote products derived
	 *     from this software without specific prior written permission.
	 *
	 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	 *  POSSIBILITY OF SUCH DAMAGE.
	 *
	 *
 */

#pragma once

#include "PointCloudAccessor.h"
#include "PointCloudAccessorImpl.h"
#include "PointCloudDataDescriptor.h"
#include "PointCloudDataRequest.h"
#include "PointCloudElement.h"
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
	
	Ransac::Ransac(std::string path_for_result);
	Ransac::Ransac(void);
	Ransac::~Ransac(void);

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

