#include "Ransac.h"



Ransac::Ransac(void)
{
}


Ransac::~Ransac(void)
{
}


bool Ransac::ComputeModel(PointCloudElement* pElement)
{
	double probability_= 0.8;            // probability to pick a sample with 0 outliers
	int max_iterations_ = 10000;//100000;  // safeguard against being stuck in this loop forever
	nr_p = 0;

	if (pElement == NULL)
	 { 
        msg2 += "A valid point cloud element must be provided. \n\n";		 
		return false;
	 }

	 pDesc = static_cast<const PointCloudDataDescriptor*>(pElement->getDataDescriptor());
	 FactoryResource<PointCloudDataRequest> req;
     req->setWritable(true);
	 PointCloudAccessor acc(pElement->getPointCloudAccessor(req.release()));

	 int iterations_ = 0;
     int n_best_inliers_count = -INT_MAX;

	 double k = 1.0;

     std::vector<int> selection;
     Eigen::VectorXd final_model_coefficients;// the coefficients corriispondent to the max number of inliers
	
     double log_probability  = log (1.0 - probability_);
     double one_over_indices = 1.0 / static_cast<double> (pDesc->getPointCount());
	
	int n_inliers_count = 0;
   
	
	// Iterate
    while (iterations_ < k && iterations_ <  max_iterations_)
    {
		//msg2 += "It "+ StringUtilities::toDisplayString(iterations_)+'\n';
		
		// Get the 3 samples which satisfy the plane model criteria
		getSamples(3);
		if (getSamples(3)==true)
		{
			computeModelCoefficients(acc);

			// Select the inliers that are within threshold_ from the model
			//countWithinDistance(0.00000001, acc);
			countWithinDistance(0.2, acc);
			if (nr_p > n_best_inliers_count)
			{
				n_best_inliers_count = nr_p;
				final_model_coefficients = model_coefficients;
			}
			nr_p=0;
			//// Compute the k parameter (k=log(z)/log(1-w^n))
			double w = static_cast<double> (n_best_inliers_count) * one_over_indices;
			//double p_no_outliers = 1.0 - pow (w, static_cast<double> (selection.size ()));
			double p_no_outliers = 1.0 - pow (w, static_cast<double> (n_best_inliers_count));
			p_no_outliers = (std::max) (std::numeric_limits<double>::epsilon (), p_no_outliers);         // Avoid division by -Inf
			p_no_outliers = (std::min) (1.0 - std::numeric_limits<double>::epsilon (), p_no_outliers);   // Avoid division by 0.
			k = log_probability / log (p_no_outliers);
		}
		 ++iterations_;
	}
	 
	 msg2 += "iterations "+StringUtilities::toDisplayString(iterations_)+"\n\n";
	 msg2 += "model coefficients "+StringUtilities::toDisplayString(final_model_coefficients[0])+'\n'+ StringUtilities::toDisplayString(final_model_coefficients[1])+'\n'+StringUtilities::toDisplayString(final_model_coefficients[2])+'\n'+StringUtilities::toDisplayString(final_model_coefficients[3])+'\n'+'\n';// verifica
	 return true;
}

bool Ransac::getSamples (int  model_points)
{
	   int while_counter = 0;

	   random_selected_indices.resize(model_points);
	   int point_index = -1;
	   //http://www.boost.org/doc/libs/1_55_0/doc/html/boost_random/tutorial.html#boost_random.tutorial.generating_a_random_password.c5
	   boost::random::random_device rng;
	   boost::random::uniform_int_distribution<> index_dist(0, pDesc->getPointCount() - 1);
  
	   for (size_t i = 0; i < model_points; ++i)
	   {
		   //a check not to select the same points
		   while(point_index == index_dist(rng))
		   {
			   while_counter ++;
			   if (while_counter > 10000)
			   {
			       msg2 += "Not able to find 3 different points \n\n";
				   return false;
			   }
		   }
		  point_index = index_dist(rng);
		  random_selected_indices[i] = point_index;
	   }
	   
	   //msg2 = "Selected points \n"+StringUtilities::toDisplayString(random_selected_indices[0])+'\n'+ StringUtilities::toDisplayString(random_selected_indices[1])+'\n'+StringUtilities::toDisplayString(random_selected_indices[2])+'\n'+'\n';// verifica
	   return true;
}

bool Ransac::computeModelCoefficients (PointCloudAccessor acc)
{
		/////////////////////      RETRIEVE DATA FROM POINTCLOUDS       ///////////////////
		Eigen::Array4d p0, p1, p2;//store for the 3 random selected points
		
		acc->toIndex(random_selected_indices[0]);
		p0[0] = acc->getXAsDouble(true);
		p0[1] = acc->getYAsDouble(true);
		p0[2] = acc->getZAsDouble(true);

		acc->toIndex(random_selected_indices[1]);
		p1[0] = acc->getXAsDouble(true);
		p1[1] = acc->getYAsDouble(true);
		p1[2] = acc->getZAsDouble(true);

		acc->toIndex(random_selected_indices[2]);
		p2[0] = acc->getXAsDouble(true);
		p2[1] = acc->getYAsDouble(true);
		p2[2] = acc->getZAsDouble(true);

		acc->toIndex(0);
	
		/////////////////// FIND PLANE COEFFICIENTS ///////////////////////////////////////
  
		/*http://www.cplusplus.com/forum/general/74720/
		normal = cross_product(b-a, c-a) = cross_product(p1-p0, p2-p0)
		distance = dot_product(normal, P): P is a point in the plane, like a, b or c */
  
		//  Compute the segment values (in 3d) between p1 and p0
		Eigen::Array4d p1p0 = p1 - p0;

		// Compute the segment values (in 3d) between p2 and p0
		Eigen::Array4d p2p0 = p2 - p0;

		// dy1dy2: useful to check for collinearity 
		Eigen::Array4d dy1dy2 = (p1-p0) / (p2-p0);

		if ( (dy1dy2[0] == dy1dy2[1]) && (dy1dy2[2] == dy1dy2[1])  )  // Check for collinearity
		{ 
			msg2 += "The selected points are collinear: repeat the selection\n\n";
			return false;
		}
  
		// Compute the plane coefficients from the 3 given points in a straightforward manner
		// calculate the plane normal n = (p2-p1) x (p3-p1) = cross (p2-p1, p3-p1)
		model_coefficients.resize (4);
		model_coefficients[0] = p1p0[1] * p2p0[2] - p1p0[2] * p2p0[1];
		model_coefficients[1] = p1p0[2] * p2p0[0] - p1p0[0] * p2p0[2];
		model_coefficients[2] = p1p0[0] * p2p0[1] - p1p0[1] * p2p0[0];
		model_coefficients[3] = 0;

		// Normalize
		model_coefficients.normalize();

		// ax + by + cz + d = 0
		model_coefficients[3] = -1 * (model_coefficients.template head<4>().dot (p0.matrix ()));

		//msg2 += "Approximated plane parameters \n"+StringUtilities::toDisplayString(model_coefficients[0])+'\n'+ StringUtilities::toDisplayString(model_coefficients[1])+'\n'+StringUtilities::toDisplayString(model_coefficients[2])+'\n'+StringUtilities::toDisplayString(model_coefficients[3])+'\n'+'\n';// verifica
		return true;
}


bool Ransac::countWithinDistance(double threshold,PointCloudAccessor acc)
{
	  std::vector<double> distances; 
	  std::vector<double> error_sqr_dists_;
	 
	  inliers.resize (pDesc->getPointCount());
	  distances.resize(pDesc->getPointCount());
	  error_sqr_dists_.resize(pDesc->getPointCount());
	  double sum_distances = 0;
	  double mean_distances = 0;
     // msg2 += "inliers \n";
	  // Iterate through the 3d points and calculate the distances from them to the plane
	  for (size_t i = 0; i < pDesc->getPointCount(); ++i)
	  {
		// Calculate the distance from the point to the plane normal as the dot product
		// D = (P-A).N/|N|
	   Eigen::Vector4d pt (acc->getXAsDouble(true),
						   acc->getYAsDouble(true),
						   acc->getZAsDouble(true),
							1);
		distances[i] = fabs (model_coefficients.dot (pt));
		sum_distances += distances[i];
	
		acc->getPointId();
		if (distances[i] < threshold)//I verified that with a treshold = 0.00000001 (approximation for 0), the method finds only 3 inliers: the 3 points with whom we build the plane
		{
		  // Returns the indices of the points whose distances are smaller than the threshold
		  inliers[nr_p] =i;
		  error_sqr_dists_[nr_p] =  static_cast<double> (distances[i]);// così dava problemi static_cast<double> (distances[i]);
		  ++nr_p;
		  //msg2 += StringUtilities::toDisplayString(i)+' ';
		}
		acc->nextValidPoint();
	  }
	  acc->toIndex(0);
	  mean_distances = sum_distances / static_cast<double>(pDesc->getPointCount());
	 // msg2 += "\n"+StringUtilities::toDisplayString(nr_p)+" inliers found on"+StringUtilities::toDisplayString(pDesc->getPointCount())+" total points\n";
	return true;
}


bool Ransac::optimizeModelCoefficients(PointCloudAccessor acc)
{
	acc->toIndex(0);
	// Eigen align dispone le matrici su un array e poi vi si accede con () ??????
	EIGEN_ALIGN16 Eigen::Matrix3d covariance_matrix;
    Eigen::Vector4d xyz_centroid;// centroid: the mean vector  
	
	Eigen::Matrix<double, 1, 9, Eigen::RowMajor> sum = Eigen::Matrix<double, 1, 9, Eigen::RowMajor>::Zero (); // matrix used to compute covariance matrix see centroid.hpp (common)

	 //I must use only the inliers; 
	  for (size_t i=0; i<nr_p; i++)
	  {
		  //acc->toIndex(inliers[i]);
		  acc->toIndex(inliers[i]);
		  sum [0] += acc->getXAsDouble(true) * acc->getXAsDouble(true);
		  sum [1] += acc->getXAsDouble(true) * acc->getYAsDouble(true);
		  sum [2] += acc->getXAsDouble(true) * acc->getZAsDouble(true);
		  sum [3] += acc->getYAsDouble(true) * acc->getYAsDouble(true);
		  sum [4] += acc->getYAsDouble(true) * acc->getZAsDouble(true);
		  sum [5] += acc->getZAsDouble(true) * acc->getZAsDouble(true);
		  sum [6] += acc->getXAsDouble(true);
		  sum [7] += acc->getYAsDouble(true);
		  sum [8] += acc->getZAsDouble(true);
	 }
  
   sum  /= static_cast<double> (pDesc->getPointCount());

   // see centroid.hpp (common)
    xyz_centroid[0] = sum [6]; // mean of the X values 
	xyz_centroid[1] = sum [7]; // mean of the Y values
	xyz_centroid[2] = sum [8]; // mean of the Z values
    xyz_centroid[3] = 0;       // unused
    covariance_matrix.coeffRef (0) = sum  [0] - sum [6] * sum [6]; //   variance_X  = sum (X^2)/n - (mean(X))^2
    covariance_matrix.coeffRef (1) = sum  [1] - sum [6] * sum [7]; // covariance_XY = sum (X*Y)/n - (mean(X) * mean(Y))
    covariance_matrix.coeffRef (2) = sum  [2] - sum [6] * sum [8]; // covariance_XZ = sum (X*Z)/n - (mean(X) * mean(Z))
    covariance_matrix.coeffRef (4) = sum  [3] - sum [7] * sum [7]; //   variance_Y  = sum (Y^2)/n - (mean(Y))^2
    covariance_matrix.coeffRef (5) = sum  [4] - sum [7] * sum [8]; // covariance_YZ = sum (Y*Z)/n - (mean(Y) * mean(Z))
    covariance_matrix.coeffRef (8) = sum  [5] - sum [8] * sum [8]; //   variance_Z  = sum (Z^2)/n - (mean(Z))^2
    covariance_matrix.coeffRef (3) = covariance_matrix.coeff (1);  //  covariance_YX = sum (X*Y)/n - (mean(X) * mean(Y)) (the covariance matrix is symmetric)
    covariance_matrix.coeffRef (6) = covariance_matrix.coeff (2);  // covariance_ZX = sum (X*Z)/n - (mean(X) * mean(Z))
    covariance_matrix.coeffRef (7) = covariance_matrix.coeff (5);   

	// Compute the model coefficients
    EIGEN_ALIGN16 Eigen::Vector3d::Scalar eigen_value;
    EIGEN_ALIGN16 Eigen::Vector3d eigen_vector;
	eigen33 (covariance_matrix, eigen_value, eigen_vector); //now it compiless, before it gave error LNK 2019, LNK 1120 eigen33: common/eigen.h: non funziona, provo a copiarla di sotto

	//msg2 += "eigen vector: BEFORE COMPUTATION \n"+StringUtilities::toDisplayString(eigen_vector[0])+'\n'+ StringUtilities::toDisplayString(eigen_vector[1])+'\n'+StringUtilities::toDisplayString(eigen_vector[2])+'\n'+'\n';// verifica

	///////////////////////////////////////      PCL::EIGEN33  Pcl/common/eigen.h        /////////////////////////////////////////////////
	//
	//double scale = covariance_matrix.cwiseAbs ().maxCoeff ();
 //   if (scale <= std::numeric_limits<double>::min ())
	//{ scale = double (1.0);}

	//
	//EIGEN_ALIGN16 Eigen::Matrix3d scaledMat = covariance_matrix / scale;
	//EIGEN_ALIGN16 Eigen::Vector3d eigenvalues; 

	///*Eigen::Matrix3d scaledMat = covariance_matrix / scale;
	//Eigen::Vector3d eigenvalues; */

	//// computeRoots (scaledMat, eigenvalues); // error LNK 2019, LNK 1120 eigen33: common/eigen.h: non funziona, provo a copiarla di sotto
 // 
	//                /////////////////      PCL::COMPUTE ROOTS  PCL::common/eigen.h        ///////////////////
	//				// The characteristic equation is x^3 - c2*x^2 + c1*x - c0 = 0.  The
	//				// eigenvalues are the roots to this equation, all guaranteed to be
	//				// real-valued, because the matrix is symmetric.
	//				double c0 =            scaledMat (0, 0) * scaledMat (1, 1) * scaledMat (2, 2)
	//						+ double (2) * scaledMat (0, 1) * scaledMat (0, 2) * scaledMat (1, 2)
	//									 - scaledMat (0, 0) * scaledMat (1, 2) * scaledMat (1, 2)
	//									 - scaledMat (1, 1) * scaledMat (0, 2) * scaledMat (0, 2)
	//									 - scaledMat (2, 2) * scaledMat (0, 1) * scaledMat (0, 1);
	//				double c1 = scaledMat (0, 0) * scaledMat (1, 1) -
	//							scaledMat (0, 1) * scaledMat (0, 1) +
	//							scaledMat (0, 0) * scaledMat (2, 2) -
	//							scaledMat (0, 2) * scaledMat (0, 2) +
	//							scaledMat (1, 1) * scaledMat (2, 2) -
	//							scaledMat (1, 2) * scaledMat (1, 2);
	//				double c2 = scaledMat (0, 0) + scaledMat (1, 1) + scaledMat (2, 2);

	//				if (fabs (c0) < Eigen::NumTraits<double>::epsilon ())// one root is 0 -> quadratic equation
	//				{ 
	//					eigenvalues (0) = double (0);
	//					double b= c2;
	//					double c= c1;
	//					double d = double (b * b - 4.0 * c);
	//					if (d < 0.0) // no real roots!!!! THIS SHOULD NOT HAPPEN!
	//					  d = 0.0;

	//					double sd = ::std::sqrt (d);

	//					eigenvalues (2) = 0.5f * (b + sd);
	//					eigenvalues (1) = 0.5f * (b - sd);
	//				}
	//				else
	//				{
	//				  const double s_inv3 = double (1.0 / 3.0);
	//				  const double s_sqrt3 = std::sqrt (double (3.0));
	//				  // Construct the parameters used in classifying the roots of the equation
	//				  // and in solving the equation for the roots in closed form.
	//				  double c2_over_3 = c2*s_inv3;
	//				  double a_over_3 = (c1 - c2 * c2_over_3) * s_inv3;
	//				  if (a_over_3 > double (0))
	//					a_over_3 = double (0);

	//				  double half_b = double (0.5) * (c0 + c2_over_3 * (double (2) * c2_over_3 * c2_over_3 - c1));

	//				  double q = half_b * half_b + a_over_3 * a_over_3*a_over_3;
	//				  if (q > double (0))
	//					q = double (0);

	//				  // Compute the eigenvalues by solving for the roots of the polynomial.
	//				  double rho = std::sqrt (-a_over_3);
	//				  double theta = std::atan2 (std::sqrt (-q), half_b) * s_inv3;
	//				  double cos_theta = std::cos (theta);
	//				  double sin_theta = std::sin (theta);
	//				  eigenvalues (0) = c2_over_3 + double (2) * rho * cos_theta;
	//				  eigenvalues (1) = c2_over_3 - rho * (cos_theta + s_sqrt3 * sin_theta);
	//				  eigenvalues (2) = c2_over_3 - rho * (cos_theta - s_sqrt3 * sin_theta);

	//				  // Sort in increasing order.
	//				  if (eigenvalues (0) >= eigenvalues (1))
	//					std::swap (eigenvalues (0), eigenvalues (1));
	//				  if (eigenvalues (1) >= eigenvalues (2))
	//				  {
	//					std::swap (eigenvalues (1), eigenvalues (2));
	//					if (eigenvalues(0) >=eigenvalues (1))
	//					  std::swap (eigenvalues (0), eigenvalues (1));
	//				  }

	//				  if (eigenvalues (0) <= 0) // eigenval for symetric positive semi-definite matrix can not be negative! Set it to 0
	//				  {//computeRoots2double(c2, c1, roots);
	//				  //computeRoots2double(c2, c1,roots);
	//						eigenvalues (0) = double (0);
	//						double b= c2;
	//						double c= c1;
	//						double d = double (b * b - 4.0 * c);
	//						if (d < 0.0) // no real roots!!!! THIS SHOULD NOT HAPPEN!
	//						  d = 0.0;

	//						double sd = ::std::sqrt (d);

	//						eigenvalues (2) = 0.5f * (b + sd);
	//						eigenvalues (1) = 0.5f * (b - sd);
	//				  }
	//				}
	//				///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////eigen_value (0) = eigenvalues (0) * scale;
	//eigen_value  = eigenvalues (0) * scale;

 //   scaledMat.diagonal ().array () -= eigenvalues (0);

	////msg2 += "\neigen value \n"+StringUtilities::toDisplayString(eigen_value (0))+'\n'+'\n';// verifica
	//msg2 += "\neigen value \n"+StringUtilities::toDisplayString(eigen_value)+'\n'+'\n';// verifica

 //   EIGEN_ALIGN16 Eigen::Vector3d vec1 = scaledMat.row (0).cross (scaledMat.row (1));
 //   EIGEN_ALIGN16 Eigen::Vector3d vec2 = scaledMat.row (0).cross (scaledMat.row (2));
 //   EIGEN_ALIGN16 Eigen::Vector3d vec3 = scaledMat.row (1).cross (scaledMat.row (2));

 //   double len1 = vec1.squaredNorm ();
 //   double len2 = vec2.squaredNorm ();
 //   double len3 = vec3.squaredNorm ();

 //   if (len1 >= len2 && len1 >= len3)
 //     eigen_vector = vec1 / std::sqrt (len1);
 //   else if (len2 >= len1 && len2 >= len3)
 //     eigen_vector = vec2 / std::sqrt (len2);
 //   else
 //     eigen_vector = vec3 / std::sqrt (len3);

   msg2 += "\neigen value \n"+StringUtilities::toDisplayString(eigen_value)+'\n'+'\n';// verifica
   msg2 += "eigen vector: normal of the plane \n"+StringUtilities::toDisplayString(eigen_vector[0])+'\n'+ StringUtilities::toDisplayString(eigen_vector[1])+'\n'+StringUtilities::toDisplayString(eigen_vector[2])+'\n'+'\n';// verifica

 //  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Hessian form (D = nc . p_plane (centroid here) + p)
  // The eigenvector (we must use the smallest) values are a, b and c coefficients and then we use the mean of x, y and z to define a point on that plane in order to solve for d: http://xiang-jun.blogspot.it/2009/08/fit-least-squares-plane-to-set-of.html
  Eigen::VectorXd optimized_coefficients;

  optimized_coefficients.resize (4);
  optimized_coefficients[0] = eigen_vector [0];
  optimized_coefficients[1] = eigen_vector [1];
  optimized_coefficients[2] = eigen_vector [2];
  optimized_coefficients[3] = 0;
  optimized_coefficients[3] = -1 * optimized_coefficients.dot (xyz_centroid);

  msg2 += "Optimized plane parameters \n"+StringUtilities::toDisplayString(optimized_coefficients[0])+'\n'+ StringUtilities::toDisplayString(optimized_coefficients[1])+'\n'+StringUtilities::toDisplayString(optimized_coefficients[2])+'\n'+StringUtilities::toDisplayString(optimized_coefficients[3])+'\n';// verifica

	 return true;
}

 /** determines the eigenvector and eigenvalue of the smallest eigenvalue of the symmetric positive semi definite input matrix
    * \param[in] mat symmetric positive semi definite input matrix
    * \param[out] eigenvalue smallest eigenvalue of the input matrix
    * \param[out] eigenvector the corresponding eigenvector for the input eigenvalue
    * \note if the smallest eigenvalue is not unique, this function may return any eigenvector that is consistent to the eigenvalue.
    */
  template<typename Matrix, typename Vector> inline void
  eigen33 ( Matrix& mat, typename Matrix::Scalar& eigenvalue, Vector& eigenvector)
  {
    typedef typename Matrix::Scalar Scalar;
    // Scale the matrix so its entries are in [-1,1].  The scaling is applied
    // only when at least one matrix entry has magnitude larger than 1.

    Scalar scale = mat.cwiseAbs ().maxCoeff ();
    if (scale <= std::numeric_limits<Scalar>::min ())
      scale = Scalar (1.0);

    Matrix scaledMat = mat / scale;

    Vector eigenvalues;
    computeRoots (scaledMat, eigenvalues);

    eigenvalue = eigenvalues (0) * scale;

    scaledMat.diagonal ().array () -= eigenvalues (0);

    Vector vec1 = scaledMat.row (0).cross (scaledMat.row (1));
    Vector vec2 = scaledMat.row (0).cross (scaledMat.row (2));
    Vector vec3 = scaledMat.row (1).cross (scaledMat.row (2));

    Scalar len1 = vec1.squaredNorm ();
    Scalar len2 = vec2.squaredNorm ();
    Scalar len3 = vec3.squaredNorm ();

    if (len1 >= len2 && len1 >= len3)
      eigenvector = vec1 / std::sqrt (len1);
    else if (len2 >= len1 && len2 >= len3)
      eigenvector = vec2 / std::sqrt (len2);
    else
      eigenvector = vec3 / std::sqrt (len3);
  }

//*  computes the roots of the characteristic polynomial of the input matrix m, which are the eigenvalues
//    * \param[in] m input matrix
//    * \param[out] roots roots of the characteristic polynomial of the input matrix m, which are the eigenvalues
//    */
  template<typename Matrix, typename Roots> inline void
  computeRoots (const Matrix& m, Roots& roots)
  {
    typedef typename Matrix::Scalar Scalar;

    // The characteristic equation is x^3 - c2*x^2 + c1*x - c0 = 0.  The
    // eigenvalues are the roots to this equation, all guaranteed to be
    // real-valued, because the matrix is symmetric.
    Scalar c0 =            m (0, 0) * m (1, 1) * m (2, 2)
            + Scalar (2) * m (0, 1) * m (0, 2) * m (1, 2)
                         - m (0, 0) * m (1, 2) * m (1, 2)
                         - m (1, 1) * m (0, 2) * m (0, 2)
                         - m (2, 2) * m (0, 1) * m (0, 1);
    Scalar c1 = m (0, 0) * m (1, 1) -
                m (0, 1) * m (0, 1) +
                m (0, 0) * m (2, 2) -
                m (0, 2) * m (0, 2) +
                m (1, 1) * m (2, 2) -
                m (1, 2) * m (1, 2);
    Scalar c2 = m (0, 0) + m (1, 1) + m (2, 2);


    if (fabs (c0) < Eigen::NumTraits<Scalar>::epsilon ())// one root is 0 -> quadratic equation
      computeRoots2 (c2, c1, roots);
    else
    {
      const Scalar s_inv3 = Scalar (1.0 / 3.0);
      const Scalar s_sqrt3 = std::sqrt (Scalar (3.0));
      // Construct the parameters used in classifying the roots of the equation
      // and in solving the equation for the roots in closed form.
      Scalar c2_over_3 = c2*s_inv3;
      Scalar a_over_3 = (c1 - c2 * c2_over_3) * s_inv3;
      if (a_over_3 > Scalar (0))
        a_over_3 = Scalar (0);

      Scalar half_b = Scalar (0.5) * (c0 + c2_over_3 * (Scalar (2) * c2_over_3 * c2_over_3 - c1));

      Scalar q = half_b * half_b + a_over_3 * a_over_3*a_over_3;
      if (q > Scalar (0))
        q = Scalar (0);

      // Compute the eigenvalues by solving for the roots of the polynomial.
      Scalar rho = std::sqrt (-a_over_3);
      Scalar theta = std::atan2 (std::sqrt (-q), half_b) * s_inv3;
      Scalar cos_theta = std::cos (theta);
      Scalar sin_theta = std::sin (theta);
      roots (0) = c2_over_3 + Scalar (2) * rho * cos_theta;
      roots (1) = c2_over_3 - rho * (cos_theta + s_sqrt3 * sin_theta);
      roots (2) = c2_over_3 - rho * (cos_theta - s_sqrt3 * sin_theta);

      // Sort in increasing order.
      if (roots (0) >= roots (1))
        std::swap (roots (0), roots (1));
      if (roots (1) >= roots (2))
      {
        std::swap (roots (1), roots (2));
        if (roots (0) >= roots (1))
          std::swap (roots (0), roots (1));
      }

      if (roots (0) <= 0) // eigenval for symetric positive semi-definite matrix can not be negative! Set it to 0
        computeRoots2 (c2, c1, roots);
    }
  }

   /** Compute the roots of a quadratic polynom x^2 + b*x + c = 0
    * \param[in] b linear parameter
    * \param[in] c constant parameter
    * \param[out] roots solutions of x^2 + b*x + c = 0
    */
  template<typename Scalar, typename Roots> inline void
  computeRoots2 (const Scalar& b, const Scalar& c, Roots& roots)
  {
    roots (0) = Scalar (0);
    Scalar d = Scalar (b * b - 4.0 * c);
    if (d < 0.0) // no real roots!!!! THIS SHOULD NOT HAPPEN!
      d = 0.0;

    Scalar sd = ::std::sqrt (d);

    roots (2) = 0.5f * (b + sd);
    roots (1) = 0.5f * (b - sd);
  }


