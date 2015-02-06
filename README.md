Opticks_GSoC2014
================

LiDAR segmentation Plug-In based on RANSAC and PCA algorithms for Opticks

This is the repository for Opticks GSoC 2014: LiDAR segmentation Plug-In based on RANSAC and PCA algorithms.

The Plug-In implements a RANSAC-based technique for extracting roof planes of buildings from LiDAR point clouds.

Additional information about the project is available at the following link: http://opticks.org/confluence/display/~roberta.ravanelli/GSoC+2014%3A+LiDAR+segmentation+Plug-In+based+on+RANSAC+and+PCA+algorithms+for+Opticks

Documentation for the Pug-In can be found at: http://opticks.org/confluence/display/~roberta.ravanelli/Final+report%3A+from+08-15-2014+to+08-18-2014

================ Installing instructions ====================================================

Download the .aeb installation file

Copy the opencv_highgui220.dll and opencv_highgui220d.dll (you can find them into Dependencies folder) into Opticks installation Bin folder (for me C:\Program Files\Opticks\4.12.0\Bin)

Run Opticks as Administrator

Go to Help->Extensions and install the .aeb installation file

================ Compiling instructions ====================================================

Inside the LiDAR folder, there is the Visual Studio project file (LiDAR.vcxproj). To compile it, the user can add it to the Opticks SDK solution (the version 20140410 of the Opticks SDK was used), but first some steps are needed.

OPTICKS_CODE_DIR environment variable must be set: it is the path to the location where the Opticks SDK is installed.

All the Plug-In classes depend on the Eigen library, that it is not included in the Opticks dependencies: instructions on how to use it inside Visual Studio can be found at following link (http://eigen.tuxfamily.org/index.php?title=Visual_Studio ).

Since the Plug-In uses the OpenCV highgui methods, the default OpenCV property sheets (SDK_HOME/application/CompileSettings) must be changed (both the debug and release version), adding the opencv_highgui220.lib to the additional dependencies (line 11).

In addition, the cxcore.h header file (needed for the conversion from/to OpenCV - Eigen matrices) must be copied in the OpenCV2 core folder (SDK_HOME/Dependencies/64/include/opencv2/core).

Then, to run the Plug-In (Opticks 4.12 Nightly Build 20140410 was used), the open_cv_highgui220.dll and open_cv_highgui220d.dll must be copied from the SDK dependencies folder (SDK_HOME/Dependencies/64/bin) to the Opticks installation bin folder (C:/Program Files/Opticks/4.12Nightly20140410.18697/Bin).

=============== Test point cloud ===========================================================

A test point cloud is available at the link below:

https://www.dropbox.com/s/iewrzirro79o0hn/GSoC2014_test_point_cloud.las

=============== Contacts ===================================================================

Roberta Ravanelli: roberta.ravanelli@uniroma1.it