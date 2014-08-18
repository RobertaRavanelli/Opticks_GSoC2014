Opticks_GSoC2014
================

LiDAR segmentation Plug-In based on RANSAC and PCA algorithms for Opticks

This is the repository for Opticks GSoC 2014: LiDAR segmentation Plug-In based on RANSAC and PCA algorithms.

For more information about this project: http://opticks.org/confluence/display/~roberta.ravanelli/GSoC+2014:+LiDAR+segmentation+Plug-In+based+on+RANSAC+and+PCA+algorithms+for+Opticks

The program now saves the png files of the point cloud tiles inside the directory "Tiles" that must be inside the Results directory (C:\Users\Roberta\Desktop\Results\Tiles).

OPTICKS CODE DIR environment variable must be set: it is the path to the location where the Opticks SDK is installed.

All the Plug-In classes depend on the Eigen library, that it is not included in the Opticks dependencies: instructions on how to use it inside Visual Studio can be found at following link (http://eigen.tuxfamily.org/index.php?title=Visual_Studio).

Since the Plug-In uses the OpenCV highgui methods, the default OpenCV property sheets (SDK HOME/application/CompileSettings) must be changed (both the debug and release version), adding the opencv_highgui220.lib to the additional dependencies (line 11).

In addition, the cxcore.h header filele (needed for the conversion from/to OpenCV - Eigen matrices) must be copied in the OpenCV2 core folder (SDK_HOME/Dependencies/64/include/opencv2/core).

Then, to run the Plug-In (Opticks 4.12 Nightly Build 20140410 was used), the open_cv_highgui220.dll and open_cv_highgui220d.dll must be copied from the SDK dependencies folder (SDK_HOME/Dependencies/64/bin) to the Opticks installation bin folder (C:/Program Files/Opticks/4.12Nightly20140410.18697/Bin).

=============== Test point cloud ========

You can find a test point cloud at the link below:

https://www.dropbox.com/s/iewrzirro79o0hn/GSoC2014_test_point_cloud.las