dual camera SLAM. This tends to extend PTAM with full SLAM capability, inculding loop closing and pose drift corrections in large scale.

port rosnodes to nodelets:

    add the necessary #includes
    get rid of int main()
    subclass nodelet::Nodelet
    move code from constructor to onInit()
    add the PLUGINLIB_DECLARE_CLASS macro

    add the <nodelet> item in the <export> part of the package manifest
    create the .xml file to define the nodelet as a plugin
    make the necessary changes to CMakeLists.txt (comment out a rosbuild_add_executable, add a rosbuild_add_library) 

in this project, only the camera drivers and main slam is orgernised in nodelet manager, since massive image copy can be avoided.

###########################################
Installation:

1. install all required library for PTAM
   TooN, libcvd, gvars-3.0 (ptam install)
2. install and correctly inculde the following library for the back-end:
	DBoW2
	G2O
	Sophus

############################################
Problems occured before:
1. lib and include dir error:

CMake Error: The following variables are used in this project, but they are set to NOTFOUND.
Please set them or make sure they are set and tested correctly in the CMake files:
CSPARSE_LIBRARY
    linked by target "backend" in directory /home/young/localhome/gitlab/dcslam/src/backend/cslam
    linked by target "creatvoc" in directory /home/young/localhome/gitlab/dcslam/src/backend/cslam/loops
G2O_CORE_LIBRARY
    linked by target "backend" in directory /home/young/localhome/gitlab/dcslam/src/backend/cslam
    linked by target "creatvoc" in directory /home/young/localhome/gitlab/dcslam/src/backend/cslam/loops
G2O_INCLUDE_DIR
   used as include directory in directory /home/young/localhome/gitlab/dcslam/src/backend/cslam
   used as include directory in directory /home/young/localhome/gitlab/dcslam/src/backend/cslam/tools
   used as include directory in directory /home/young/localhome/gitlab/dcslam/src/backend/cslam/registration
   used as include directory in directory /home/young/localhome/gitlab/dcslam/src/backend/cslam/matching
   used as include directory in directory /home/young/localhome/gitlab/dcslam/src/backend/cslam/loops
   used as include directory in directory /home/young/localhome/gitlab/dcslam/src/backend/cslam/loops
   used as include directory in directory /home/young/localhome/gitlab/dcslam/src/backend/cslam/slam
   used as include directory in directory /home/young/localhome/gitlab/dcslam/src/backend/cslam/cs_geometry
   used as include directory in directory /home/young/localhome/gitlab/dcslam/src/backend/cslam/g2o_types
G2O_SOLVER_CSPARSE
    linked by target "backend" in directory /home/young/localhome/gitlab/dcslam/src/backend/cslam
    linked by target "creatvoc" in directory /home/young/localhome/gitlab/dcslam/src/backend/cslam/loops
G2O_STUFF_LIBRARY
    linked by target "backend" in directory /home/young/localhome/gitlab/dcslam/src/backend/cslam
    linked by target "creatvoc" in directory /home/young/localhome/gitlab/dcslam/src/backend/cslam/loops
G2O_TYPES_SIM3
    linked by target "backend" in directory /home/young/localhome/gitlab/dcslam/src/backend/cslam
    linked by target "creatvoc" in directory /home/young/localhome/gitlab/dcslam/src/backend/cslam/loops
G2O_TYPES_SLAM3D
    linked by target "backend" in directory /home/young/localhome/gitlab/dcslam/src/backend/cslam
    linked by target "creatvoc" in directory /home/young/localhome/gitlab/dcslam/src/backend/cslam/loops

reason:
(1) g2o wasn't fully installed
(2) CSPARSE_LIB: remove CSPARSE_LIB from EXTERNAL_LIB, but this may potentially cause problems later.

2. g2o related problems: the backend only support the old g2o library. However, the cmakelist in the old g2o library 
is modified according the lated svn version in order to find the cholmod solver.

3. LinearMath/btTransform.h cannot be found:
	bullet libary need to be installed as dependency. apt-get libbullet-dev, link LinearMath to /usr/include

4. /usr/bin/ld: cannot find -ltoontag:
	install tag, need to cp SVD.h in TooN to svd.h
