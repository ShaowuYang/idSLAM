idSLAM (InDoorSLAM)
======

A SLAM system using multiple Kinects for indoor applications. It extends the PTAM system (http://www.robots.ox.ac.uk/~gk/PTAM/) as described in [4] 
to be able to integrate measurements from multiple Kinects. Furthermore, an efficient back-end for loop closing is implemented 
to form a full SLAM system. More robust performance is achieved using our multi-kinect setup, compared with conventional RGBD-SLAM systems.

This work is based on the previous work described in [1, 2, 3].

[1]	Shaowu Yang, Sebastian A. Scherer, and Andreas Zell. Robust onboard visual SLAM for autonomous MAVs. 
    In 2014 International Conference on Intelligent Autonomous Systems (IAS-13), Padova, Italy, July 2014. 

[2]	Shaowu Yang, Sebastian A. Scherer, and Andreas Zell. Visual SLAM for autonomous MAVs with dual cameras. 
    In 2014 International Conference on Robotics and Automation (ICRA'14), Hongkong, China, June 2014.

[3] Sebastian A. Scherer, Daniel Dube, and Andreas Zell. Using Depth in Visual Simultaneous Localisation and Mapping. 
    In IEEE International Conference on Robotics and Automation, St. Paul, Minnesota, USA, May 2012. 

[4] Georg Klein and David Murray. Parallel tracking and mapping for small AR workspaces. 
    In Proc. Sixth IEEE and ACM International Symposium on Mixed and Augmented Reality (ISMAR’07), Nara, Japan, November 2007.

The following third-party libraries are used in this system:

1. Third-party libraries used in PTAM: TooN, libCVD, Gvars3

2. DBoW2 (http://webdiis.unizar.es/~dorian/index.php?p=32)

3. g2o (http://openslam.org/g2o.html)
