parameter_wrt.txt 
contains camera intrinsic and extrinsic parameter.

extrinsic parameters are those of the camera wrt the calibration pattern,
for camera to camera calibration. (Tc_ext or Tcp)

camera intrinsic parameters are kept in each folder. 
now cam2cam calibration becomes very simple, since the pattern is fixed, with three markers fixed on it.
Then we only need to change the cam2pattern transformation and the corresponding robot pose in camera_wrt.txt.

so:

1. set ground plane with our pattern. the long side length is 295mm

2. make trackerble of the quadrotor.

3. take a image of the pattern by downward looking camera, and log the quadrotor pose info from the tracking system.

4. do the same in 3 for the forward looking camera.

5. do camera extrinsic calibrations for down- and for-ward cameras.

6. change the cameras_wrt.txt.

7. roslaunch dmslam calib.launch to get the cam2cam.txt .
