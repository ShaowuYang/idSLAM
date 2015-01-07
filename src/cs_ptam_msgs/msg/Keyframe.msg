Header header

int64 id # keyframe id (sequential)
time kftime

geometry_msgs/Pose pose

bool haveImuData
geometry_msgs/Quaternion naviTCam

bool haveGroundData
geometry_msgs/Pose groundTCam

Mappoint[] mappoints
Level[] levels

DepthImage depthImage
RGBImage   rgbImage   # if available
