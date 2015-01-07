Header header

int64 id # keyframe id (sequential)
time kftime

int32 nSourceCamera
bool mAssociatedinFinalQueue
bool mAssociateKeyframe
uint32 nAssociatedKf

geometry_msgs/Pose pose

bool haveImuData
geometry_msgs/Quaternion naviTCam

Mappoint[] mappoints
Level[] levels

DepthImage depthImage
RGBImage   rgbImage   # if available
