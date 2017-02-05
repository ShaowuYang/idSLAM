add_library(ptam
    ATANCamera.cc
    KeyFrame.cc
    MapPoint.cc
    Map.cc
    SmallBlurryImage.cc
    ShiTomasi.cc
    HomographyInit.cc
    MapMaker.cc
    Bundle.cc
    PatchFinder.cc
    Relocaliser.cc
    MiniPatch.cc
    Tracker.cc
    CameraModel.cc
    PolynomialCamera.cc
)

target_link_libraries(ptam
    lapack
    GVars3
    cvd
#    cs_geometry
)
