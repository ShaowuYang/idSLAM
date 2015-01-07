add_library(cslam_reg
  RegistratorSIM3.cpp
  Registrator3P.cpp
  Registrator5P.cpp
  RegistratorBAPTAM.cpp
  RegistratorKFs.cpp
)

target_link_libraries(cslam_reg ptam yaml-cpp)

