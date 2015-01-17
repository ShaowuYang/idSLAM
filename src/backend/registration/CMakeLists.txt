add_library(backend_reg
  RegistratorSIM3.cpp
  Registrator3P.cpp
  RegistratorKFs.cpp
)

target_link_libraries(backend_reg ptam yaml-cpp)

