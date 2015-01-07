# - Try to find DBoW2
# Once done this will define
#  DBoW2_FOUND - System has DBoW2
#  DBoW2_INCLUDE_DIRS - The DBoW2 include directories
#  DBoW2_LIBRARIES - The libraries needed to use DBoW2
#  DBoW2_DEFINITIONS - Compiler switches required for using DBoW2

set(DBoW2_DEFINITIONS "")

find_path(DBoW2_INCLUDE_DIR DBoW2/DBoW2.h)
find_library(DBoW2_LIBRARY NAMES DBoW2 libDBoW2)
find_library(DUtils_LIBRARY NAMES DUtils libDUtils)
find_library(DUtilsCV_LIBRARY NAMES DUtilsCV libDUtilsCV)
find_library(DVision_LIBRARY NAMES DVision libDVision)

set(DBoW2_LIBRARIES
  ${DBoW2_LIBRARY}
  ${DVision_LIBRARY}
  ${DUtils_LIBRARY}
  ${DUtilsCV_LIBRARY}
)

set(DBoW2_INCLUDE_DIRS
  ${DBoW2_INCLUDE_DIR}
  ${DBoW2_INCLUDE_DIR}/DBoW2/DVision
)

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set DBoW2_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(DBoW2 DEFAULT_MSG
                                  DBoW2_LIBRARY DBoW2_INCLUDE_DIR)

mark_as_advanced(DBoW2_INCLUDE_DIRS DBoW2_LIBRARY)
