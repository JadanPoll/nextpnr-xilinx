set(Eigen3_FOUND TRUE)
set(EIGEN3_FOUND TRUE)
set(EIGEN3_INCLUDE_DIR /usr/include/eigen3)
if(NOT TARGET Eigen3::Eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES /usr/include/eigen3)
endif()
