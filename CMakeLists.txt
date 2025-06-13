#
# CMakeLists.txt para el módulo disturbance_estimator
#

px4_add_module(
        MODULE modules_disturbance_estimator
        MAIN disturbance_estimator
        SRCS
                disturbance_estimator.cpp
        DEPENDS
                px4_platform
                uORB


)

