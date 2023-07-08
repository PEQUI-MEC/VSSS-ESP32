set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../lib)
include("${LIB_DIR}/Eigen/eigen_files.cmake")


idf_component_register(SRCS
    "main.cpp"
    "esp_now_msg.cpp"
    "motor_control.cpp"
    "encoder.cpp"
    "imu.cpp"
    ${EIGEN_FILES}
    INCLUDE_DIRS "."
    ${EIGEN_DIRS})
