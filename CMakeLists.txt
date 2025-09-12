cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Include the third_party components
set(EXTRA_COMPONENT_DIRS "${CMAKE_SOURCE_DIR}/third_party/esp-video-components/esp_cam_sensor"
                         "${CMAKE_SOURCE_DIR}/third_party/esp-video-components/esp_sccb_intf"
                         "${CMAKE_SOURCE_DIR}/third_party/esp-video-components/esp_video"
                         "${CMAKE_SOURCE_DIR}/third_party/esp-video-components/esp_ipa")

project(esp-p4_camera)