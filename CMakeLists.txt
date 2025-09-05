cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Bruk denne KUN hvis du har egne lokale komponenter i ./components
# set(EXTRA_COMPONENT_DIRS "${CMAKE_SOURCE_DIR}/components")

project(esp-p4_camera)