# This is the top-level project CMake file.
cmake_minimum_required(VERSION 3.16)

# Include the ESP-IDF project-level cmake file.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Declare the project name.
project(pycontroller_cpp)

# (spiffs_create_partition_image removed so the build doesn't 
# require the images or data folder to successfully build the firmware .bin)