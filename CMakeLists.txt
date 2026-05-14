cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(pycontroller_cpp)

# This command takes everything inside the "data/" folder, 
# creates a file system, and flashes it to the ESP32!
spiffs_create_partition_image(spiffs data FLASH_IN_PROJECT)