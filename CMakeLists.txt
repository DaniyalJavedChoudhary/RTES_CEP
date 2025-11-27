# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(RTES_CEP)

# Create a SPIFFS partition image from the project's `data/` folder
# This will produce a partition image and include it in the flash binary
# The partition name `storage` must match the entry in `partitions.csv`
spiffs_create_partition_image(storage data FLASH_IN_PROJECT)
