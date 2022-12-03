cmake_minimum_required(VERSION 3.12)

set(PICO_SDK_PATH /usr/share/pico-sdk)
set(PICO_EXTRAS_PATH /home/hkk/Documents/c/pico/pico-extras)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_BOARD pico_w)

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include(${PICO_EXTRAS_PATH}/external/pico_extras_import.cmake)

# include(pico_sdk_import.cmake)
# include(pico_extras_import.cmake)

project(Pico-RTOS)

# set(LWIP_DIR ${PICO_EXTRAS_PATH}/lib/lwip)
# include(${LWIP_DIR}/src/Filelists.cmake)

pico_sdk_init()

add_subdirectory(src)
add_subdirectory(rtos)