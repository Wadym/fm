# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(DTC_OVERLAY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/boards/teensy40.overlay")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(simple_drone)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/led.c)
#target_sources(app PRIVATE src/pin.c)
#target_sources(app PRIVATE src/button.c)
#target_sources(app PRIVATE src/uart.c)
#target_sources(app PRIVATE src/uart4.c)

