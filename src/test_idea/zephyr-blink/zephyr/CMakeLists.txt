# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(blinky)

#find_package(Zephyr)
#find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
#zephyr_include_directories(.)   # Include the current directory

target_sources(app PRIVATE ../src/main.c)
#target_sources(app PRIVATE ../src/uart.c)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../src)