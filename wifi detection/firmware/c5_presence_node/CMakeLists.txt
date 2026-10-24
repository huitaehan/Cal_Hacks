# ESP32-C5 presence node: ESP-NOW CSI link + RuView edge processing (ADR-039)
# Requires ESP-IDF v5.5+ (first release with ESP32-C5 support)
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(c5_presence_node)
