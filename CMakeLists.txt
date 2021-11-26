cmake_minimum_required(VERSION 3.12)

project(h3lis331dl)

set(H3_SOURCES
    "src/h3lis331dl.c"
)

set(H3LIS331DL_INCLUDES
    "include"
)

add_library(${PROJECT_NAME} ${H3_SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC ${H3LIS331DL_INCLUDES})