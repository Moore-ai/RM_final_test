cmake_minimum_required(VERSION 3.20)

project(learn LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

# add_definitions(-DIGNORE_INPUT)

file(GLOB MAIN_SRC ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
file(GLOB MAIN_HEAD ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp)

message(STATUS "头文件 = ${MAIN_HEAD}")
message(STATUS "源文件 = ${MAIN_SRC}")

find_package(OpenCV REQUIRED)
find_package(nlohmann_json 3.11.3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(run ${MAIN_HEAD} ${MAIN_SRC})

target_link_libraries(run PUBLIC ${OpenCV_LIBS})
target_link_libraries(run PRIVATE nlohmann_json::nlohmann_json)
