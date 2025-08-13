project(Picking_Studio)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  pcl_conversions
  pcl_ros
  rviz
  visualization_msgs
  interactive_markers
  dangkang_msgs
  hy_manipulation_applications
)

find_package(Qt5 REQUIRED Core Widgets Charts)
find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED QUIET)

catkin_package()  # 关键：注册为 catkin 包

set(QT_LIBRARIES Qt5::Widgets Qt5::Charts)

file(GLOB SOURCES_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)
file(GLOB INCLUDE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Picking_Studio/*.hpp
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include           # 允许包含 "Picking_Studio/mainwindow.hpp"
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 若要用本地 lib 目录，写成正确路径；否则注释掉
# link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

add_executable(picking_studio
  app/main.cpp
  ${SOURCES_FILES}
  ${INCLUDE_FILES}    # 显式列入，保证 AUTMOC 处理到 Q_OBJECT
)

target_link_libraries(picking_studio
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  Eigen3::Eigen
  ${OpenCV_LIBS}
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libElaWidgetTools.so
)

# 安装（可选，用于 install 空间）
install(TARGETS picking_studio
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)