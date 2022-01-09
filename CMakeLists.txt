cmake_minimum_required(VERSION 3.0.0)
project(dataViewer VERSION 0.1.0)

find_package(Pangolin REQUIRED)
find_package(OpenCV 3 REQUIRED)

include_directories( 
  ${Pangolin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS} 
  include/
  )

add_executable(run_viewer src/run_viewer.cc src/data_viewer.cc)
target_link_libraries(run_viewer 
  ${Pangolin_LIBRARIES} 
  ${EIGEN3_LIBS}
  ${OpenCV_LIBS}
  )

