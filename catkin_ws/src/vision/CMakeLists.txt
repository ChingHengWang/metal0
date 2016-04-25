cmake_minimum_required (VERSION 2.8.3)
project (vision)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR})


find_package(catkin REQUIRED COMPONENTS message_generation roscpp std_msgs image_transport cv_bridge)

find_package (OpenCV REQUIRED)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -o0")


add_message_files(
  FILES
  RecoGoal.msg
  RecoResults.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp std_msgs sensor_msgs image_transport cv_bridge
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})


# Set the name to use for the executable.
set (BINNAME1 vision)

# Set the source files to use with the executable.
set (SRCS1 ${SRCS1} src/main.cpp)
set (SRCS1 ${SRCS1} src/FaceDetector.cpp)
set (SRCS1 ${SRCS1} src/FeedbackPublisher.cpp)
set (SRCS1 ${SRCS1} src/FaceRecognizerTeacher.cpp)
set (SRCS1 ${SRCS1} src/FaceRecog.cpp)
set (SRCS1 ${SRCS1} src/BodyDetector.cpp)
set (SRCS1 ${SRCS1} src/BodyRecognizerTeacher.cpp)
set (SRCS1 ${SRCS1} src/BodyRecognizer.cpp)
set (SRCS1 ${SRCS1} src/VisionNode.cpp)

# Build the executable that will be used to run this node.
add_executable (${BINNAME1} ${SRCS1})
add_dependencies(${BINNAME1} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${BINNAME1} ${catkin_LIBRARIES} ${OpenCV_LIBS})

file(COPY predefined DESTINATION .)
file(COPY res DESTINATION .)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${BINNAME1}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

set(CPACK_PACKAGE_VERSION_MAJOR "1")
set(CPACK_PACKAGE_VERSION_MINOR "0")
set(CPACK_PACKAGE_VERSION_PATCH "0")
set(CPACK_SOURCE_GENERATOR "TGZ")
set(CPACK_SOURCE_PACKAGE_FILE_NAME
  "${CMAKE_PROJECT_NAME}-${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
 
add_custom_target(dist COMMAND ${CMAKE_MAKE_PROGRAM} package_source)

include(CPack)


