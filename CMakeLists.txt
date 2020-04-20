cmake_minimum_required(VERSION 3.7)
project(hw2)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

FIND_PACKAGE( OpenCV REQUIRED)
FIND_PACKAGE( OpenGL REQUIRED)
FIND_PACKAGE( GLU REQUIRED)
FIND_PACKAGE( GLUT REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})

SET(MySources opengl_cv_skeleton.cpp camera_calibration.cpp)
foreach( sourcefile ${MySources} )
    string( REPLACE ".cpp" "" source ${sourcefile} )
    ADD_EXECUTABLE( ${source} ${sourcefile} )
    TARGET_LINK_LIBRARIES( ${source} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} )
endforeach( sourcefile ${MySources} )

#ADD_EXECUTABLE( opengl_cv_skeleton ${MySources} camera_calibration.cpp ${MySources})
    #TARGET_LINK_LIBRARIES( opengl_cv_skeleton ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} )
