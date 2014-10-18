cmake_minimum_required (VERSION 2.6)
set(CMAKE_BUILD_TYPE Debug) 
project(dtam)
cmake_policy(SET CMP0012 NEW)
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/local_store/Qt/5.3/gcc_64/lib/cmake")# "/local_store/opencv-2.4.9/debug/unix-install/")
#message(STATUS ${CMAKE_MODULE_PATH})
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g -ggdb -fpermissive")

find_package( OpenCV 2.4 REQUIRED )
# find_package( Qt5Core REQUIRED)
find_package(CUDA REQUIRED)
find_package(Boost COMPONENTS system thread)

find_package( OpenGL )

#find_package( MKL REQUIRED)
find_package(Qt5Widgets)
find_package(Qt5Declarative)
find_package(Qt5MultimediaWidgets)
find_package(Qt5OpenGL)
find_package(Qt5Core)
find_package(Qt5Gui)
find_package(Qt5Multimedia)
find_package(Qt5Sql)
find_package(Qt5WebKitWidgets)
#add_definitions( -D__OPENCV_BUILD=1 )
#message(STATUS ${OpenCV_CONSIDERED_CONFIGS})

set( EIGEN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/eigen3 CACHE PATH "set eigen dir" ) 
 
link_directories( /opt/intel/ipp/lib/intel64 /opt/intel/lib/intel64)

set( VC_OPENGL_CORE_PROFILE "GLEW" CACHE STRING "Select OpenGL Core Profile Provider")

SET_PROPERTY( CACHE VC_OPENGL_CORE_PROFILE PROPERTY STRINGS "GLEW" "QOPENGL_FUNCTIONS" )


add_definitions( -DDATASET_PATH="${CMAKE_CURRENT_SOURCE_DIR}/dataset/DSCF0159.AVI" )

if(  ${VC_OPENGL_CORE_PROFILE} STREQUAL "GLEW"  )

add_definitions( -DVC_GLEW )

else()
 add_definitions( -DVC_QOPENGL_FUNCTIONS )
endif() 
#add_library(GraphicsLib SHARED graphics.cpp)#must be in own library so we can link against it.

set( QT_INCLUDES 
                 ${Qt5Widgets_INCLUDE_DIRS}
                 ${Qt5Core_INCLUDE_DIRS} 
				 ${Qt5MultimediaWidgets_INCLUDE_DIRS}
				 ${Qt5OpenGL_INCLUDE_DIRS}
				 ${Qt5Multimedia_INCLUDE_DIRS}
				 ${Qt5Gui_INCLUDE_DIRS}
				 ${Qt5Declarative_INCLUDE_DIRS}
				 ${Qt5WebKitWidgets_INCLUDE_DIRS}
 )
 
 set ( QT_LIBRARIES ${Qt5Widgets_LIBRARIES} 
                    ${Qt5Core_LIBRARIES}
                    ${Qt5MultimediaWidgets_LIBRARIES}
                    ${Qt5OpenGL_LIBRARIES}
                    ${Qt5Multimedia_LIBRARIES}
                    ${Qt5Gui_LIBRARIES}		
                    ${Qt5Declarative_LIBRARIES}		
                    ${Qt5Sql_LIBRARIES}
                    ${Qt5WebKitWidgets_LIBRARIES}
		)
		
set( GLFW3_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/link_libraries/glfw.so CACHE PATH "set glfw3 library" )	

set( PTAM_LIBRARIES cvd GVars3 )

macro (add_sources)
    file (RELATIVE_PATH _relPath "${CMAKE_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}")
    foreach (_src ${ARGN})
        if (_relPath)
            list (APPEND DTAM_SOURCES "${_relPath}/${_src}")
        else()
            list (APPEND DTAM_SOURCES "${_src}")
        endif()
    endforeach()
    if (_relPath)
        # propagate SRCS to parent directory
        set (DTAM_SOURCES ${DTAM_SOURCES} PARENT_SCOPE)
    endif()
endmacro(add_sources)



set(BASEPATH "${CMAKE_SOURCE_DIR}")
 
include_directories( "${BASEPATH}" ${CUDA_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN_DIR} ${QT_INCLUDES} ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty_includes )
#add_library(GraphicsLib SHARED graphics.cpp)#must be in own library so we can link against it.


add_subdirectory(CostVolume)
add_subdirectory(Track)
add_subdirectory(Optimizer)
add_subdirectory(DepthmapDenoiseWeightedHuber)
add_subdirectory(utils)
add_subdirectory(Scheduler)
add_subdirectory( ptam )
add_subdirectory( Visualization3D )
add_subdirectory(link_libraries)
add_subdirectory(thirdparty_includes)

qt5_add_resources(SHADERS_SRCS shaders.qrc )

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; -gencode arch=compute_30,code=sm_30; -gencode arch=compute_35,code=sm_35;--use_fast_math ;--restrict; -lineinfo; -O3)
cuda_add_library(OpenDTAM SHARED ${DTAM_SOURCES} OpenDTAM.cpp)
target_link_libraries(OpenDTAM ${OpenCV_LIBS})
add_executable( a.out testprog.cpp convertAhandaPovRayToStandard.cpp graphics.cpp)
add_executable( opendtamdemo opendtamdemo.cpp projectdata.cpp densedata.cpp drawimage.cpp graphics.cpp depthmapstereo.cpp ${SHADERS_SRCS})
target_link_libraries( a.out  OpenDTAM ${OpenCV_LIBS} ${Boost_LIBRARIES})
target_link_libraries( opendtamdemo  OpenDTAM ${OpenCV_LIBS} ${Boost_LIBRARIES} ${QT_LIBRARIES} visualization3d GLEW cvd ptam ${GLFW3_LIBRARY} )



