
# if( UNIX )
# set( LIB_CVD "/home/avanindra/projects/libcvd/libcvd.so" )
# set( LIB_GVARS3 "/home/avanindra/projects/gvars/libGVars3.a" )
# endif()

add_definitions( -DCVD_HAVE_TOON )

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}
                     /usr/local/include
                     /opt/local/include)
                     #"/home/avanindra/projects/gvars"
                     #"/home/avanindra/projects/libcvd"
                     #"/home/avanindra/projects/TooN"
                     #"/home/avanindra/projects")
                     
link_directories( /usr/local/lib )                     

add_library( ptam   HomographyInit.cc Tracker.cc System.cc SmallBlurryImage.cc ShiTomasi.cc Relocaliser.cc PatchFinder.cc 
                   MiniPatch.cc MapViewer.cc MapPoint.cc MapMaker.cc Map.cc KeyFrame.cc 
                   GLWindowMenu.cc GLWindow2.cc EyeGame.cc  
                   CalibImage.cc CalibCornerPatch.cc Bundle.cc ATANCamera.cc ARDriver.cc )
                   
                   
add_executable( ptam_main main.cc VideoSourceOpencv.cpp )
add_executable( CameraCalibrator CameraCalibrator.cc VideoSourceOpencv.cpp )

message( STATUS " OpenGL Libraries :  ${OPENGL_LIBRARIES}" )

if( UNIX )
  target_link_libraries( ptam cvd GVars3  lapack ${OpenCV_LIBS}  ${QT_LIBRARIES}  ${OPENGL_LIBRARIES})#${LIB_CVD} ${LIB_GVARS3}
  target_link_libraries( ptam_main ptam cvd GL)
else()

  message( STATUS "GLEW Library : ${GLEW_LIB}" )

  target_link_libraries( ptam  ${GLEW_LIB} ${OpenCV_LIBS} ${PTAM_LIBRARIES}  OpenGL32 ${QT_LIBRARIES} ${MKL_LAPACK} hessian-feature-detector )
  target_link_libraries( ptam_main ptam ${OpenCV_LIBS} )

  get_target_property( CameraCalibrator_BIN_NAME CameraCalibrator LOCATION )

  get_target_property( PTAM_BIN_NAME ptam_main LOCATION )

  add_custom_command( TARGET CameraCalibrator  POST_BUILD
                      COMMAND ${CMAKE_COMMAND} -E make_directory ${CURRENT_PROJECT_BINARY_DIR}
                      COMMAND ${CMAKE_COMMAND} -E copy ${CameraCalibrator_BIN_NAME} ${CURRENT_PROJECT_BINARY_DIR}  )

  add_custom_command( TARGET ptam_main  POST_BUILD
                      COMMAND ${CMAKE_COMMAND} -E make_directory ${CURRENT_PROJECT_BINARY_DIR}
                      COMMAND ${CMAKE_COMMAND} -E copy ${PTAM_BIN_NAME} ${CURRENT_PROJECT_BINARY_DIR}  )
endif()


target_link_libraries( CameraCalibrator ptam ${OpenCV_LIBS} GL)
