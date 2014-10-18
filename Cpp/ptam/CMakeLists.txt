
add_definitions( -DCVD_HAVE_TOON )

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}
                     )

add_library( ptam   HomographyInit.cc Tracker.cc System.cc SmallBlurryImage.cc ShiTomasi.cc Relocaliser.cc PatchFinder.cc 
                   MiniPatch.cc MapViewer.cc MapPoint.cc MapMaker.cc Map.cc KeyFrame.cc 
                   GLWindowMenu.cc GLWindow2.cc EyeGame.cc  
                   CalibImage.cc CalibCornerPatch.cc Bundle.cc ATANCamera.cc ARDriver.cc )
                   
                   
add_executable( ptam_main main.cc VideoSourceOpencv.cpp )
add_executable( CameraCalibrator CameraCalibrator.cc VideoSourceOpencv.cpp )

target_link_libraries( ptam ${PTAM_LIBRARIES}  lapack ${OpenCV_LIBS}  ${QT_LIBRARIES}  ${OPENGL_LIBRARIES})
target_link_libraries( ptam_main ptam ${PTAM_LIBRARIES} GL)

target_link_libraries( CameraCalibrator ptam ${OpenCV_LIBS} GL)
