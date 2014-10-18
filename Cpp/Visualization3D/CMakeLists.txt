

SET( VC_VISUALIZATION_3D_LINK_LIBRARIES "" )

LIST( APPEND VC_VISUALIZATION_3D_LINK_LIBRARIES ${VC_VISUALIZATION_3D_LINK_LIBRARIES} ${QT_LIBRARIES}  )


add_library( visualization3d offscreenrenderer.cpp buffermanager.cpp gl_hud.cpp hud.cpp font_image.cpp openglcamera.cpp trackballcamera.cpp )

target_link_libraries( visualization3d ${VC_VISUALIZATION_3D_LINK_LIBRARIES} )

#if( WIN32 )
 set_property( TARGET visualization3d PROPERTY FOLDER "UserInterface")
#endif()