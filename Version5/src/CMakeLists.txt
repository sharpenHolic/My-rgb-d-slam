# 增加PCL库的依赖
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization filters)

# 增加opencv的依赖
FIND_PACKAGE( OpenCV 3.2.0 REQUIRED )

# 添加头文件和库文件
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

#将写好的库文件链接
ADD_LIBRARY( slambase_version5 slamBase.cpp )
TARGET_LINK_LIBRARIES( slambase_version5
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES} )


#生成可执行文件
ADD_EXECUTABLE(visualOdometry visualOdometry.cpp )
#可执行文件链接相应的库文件
TARGET_LINK_LIBRARIES(visualOdometry ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        slambase_version5 )
