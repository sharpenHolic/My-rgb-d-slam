# 增加PCL库的依赖
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization)

# 增加opencv的依赖
FIND_PACKAGE( OpenCV REQUIRED )

# 添加头文件和库文件
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

#将写好的库文件链接
ADD_LIBRARY( slambase_version4 slamBase.cpp )
TARGET_LINK_LIBRARIES( slambase_version4
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES} )

#生成可执行文件
ADD_EXECUTABLE( generatePointCloud_version4 generatePointCloud.cpp )
#可执行文件链接相应的库文件
TARGET_LINK_LIBRARIES( generatePointCloud_version4 ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        slambase_version4 )

#生成可执行文件
ADD_EXECUTABLE( detectFeatures_version4 detectFeatures.cpp)
#可执行文件链接相应的库文件
TARGET_LINK_LIBRARIES( detectFeatures_version4 ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        slambase_version4 )

#生成可执行文件
ADD_EXECUTABLE( joinPointCloud_version4 joinPointCloud.cpp)
#可执行文件链接相应的库文件
TARGET_LINK_LIBRARIES( joinPointCloud_version4 ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        slambase_version4 )