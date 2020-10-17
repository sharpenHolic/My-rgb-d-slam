bin 用来放编译好的可执行二进制文件。
src 用来放源代码。
lib 用来放编译好的库文件。
include 用来放头文件。
为什么要用这种目录结构呢？其实这是一种编译习惯，当然你可以把所有文件都搁一个目录里，但是这样看起来很乱不是么。通常我们把源代码和编译好的东西分开。如果源代码比较多，还会按模板分开。像opencv和pcl的代码就是由好多个模块组成的。





### version2 实现了单帧图像的点云表示

### version3 实现了slamBase的库函数构建

### version3实现了功能函数的封装

在main函数中实现了功能之后，把函数封装出来

利用了结构体和类来对Frame和读取参数文件的操作进行封装

### version4实现了两帧图片点云的拼接

### version5完成一个视觉里程计(visual odometry)的编写

在写函数的时候，注意头文件的定义和实现之中对应的参数要一致，不然出现找不到定义的错误
// 求解pnp
细节一：当goodmatches为少于5的情况需要做一个判断：

```c++
if (_goodMatches.size() <= 5)
    {
        _res.inliers = -1;
        return _res;
    }
```

细节二：求解的R和t到底是以谁为坐标系的旋转和平移

### version6加上后端优化

https://www.cnblogs.com/gaoxiang12/p/4739934.html

累积误差是里程计中不可避免的，后续的相机姿态依赖着前面的姿态。想要保证地图的准确，必须要保证每次匹配都精确无误，而这是难以实现的。所以，我们希望用更好的方法来做slam。不仅仅考虑两帧的信息，而要把所有整的信息都考虑进来，成为一个全slam问题（full slam）。

引入图优化：

在图优化中，顶点代表了要被优化的变量，而边则是连接被优化变量的桥梁，因此，也就造成了说我们在程序中见得较多的就是这两种类型的初始化和赋值。

[图优化原理](https://blog.csdn.net/heyijia0327/article/details/47686523)

加入位姿图来进行优化

顶点和边有不同的类型，这要看我们想求解什么问题。由于我们是3D的slam，所以顶点取成了相机姿态：g2o::VertexSE3，而边则是连接两个VertexSE3的边：g2o::EdgeSE3
V ：节点
E：边


### 关于cmake知识

https://www.jianshu.com/p/7d5284ca6ae5

使用过很多库应该发现他们都有一个文件夹叫`cmake`或者`cmake_modules`之类的。这类文件夹一般包含很多`.cmake`文件，这类文件把本来该写在CMakeLists.txt里的内容分散开来，用来找特定的库之类的

可执行程序需要用到一个叫`libbayesopt.a`的静态库，库的位置位于`/usr/local/lib`.
 我们使用一些有名的库的时候，他好像自己就能找到，不需要多费精神，比如OpenCv，我们只需要在CMakeLists.txt里添加

```yaml
# 添加g2o的依赖
# 因为g2o不是常用库，要添加它的findg2o.cmake文件

# 告诉cmake在哪儿去找我们的modules
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
SET( G2O_ROOT /usr/local/include/g2o )
FIND_PACKAGE( G2O )
# CSparse
FIND_PACKAGE( CSparse )
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )
```

