![image-20201016102208673](/home/xgq/.config/Typora/typora-user-images/image-20201016102208673.png)



## 跟着高博完成一个RGBD-SLAM

**bin** 用来放编译好的可执行二进制文件。
**src** 用来放源代码。
**lib** 用来放编译好的库文件。
**include** 用来放头文件。
为什么要用这种目录结构呢？其实这是一种编译习惯，当然你可以把所有文件都搁一个目录里，但是这样看起来很乱不是么。通常我们把源代码和编译好的东西分开。如果源代码比较多，还会按模板分开。像opencv和pcl的代码就是由好多个模块组成的。



### version1 hello world

### version2 实现了单帧图像的点云表示

### version3 实现了slamBase的库函数构建

### version3实现了功能函数的封装

### version4实现了两帧图片点云的拼接


