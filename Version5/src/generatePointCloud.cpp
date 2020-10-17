/*************************************************************************
	> File Name: Version4/src/generatePointCloud.cpp
	> Author: gaoqiang xiong
	> Mail: 1142008606@qq.com
    > 点云输出
	> Created Time: 2020年10月9日
 ************************************************************************/

#include <slamBase.h>


// 主函数
int main( int argc, char** argv )
{
    // 读取./data/rgb.png和./data/depth.png，并转化为点云

    // 图像矩阵
    cv::Mat rgb, depth;
    // 使用cv::imread()来读取图像
    // API: http://docs.opencv.org/modules/highgui/doc/reading_and_writing_images_and_video.html?highlight=imread#cv2.imread
    rgb = cv::imread( "./data/rgb.png" );
    // rgb 图像是8UC3的彩色图像
    // depth 是16UC1的单通道图像，注意flags设置-1,表示读取原始数据不做任何修改
    depth = cv::imread( "./data/depth.png", -1 );

    struct myslam::CAMERA_INTRINSIC_PARAMETERS camera;
    // 相机内参
    camera.fx = 518.0;
    camera.scale = 1000;
    camera.cx = 325.5;
    camera.cy = 253.5;
    camera.fy = 519.0;
    
    //自己的命名空间，防止重复的函数名相互影响
    //将2D图片转化为点云文件
    myslam::image2PointCloud(rgb, depth, camera);
    
    

    return 0;
}