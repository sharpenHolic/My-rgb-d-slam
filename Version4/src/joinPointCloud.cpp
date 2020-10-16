/*************************************************************************
	> File Name: Version4/src/jointPointCloud.cpp
	> Author: Gaoqiang Xiong
	> Mail: 1142008606@qq.com
	> Created Time: 2020年10月9日
 ************************************************************************/

#include<iostream>
using namespace std;

#include "slamBase.h"

#include <opencv2/core/eigen.hpp>

#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

// Eigen !
#include <Eigen/Core>
#include <Eigen/Geometry>

int main( int argc, char** argv )
{
    //本节要拼合data中的两对图像
    myslam::ParameterReader pd; //参数读取器的实例化对象
    // 声明两个帧，FRAME结构请见include/slamBase.h
    myslam::FRAME frame1, frame2;   //用帧结构体创建两帧图片


    //读取图像
    frame1.rgb = cv::imread( "./data/rgb1.png" );
    frame1.depth = cv::imread( "./data/depth1.png", -1);
    frame2.rgb = cv::imread( "./data/rgb2.png" );
    frame2.depth = cv::imread( "./data/depth2.png", -1 );
    //判断读取的图像是否正常
    if(frame1.rgb.data == 0 || frame2.rgb.data == 0
            || frame1.depth.data == 0 || frame2.depth.data == 0)
    {
        cout << "Load pictures errs...." << endl;
        return -1;
    }


    // 提取特征并计算描述子
    cout<<"extracting features"<<endl;
    computeKeyPointsAndDesp(frame1);    //计算frame1描述子，默认的ORB，可以进函数改
    computeKeyPointsAndDesp(frame2);


    // 相机内参
    myslam::CAMERA_INTRINSIC_PARAMETERS camera;
    camera.fx = atof( pd.getData( "camera.fx" ).c_str());   //atof:Convert a string to a floating-point number
    camera.fy = atof( pd.getData( "camera.fy" ).c_str());
    camera.cx = atof( pd.getData( "camera.cx" ).c_str());
    camera.cy = atof( pd.getData( "camera.cy" ).c_str());
    camera.scale = atof( pd.getData( "camera.scale" ).c_str() );


    // 求解pnp
    //求出来的是第一帧(Frame1)三维点在第一帧的坐标系下，经过怎样的变换(result)到第二帧的坐标系下
    cout<<"solving pnp"<<endl;
    //自己封装的函数：：利用PNP求解相对位姿，输入两帧的结构，相机内参；返回RESULT_OF_PNP结构体
    myslam::RESULT_OF_PNP result = estimateMotion( frame1, frame2, camera );
    cout<<result.rvec<<endl<<result.tvec<<endl;


    // 处理result
    // 将旋转向量转化为旋转矩阵
    cv::Mat R;
    cv::Rodrigues( result.rvec, R );    //罗德里格斯公式，转换为旋转矩阵R
    cout << "罗德里格斯公式转换后的R = \n" << R << endl;
    Eigen::Matrix3d r;
    cv::cv2eigen(R, r);     //转换为Eigen形式


    // 将平移向量和旋转矩阵转换成变换矩阵
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    Eigen::AngleAxisd angle(r);     //构造一个旋转向量
    cout << "angle = \n" << angle.matrix() << endl; //angle.matrix() 和 r 相等
    cout<<"translation"<<endl;
    //T = angle;
    T.rotate(angle);    //构造变换矩阵T-----参考《视觉SLAM14讲》p63
    T(0,3) = result.tvec.at<double>(0,0);
    T(1,3) = result.tvec.at<double>(0,1);
    T(2,3) = result.tvec.at<double>(0,2);
    cout << "变换矩阵 T = \n" << T.matrix() << endl;


    // 转换点云
    cout<<"converting image to clouds"<<endl;
    //将rgb和depth转化为点云
    myslam::PointCloud::Ptr cloud1 = image2PointCloud( frame1.rgb, frame1.depth, camera );
    myslam::PointCloud::Ptr cloud2 = image2PointCloud( frame2.rgb, frame2.depth, camera );


    // 合并点云
    cout<<"combining clouds"<<endl;
    myslam::PointCloud::Ptr output (new myslam::PointCloud());
    //这里的意思为：T左乘 *cloud1 变换到第二帧坐标系下
    pcl::transformPointCloud( *cloud1, *output, T.matrix() );  //将第一个点云旋转
    *output += *cloud2;     //与第二帧坐标系下点云做叠加
    pcl::io::savePCDFile("data/result.pcd", *output);
    cout<<"Final result saved."<<endl;


    //显示点云
    pcl::visualization::CloudViewer viewer( "viewer" );
    viewer.showCloud( output );
    while( !viewer.wasStopped() )
    {

    }
    return 0;
}
