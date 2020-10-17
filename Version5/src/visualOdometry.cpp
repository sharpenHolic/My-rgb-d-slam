/*************************************************************************
	> File Name: Version5/src/visualOdometry.cpp
	> Author: Gaoqiang Xiong
	> Mail: 1142008606@qq.com
	> Created Time: 2020年10月11日
 ************************************************************************/

#include <iostream>
#include <fstream>
#include <sstream>
//using namespace myslam;

#include "slamBase.h"

int main( int argc, char** argv )
{

    ParameterReader pd;
    int startIndex  =   atoi( pd.getData( "start_index" ).c_str() );
    int endIndex    =   atoi( pd.getData( "end_index"   ).c_str() );

    // initialize
    cout<<"Initializing ..."<<endl;
    int currIndex = startIndex; // 当前索引为currIndex
    FRAME lastFrame = readFrame( currIndex, pd ); // 定为上一帧数据

    // 我们总是在比较currFrame和lastFrame
    CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();    //构造一个相机内参数
    computeKeyPointsAndDesp( lastFrame );   //计算上一帧帧的关键点和描述子

    //将当前帧恢复成点云，并且返回
    //这里创建第一帧点云为初始点云，方便后面的点云添加进来
    PointCloud::Ptr cloud = image2PointCloud( lastFrame.rgb, lastFrame.depth, camera );

    //实例化一个叫“viewer”点云可视化对象---执行完出来一个白框框
    pcl::visualization::CloudViewer viewer("viewer");

    // 是否显示点云
    bool visualize = pd.getData("visualize_pointcloud")==string("yes");

    //最少内点
    int min_inliers = atoi( pd.getData("min_inliers").c_str() );

    // 最大运动误差
    double max_norm = atof( pd.getData("max_norm").c_str() );

    //从第二帧开始，循环遍历到末尾
    for ( currIndex = startIndex + 1; currIndex < endIndex; currIndex++ )
    {

        cout<<"Reading files "<<currIndex<<endl;

        FRAME currFrame = readFrame( currIndex,pd ); // 读取currFrame（Frame中只有rgb和depth）
        computeKeyPointsAndDesp( currFrame );   //计算特帧点和描述子（Frame中加上kp和desp）

        // 比较currFrame 和 lastFrame
        //计算上一帧和当前帧的R和t（放在结构体RESULT_OF_PNP实例化的对象result中），这里的R和t是第一帧转向第二帧
        RESULT_OF_PNP result = estimateMotion( lastFrame, currFrame, camera );
        if ( result.inliers < min_inliers ) //inliers不够，放弃该帧，goodmatches为0，返回的inliers为-1
            continue;

        // 判断计算运动范围是否太大
        //这里的范数判断还不太清楚
        double norm = normofTransform(result.rvec, result.tvec);
        cout<<"norm = "<<norm<<endl;
        if ( norm >= max_norm )
            continue;

        //将结果转换为变换矩阵T
        Eigen::Isometry3d T = CV2Eigen( result.rvec /*旋转向量*/, result.tvec /*平移向量*/);
        cout<<"T="<<T.matrix()<<endl;

        //封装的函数：添加当前帧到之前的点云之中
        cloud = joinPointCloud( cloud, currFrame, T, camera );

        //定义是否需要可视化
        if ( visualize == true )
            viewer.showCloud( cloud );  //显示

        lastFrame = currFrame;  //当前帧变为上一帧//重复循环
    }

    pcl::io::savePCDFile( "data/result.pcd", *cloud );
    return 0;
}


