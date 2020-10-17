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

//g2o的头文件
#include <g2o/types/slam3d/types_slam3d.h> //顶点类型
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

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
    //这里创建第一帧，方便后面的点云添加进来
    PointCloud::Ptr cloud = image2PointCloud( lastFrame.rgb, lastFrame.depth, camera );
    
    pcl::visualization::CloudViewer viewer("viewer");

    // 是否显示点云
    bool visualize = pd.getData("visualize_pointcloud")==string("yes");

    //最少内点
    int min_inliers = atoi( pd.getData("min_inliers").c_str() );

    // 最大运动误差
    double max_norm = atof( pd.getData("max_norm").c_str() );


    /*******************************
    // 新增:有关g2o的初始化
    *******************************/

    //第一步，构建一个求解器：globalOptimizer
    // 选择优化方法
    typedef g2o::BlockSolver_6_3 SlamBlockSolver;// solver for BA/3D SLAM,每个误差项优化变量纬度为6，误差值维度为3
    typedef g2o::LinearSolverCSparse< SlamBlockSolver::PoseMatrixType > SlamLinearSolver;   //线性求解器类型
    // 初始化求解器
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering( false );
    SlamBlockSolver* blockSolver = new SlamBlockSolver(unique_ptr<SlamBlockSolver::LinearSolverType> (linearSolver));
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(unique_ptr<g2o::Solver>(blockSolver));
    //SlamBlockSolver* blockSolver = new SlamBlockSolver( linearSolver );   //原版代码，可能是高博写错了
    //g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( blockSolver );
    g2o::SparseOptimizer globalOptimizer;  // 最后用的就是这个东东
    globalOptimizer.setAlgorithm( solver );
    // 不要输出调试信息
    globalOptimizer.setVerbose( false );

    // 向globalOptimizer增加第一个顶点
    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId( currIndex );
    v->setEstimate( Eigen::Isometry3d::Identity() ); //估计为单位矩阵
    v->setFixed( true ); //第一个顶点固定，不用优化
    globalOptimizer.addVertex( v );

    int lastIndex = currIndex; // 上一帧的id


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
        //cloud = joinPointCloud( cloud, currFrame, T, camera );

        // 向g2o中增加这个顶点与上一帧联系的边
        // 顶点部分
        // 顶点只需设定id即可
        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId( currIndex );
        v->setEstimate( Eigen::Isometry3d::Identity() );
        globalOptimizer.addVertex(v);


        // 边部分
        g2o::EdgeSE3* edge = new g2o::EdgeSE3();
        // 连接此边的两个顶点id
        edge->vertices() [0] = globalOptimizer.vertex( lastIndex );
        edge->vertices() [1] = globalOptimizer.vertex( currIndex );
        // 信息矩阵
        Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
        // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
        // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
        // 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
        information(0,0) = information(1,1) = information(2,2) = 100;
        information(3,3) = information(4,4) = information(5,5) = 100;
        // 也可以将角度设大一些，表示对角度的估计更加准确
        edge->setInformation( information );
        // 边的估计即是pnp求解之结果
        edge->setMeasurement( T );
        // 将此边加入图中
        globalOptimizer.addEdge(edge);

        lastFrame = currFrame;
        lastIndex = currIndex;

    }

    // pcl::io::savePCDFile( "data/result.pcd", *cloud );

    // 优化所有边
    cout<<"optimizing pose graph, vertices: "<<globalOptimizer.vertices().size()<<endl;
    globalOptimizer.save("./data/result_before.g2o");
    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize( 100 ); //可以指定优化步数
    globalOptimizer.save( "./data/result_after.g2o" );
    cout<<"Optimization done."<<endl;

    globalOptimizer.clear();

    return 0;
}


