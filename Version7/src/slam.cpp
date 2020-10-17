/*************************************************************************
	> File Name: version7/src/slam.cpp
	> Author: Gaoqiang Xiong
	> Mail: 1142008606@qq.com
	> Created Time: 2020年10月14日 星期六 15时35分42秒
    * add g2o slam end to visual odometry
    * add keyframe and simple loop closure
 ************************************************************************/

#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;

#include "slamBase.h"

// 把g2o的定义放到前面
typedef g2o::BlockSolver_6_3 SlamBlockSolver; 
typedef g2o::LinearSolverEigen< SlamBlockSolver::PoseMatrixType > SlamLinearSolver; 

int main( int argc, char** argv )
{
    // 前面部分和vo是一样的
    ParameterReader pd;
    int startIndex  =   atoi( pd.getData( "start_index" ).c_str() );
    int endIndex    =   atoi( pd.getData( "end_index"   ).c_str() );

    // 所有的关键帧都放在了这里
    vector< FRAME > keyframes; 
    // initialize
    cout<<"Initializing ..."<<endl;
    int currIndex = startIndex; // 当前索引为currIndex
    FRAME currFrame = readFrame( currIndex, pd ); // 上一帧数据

    CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();
    computeKeyPointsAndDesp( currFrame );
    PointCloud::Ptr cloud = image2PointCloud( currFrame.rgb, currFrame.depth, camera );
    
    /******************************* 
    // 新增:有关g2o的初始化
    *******************************/
    // 初始化求解器
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering( false );
    SlamBlockSolver* blockSolver = new SlamBlockSolver(unique_ptr<SlamBlockSolver::LinearSolverType> (linearSolver));
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(unique_ptr<g2o::Solver>(blockSolver));

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
    
    keyframes.push_back( currFrame );

    //keyframe_threshold=0.1
    double keyframe_threshold = atof( pd.getData("keyframe_threshold").c_str() );
    bool check_loop_closure = pd.getData("check_loop_closure")==string("yes");  //true
    
    for ( currIndex=startIndex+1; currIndex<endIndex; currIndex++ )
    {
        cout<<"Reading files "<<currIndex<<endl;
        FRAME currFrame = readFrame( currIndex,pd ); // 读取currFrame
        computeKeyPointsAndDesp( currFrame ); //提取特征
        CHECK_RESULT result = checkKeyframes( keyframes.back()/*keyframe的最后一个*/, currFrame, globalOptimizer ); //匹配该帧与keyframes里最后一帧
        switch (result) // 根据匹配结果不同采取不同策略
        {
        case NOT_MATCHED:
            //没匹配上，直接跳过
            cout<<RED"Not enough inliers."<<endl;
            break;
        case TOO_FAR_AWAY:
            // 太近了，也直接跳
            cout<<RED"Too far away, may be an error."<<endl;
            break;
        case TOO_CLOSE:
            // 太远了，可能出错了
            cout<<RESET"Too close, not a keyframe"<<endl;
            break;
        case KEYFRAME:
            cout<<GREEN"This is a new keyframe"<<endl;
            // 不远不近，刚好
            /**
             * This is important!!
             * This is important!!
             * This is important!!
             * (very important so I've said three times!)
             */
            // 检测回环
            if (check_loop_closure)
            {
                //检测当前帧与前五帧之间的关系，看是否需要跳过
                checkNearbyLoops( keyframes, currFrame, globalOptimizer );
                //随机检测5帧与当前帧的关系
                checkRandomLoops( keyframes, currFrame, globalOptimizer );
            }
            keyframes.push_back( currFrame );
            
            break;
        default:
            break;
        }
        
    }

    // 优化
    cout<<RESET"optimizing pose graph, vertices: "<<globalOptimizer.vertices().size()<<endl;
    globalOptimizer.save("./result_before.g2o");
    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize( 100 ); //可以指定优化步数
    globalOptimizer.save( "./result_after.g2o" );
    cout<<"Optimization done."<<endl;

    // 拼接点云地图
    cout<<"saving the point cloud map..."<<endl;
    PointCloud::Ptr output ( new PointCloud() ); //全局地图
    PointCloud::Ptr tmp ( new PointCloud() );

    pcl::VoxelGrid<PointT> voxel; // 网格滤波器，调整地图分辨率
    pcl::PassThrough<PointT> pass; // z方向区间滤波器，由于rgbd相机的有效深度区间有限，把太远的去掉
    pass.setFilterFieldName("z");
    pass.setFilterLimits( 0.0, 4.0 ); //4m以上就不要了

    double gridsize = atof( pd.getData( "voxel_grid" ).c_str() ); //分辨图可以在parameters.txt里调
    voxel.setLeafSize( gridsize, gridsize, gridsize );

    for (size_t i=0; i<keyframes.size(); i++)
    {
        // 从g2o里取出一帧
        g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(globalOptimizer.vertex( keyframes[i].frameID ));
        Eigen::Isometry3d pose = vertex->estimate(); //该帧优化后的位姿
        PointCloud::Ptr newCloud = image2PointCloud( keyframes[i].rgb, keyframes[i].depth, camera ); //转成点云
        // 以下是滤波
        voxel.setInputCloud( newCloud );
        voxel.filter( *tmp );
        pass.setInputCloud( tmp );
        pass.filter( *newCloud );
        // 把点云变换后加入全局地图中
        pcl::transformPointCloud( *newCloud, *tmp, pose.matrix() );
        *output += *tmp;
        tmp->clear();
        newCloud->clear();
    }

    voxel.setInputCloud( output );
    voxel.filter( *tmp );
    //存储
    pcl::io::savePCDFile( "./data/result.pcd", *tmp );
    
    cout<<"Final map is saved."<<endl;
    return 0;
}



