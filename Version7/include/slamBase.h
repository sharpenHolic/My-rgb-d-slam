# pragma once

// 各种头文件 
// C++标准库
#include <fstream>
#include <vector>
using namespace std;

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

//Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

//g2o
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>


// 类型定义
//点云3D点的定义
typedef pcl::PointXYZRGBA PointT;
//点云----由3D点组成的向量
typedef pcl::PointCloud<PointT> PointCloud;


// 相机内参结构，使用结构体的形式，更好的管理参数类文件，可以经常使用
struct CAMERA_INTRINSIC_PARAMETERS {
    double cx, cy, fx, fy, scale/*缩放尺度*/;
};


//定义一个管理配置文件的类
// 参数读取类
class ParameterReader
{
public:

    //构造函数,读取文件的相对路径
    ParameterReader( string filename="./config/parameters.txt" )
    {
        ifstream fin( filename.c_str() );   //读取文件
        if (!fin)
        {
            cerr<<"parameter file does not exist."<<endl;
            return;
        }
        while(!fin.eof())   //end of file？
        {
            string str;
            getline( fin, str );
            if (str[0] == '#')
            {
                // 以‘＃’开头的是注释
                continue;
            }

            int pos = str.find("=");    //find 找到返回位置，其余为false
            if (pos == -1)
                continue;
            string key = str.substr( 0, pos );  //复制子字符串，要求从指定位置开始，并具有指定的长度。如果没有指定长度_Count或_Count+_Off超出了源字符串的长度，则子字符串将延续到源字符串的结尾。
            string value = str.substr( pos+1, str.length() );
            data[key] = value;

            if ( !fin.good() )  //fin.good是判断文件是否打开的，如果返回真的话就是打开了，否则没有打开
                break;
        }
    }
    string getData( string key )
    {
        map<string, string>::iterator iter = data.find(key);
        if (iter == data.end())
        {
            cerr<<"Parameter name "<<key<<" not found!"<<endl;
            return string("NOT_FOUND");
        }
        return iter->second;
    }
public:
    map<string, string> data;
};


// 帧结构
struct FRAME
{
    int frameID;
    cv::Mat rgb, depth; //该帧对应的彩色图与深度图
    cv::Mat desp;       //特征描述子
    vector<cv::KeyPoint> kp; //关键点
};


// PnP 结果
struct RESULT_OF_PNP
{
    cv::Mat rvec/*旋转向量，使用的时候要转换为Eigen格式*/, tvec/*平移向量*/;
    int inliers;    //内点（就是自己定义的规则觉得比较好的点）数量
};


/**
 * @brief image2PonitCloud      将rgb图转换为点云
 * @param[in] rgb               彩色图
 * @param[in] depth             深度图
 * @param[in] camera            相机内参数
 * @return[out] PointCloud::Ptr 指向点云的一个指针
 */
PointCloud::Ptr image2PointCloud(cv::Mat &rgb, cv::Mat &depth, CAMERA_INTRINSIC_PARAMETERS &camera);


/**
 * @brief                   将单个点从图像坐标转换为空间坐标
 * @param[in] point         图像点(u,v)+一个相对深度d  ----> (u,v,d)
 * @param[in] camera        参数
 * @return cv::Point3f
 */
cv::Point3f point2dTo3d(cv::Point3f &point, CAMERA_INTRINSIC_PARAMETERS &camera);


/**
 * @brief               计算关键点和描述子
 * @param[in] frame     frame传入时应该具有rgb和depth信息，传出时有kp和descriptor数据
 */
void computeKeyPointsAndDesp(FRAME &frame);


/**
 * @brief                   pnp：  3d - 2d 关系求解位姿变换
 * @param[in] frame1        帧1
 * @param[in] frame2        帧2
 * @param[in] camera        相机内参
 * @return RESULT_OF_PNP    pnp的结果，就是一个r 一个t，一定要分清楚，他们的坐标关系！
 */
RESULT_OF_PNP estimateMotion(FRAME &frame1, FRAME &frame2, CAMERA_INTRINSIC_PARAMETERS &camera);


/**
 * @brief           将cv的旋转矢量与位移矢量转换为变换矩阵
 * @param[in] r     pnp求出的cv格式旋转向量
 * @param[in] t     pnp求出的平移向量
 * @return T        旋转矩阵
 */
Eigen::Isometry3d CV2Eigen(cv::Mat &rvec, cv::Mat &tvec);


/**
 * @brief               将新加入的帧转换为点云，然后将原始点云旋转到新加入点云的坐标系下，叠加后输出
 * @param[in] original  原始点云
 * @param[in] newFrame  新来的帧
 * @param[in] T         新来的帧以及它的位姿
 * @param[in] camera    相机内参
 * @return              将新来帧加到原始帧后的点云图像
 */
PointCloud::Ptr joinPointCloud( PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera );


/**
 * @brief       读取相机内参的函数
 * @return      相机参数
 */
inline static CAMERA_INTRINSIC_PARAMETERS getDefaultCamera()
{

    ParameterReader pd;
    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.fx = atof( pd.getData( "camera.fx" ).c_str());
    camera.fy = atof( pd.getData( "camera.fy" ).c_str());
    camera.cx = atof( pd.getData( "camera.cx" ).c_str());
    camera.cy = atof( pd.getData( "camera.cy" ).c_str());
    camera.scale = atof( pd.getData( "camera.scale" ).c_str() );
    return camera;
}


/**
 * @brief                   给定index，读取一帧数据
 * @param[in]   index       第几个图片：对应于数据集中的图片的序号
 * @param[in]   pd          传入一个读取参数的对象
 * @return                  帧的结构体
 */
FRAME readFrame( int index, ParameterReader& pd );


// 检测两个帧，结果定义
enum CHECK_RESULT {NOT_MATCHED=0, TOO_FAR_AWAY, TOO_CLOSE, KEYFRAME};


/**
 * @brief                         检测是否为关键帧
 * @param[in]       f1            前一帧
 * @param[in]       f2            当前帧
 * @param[in,out]   opti          g2o的求解器
 * @param[in]       is_loops      是否回环的判断
 * @return                        判断结果，，枚举类型
 */
CHECK_RESULT checkKeyframes( FRAME& f1, FRAME& f2, g2o::SparseOptimizer& opti, bool is_loops=false );


/**
 * @brief                           检测近距离的回环，检测当前帧与前五帧之间的关系，看是否需要跳过
 * @param[in]       frames          vector<关键帧>
 * @param[in]       currFrame       当前帧
 * @param[in,out]   opti            g2o求解器
 */
void checkNearbyLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti );


/**
 * @brief                           随机检测回环
 * @param[in]       frames          vector<关键帧>
 * @param[in]       currFrame       当前帧
 * @param[in,out]   opti            g2o求解器
 */
void checkRandomLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti );



/**
 * @brief                   度量运动的大小
 * @param[in]       rvec
 * @param[in]       tvec
 * @return
 */
double normofTransform( cv::Mat rvec, cv::Mat tvec );


//the following are UBUNTU/LINUX ONLY terminal color
//终端输出的颜色信息
#define RESET "\033[0m"
#define BLACK "\033[30m" /* Black */
#define RED "\033[31m" /* Red */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m" /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m" /* Cyan */
#define WHITE "\033[37m" /* White */
#define BOLDBLACK "\033[1m\033[30m" /* Bold Black */
#define BOLDRED "\033[1m\033[31m" /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m" /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m" /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m" /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m" /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m" /* Bold White */