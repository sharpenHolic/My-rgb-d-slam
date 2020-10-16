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

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


//在命名空间XGQ中封装两个函数，生成库文件，供其他文件调用
namespace XGQ {

    // 类型定义
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    // 相机内参结构，使用结构体的形式，更好的管理参数类文件，可以经常使用
    struct CAMERA_INTRINSIC_PARAMETERS {
        double cx, cy, fx, fy, scale;
    };

    //定义一个管理配置文件的类
    // 参数读取类
    class ParameterReader
    {
    public:

        //构造函数
        ParameterReader( string filename="./parameters.txt" )
        {
            ifstream fin( filename.c_str() );   //读取数据
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
    struct FRAME {
        cv::Mat rgb, depth; //该帧对应的彩色图与深度图
        cv::Mat desp;       //特征描述子
        vector<cv::KeyPoint> kp; //关键点
    };

    // PnP 结果
    struct RESULT_OF_PNP {
        cv::Mat rvec, tvec;
        int inliers;
    };


    /**
     * @brief image2PonitCloud 将rgb图转换为点云
     * @param[in] rgb
     * @param[in] depth
     * @param[in] camera
     * @return[out] PointCloud::Ptr
     */
    PointCloud::Ptr image2PointCloud(cv::Mat &rgb, cv::Mat &depth, CAMERA_INTRINSIC_PARAMETERS &camera);


    /**
     * @brief point2dTo3d 将单个点从图像坐标转换为空间坐标
     * @param point 3维点Point3f (u,v,d)
     * @param camera
     * @return
     */
    cv::Point3f point2dTo3d(cv::Point3f &point, CAMERA_INTRINSIC_PARAMETERS &camera);


    /**
     *
     * @param[in] frame
     */
    void computeKeyPointsAndDesp(FRAME &frame);


    /**
     *
     * @param[in] frame1 帧1
     * @param[in] frame2 帧2
     * @param[in] camera 相机内参
     * @return RESULT_OF_PNP
     */
    RESULT_OF_PNP estimateMotion(FRAME &frame1, FRAME &frame2, CAMERA_INTRINSIC_PARAMETERS &camera);

    //我写的，没有用结构体差距，很乱
    void PnpComputeRt(vector<cv::KeyPoint> kp1, vector<cv::KeyPoint> kp2, cv::Mat depth1, vector<cv::DMatch> goodMatches, CAMERA_INTRINSIC_PARAMETERS C, cv::Mat &rvec, cv::Mat &tvec);
};



