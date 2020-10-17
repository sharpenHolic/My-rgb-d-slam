/*************************************************************************
	> File Name: Version4/src/slamBase.cpp
	> Author: Gaoqiang xiong
	> Mail: 1142008606@qq.com
    > 说明：为本slam框架提供实现功能的库
	> Created Time: 2020年10月14日
 ************************************************************************/


#include "slamBase.h"


/**
 * @brief image2PonitCloud      将rgb图转换为点云
 * @param[in] rgb               彩色图
 * @param[in] depth             深度图
 * @param[in] camera            相机内参数
 * @return[out] PointCloud::Ptr 指向点云的一个指针
 */
PointCloud::Ptr image2PointCloud(cv::Mat &rgb, cv::Mat &depth, CAMERA_INTRINSIC_PARAMETERS &camera)
{
    // 使用智能指针，创建一个空点云。这种指针用完会自动释放。
    PointCloud::Ptr _cloud(new PointCloud);
    // 遍历深度图
    for (int m = 0; m < depth.rows; m++)
        for (int n = 0; n < depth.cols; n++) {
            // 获取深度图中(m,n)处的值
            ushort _d = depth.ptr<ushort>(m)[n];
            // _d 可能没有值，若如此，跳过此点
            if (_d == 0)
                continue;
            // _d 存在值，则向点云增加一个点
            PointT _p;

            // 计算这个点的空间坐标
            _p.z = double(_d) / camera.scale;
            _p.x = (n - camera.cx) * _p.z / camera.fx;
            _p.y = (m - camera.cy) * _p.z / camera.fy;

            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            _p.b = rgb.ptr<uchar>(m)[n * 3];
            _p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
            _p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

            // 把p加入到点云中
            _cloud->points.push_back(_p);
        }
    // 设置并保存点云
    _cloud->height = 1;
    _cloud->width = _cloud->points.size();
    cout << "point cloud size = " << _cloud->points.size() << endl;
    _cloud->is_dense = false;
    //pcl::io::savePCDFile("./pointcloud.pcd", *cloud);
    // 清除数据并退出
    //cloud->points.clear();
    //cout << "Point cloud saved." << endl;
    return _cloud;
}


/**
 * @brief                   将单个点从图像坐标转换为空间坐标
 * @param[in] point         图像点(u,v)+一个相对深度d  ----> (u,v,d)
 * @param[in] camera        参数
 * @return cv::Point3f
 */
cv::Point3f point2dTo3d(cv::Point3f &point, CAMERA_INTRINSIC_PARAMETERS &camera)
{
    //初始化输出3d点的类型，以及为其开辟栈区内存空间
    cv::Point3f _p;

    //相机投影的公式
    //相机投影公式出问题了，少除了个camera.scale
    //_p.z = double(point.z);
    _p.z = double(point.z) / camera.scale;
    _p.x = (point.x - camera.cx) * _p.z / camera.fx;
    _p.y = (point.y - camera.cy) * _p.z / camera.fy;

    return _p;
}


/**
 * @brief               计算关键点和描述子
 * @param[in] frame     frame传入时应该具有rgb和depth信息，传出时有kp和descriptor数据
 */
void computeKeyPointsAndDesp(FRAME &frame)
{
    // 构建提取器，默认两者都为 ORB
    cv::Ptr<cv::FeatureDetector> _detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> _descriptor = cv::ORB::create();

    _detector->detect(frame.rgb, frame.kp);  //提取图像1关键点
    if (!frame.kp.data())
    {
        cout << "关键点提取失败，没有内容,Err() \n";
        return;
    }

    cout << "Key points of image: " << frame.kp.size() << endl;

    // 计算描述子
    _descriptor->compute(frame.rgb, frame.kp, frame.desp);

}


/**
 * @brief                   pnp：  3d - 2d 关系求解位姿变换
 * @param[in] frame1        帧1
 * @param[in] frame2        帧2
 * @param[in] camera        相机内参
 * @return RESULT_OF_PNP    pnp的结果，就是一个r 一个t，一定要分清楚，他们的坐标关系！
 */
RESULT_OF_PNP estimateMotion(FRAME &frame1, FRAME &frame2, CAMERA_INTRINSIC_PARAMETERS &camera)
{
    // 求解pnp
    RESULT_OF_PNP _res;
    // 匹配描述子
    vector<cv::DMatch> _matches;
    //用BruteForce-Hamming来计算匹配关系
    cv::Ptr<cv::DescriptorMatcher> _matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    //cv::BFMatcher matcher;
    _matcher->match(frame1.desp, frame2.desp, _matches);
    if (_matches.data() == 0) {
        cout << "matches里面没有内容" << endl;
    }
    cout << "Find total " << _matches.size() << " matches." << endl;
    vector<cv::DMatch> _goodMatches;
    double minDis = 9999;   //这个9999哪来的？傻逼，这是随便取的，取大点就可以了
    for (size_t i = 0; i < _matches.size(); i++)
    {
        if (_matches[i].distance < minDis)
        {
            minDis = _matches[i].distance;
        }
    }
    cout << "min dis = " << minDis << endl;

    for (size_t i = 0; i < _matches.size(); i++)
    {
        if (_matches[i].distance < 10 * minDis)
        {
            _goodMatches.push_back(_matches[i]);
        }
    }

    //goodmatch为零的时候没有处理。我在后面几讲中处理了，
    // 你可以参照着改一下。本讲的代码只是一个中间结果。
    cout<<"good matches: "<<_goodMatches.size()<<endl;

    if (_goodMatches.size() <= 5)
    {
        _res.inliers = -1;
        return _res;
    }

    // 第一个帧的三维点
    vector<cv::Point3f> _pts_obj;
    // 第二个帧的图像点
    vector<cv::Point2f> _pts_img;
    for (size_t i = 0; i < _goodMatches.size(); i++)
    {
        // query 是第一个, train 是第二个
        cv::Point2f _p = frame1.kp[_goodMatches[i].queryIdx].pt;
        // 获取d是要小心！x是向右的，y是向下的，所以y才是行，x是列！
        ushort _d = frame1.depth.ptr<ushort>(int(_p.y))[int(_p.x)];
        if (_d == 0)
        {
            continue;
        }

        _pts_img.push_back(cv::Point2f(frame2.kp[_goodMatches[i].trainIdx].pt));

        // 将图像点(u,v,d)转成相机帧下的(x,y,z)
        cv::Point3f _pt(_p.x, _p.y, _d);
        cv::Point3f _pd = point2dTo3d(_pt, camera);  //经过内参变化
        _pts_obj.push_back(_pd);
    }

    double _camera_matrix_data[3][3] = {
            {camera.fx, 0,         camera.cx},
            {0,         camera.fy, camera.cy},
            {0,         0,         1}
    };

    // 构建相机矩阵
    cv::Mat _cameraMatrix(3, 3, CV_64F, _camera_matrix_data);
    cv::Mat _inliers;

    //brings points from the model coordinate system to the camera coordinate system.
    //把3d点从自身坐标转化到图像坐标系下
    cv::solvePnPRansac(_pts_obj, _pts_img, _cameraMatrix, cv::Mat(), _res.rvec, _res.tvec, -1, 100, 8.0, 0.99,
                       _inliers);
    _res.inliers = _inliers.rows;
    cout << "inliers: " << _res.inliers << endl;
    cout << "R=" << _res.rvec << endl;
    cout << "t=" << _res.tvec << endl;

    return _res;
}


/**
 * @brief           将cv的旋转矢量与位移矢量转换为变换矩阵
 * @param[in] r     pnp求出的cv格式旋转向量
 * @param[in] t     pnp求出的平移向量
 * @return T        旋转矩阵
 */
Eigen::Isometry3d CV2Eigen(cv::Mat &rvec, cv::Mat &tvec)
{
    // 处理result
    // 将旋转向量转化为旋转矩阵
    cv::Mat _R;
    cv::Rodrigues( rvec, _R );
    Eigen::Matrix3d _r;
    for ( int i=0; i<3; i++ )
    {
        for ( int j=0; j<3; j++ )
        {
            _r(i,j) = _R.at<double>(i,j);
        }
    }

    // 将平移向量和旋转矩阵转换成变换矩阵
    Eigen::Isometry3d _T = Eigen::Isometry3d::Identity();
    Eigen::AngleAxisd _angle(_r);

    _T.rotate(_angle);    //按照angle来旋转
    _T(0,3) = tvec.at<double>(0,0);
    _T(1,3) = tvec.at<double>(0,1);
    _T(2,3) = tvec.at<double>(0,2);

    return _T;
}


/**
 * @brief               将新加入的帧转换为点云，然后将原始点云旋转到新加入点云的坐标系下，叠加后输出
 * @param[in] original  原始点云
 * @param[in] newFrame  新来的帧
 * @param[in] T         新来的帧以及它的位姿
 * @param[in] camera    相机内参
 * @return              将新来帧加到原始帧后的点云图像
 */
PointCloud::Ptr joinPointCloud( PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera )
{
    //新加入的帧转换为点云
    PointCloud::Ptr _newCloud = image2PointCloud( newFrame.rgb, newFrame.depth, camera );

    // 合并点云
    PointCloud::Ptr _output (new PointCloud());  //定义一个输出点云
    pcl::transformPointCloud( *original, *_output, T.matrix() ); //将原始点云通过T（T为上一帧到当前帧的变换）转换到新加入的点云坐标系下
    *_newCloud += *_output;   //简单叠加

    // Voxel grid 滤波降采样
    //这里为什么要添加滤波？
    ////???????????????
    static pcl::VoxelGrid<PointT> _voxel;
    static ParameterReader  _pd;
    double gridsize = atof( _pd.getData("voxel_grid").c_str() );
    _voxel.setLeafSize( gridsize, gridsize, gridsize );
    _voxel.setInputCloud( _newCloud );
    PointCloud::Ptr _tmp( new PointCloud() );
    _voxel.filter( *_tmp );
    return _tmp;
}


/**
 * @brief                   给定index，读取一帧数据
 * @param[in]   index       第几个图片：对应于数据集中的图片的序号
 * @param[in]   pd          传入一个读取参数的对象
 * @return                  帧的结构体
 */
FRAME readFrame( int index, ParameterReader& pd )
{
    FRAME _f;    //在栈区开辟内存，实例化一个Frame的实例化对象，用于返回
    string _rgbDir   =   pd.getData("rgb_dir");      //rgb图像的文件夹路径
    string _depthDir =   pd.getData("depth_dir");    //depth图像的文件夹路径

    string _rgbExt   =   pd.getData("rgb_extension");    //后缀.png
    string _depthExt =   pd.getData("depth_extension");  //后缀.png

    stringstream _ss;    //读取文件数据流
    _ss << _rgbDir << index << _rgbExt;  //_ss = _rgbDir + index + _rgbExt
    string filename;
    _ss >> filename;
    _f.rgb = cv::imread( filename ); //_rgbDir + index + _rgbExt

    _ss.clear();
    filename.clear();
    _ss << _depthDir << index << _depthExt;  //同上
    _ss >> filename;

    _f.depth = cv::imread( filename, -1 );

    _f.frameID = index;
    return _f;
}


/**
 * @brief                         检测是否为关键帧
 * @param[in]       f1            前一帧
 * @param[in]       f2            当前帧
 * @param[in,out]   opti          g2o的求解器
 * @param[in]       is_loops      是否回环的判断
 * @return                        判断结果，，枚举类型
 */
CHECK_RESULT checkKeyframes( FRAME& f1, FRAME& f2, g2o::SparseOptimizer& opti, bool is_loops)
{
    //读取文件参数
    static ParameterReader _pd;
    static int _min_inliers = atoi( _pd.getData("min_inliers").c_str() );
    static double _max_norm = atof( _pd.getData("max_norm").c_str() );
    static double _keyframe_threshold = atof( _pd.getData("keyframe_threshold").c_str() );
    static double _max_norm_lp = atof( _pd.getData("max_norm_lp").c_str() );
    static CAMERA_INTRINSIC_PARAMETERS _camera = getDefaultCamera(); //相机内参

    // 估计f1到f2的旋转r和平移t
    RESULT_OF_PNP _result = estimateMotion( f1, f2, _camera );


    //各种判断条件来约束keyframe
    if ( _result.inliers < _min_inliers ) //inliers不够，放弃该帧# 最小内点，min_inliers=5
    {
        return NOT_MATCHED;
    }

    // 计算运动范围是否太大
    double _norm = normofTransform(_result.rvec, _result.tvec);
    if ( is_loops == false )    //没有回环检测
    {
        if ( _norm >= _max_norm )//大于0.2就判断太远了
        {
            return TOO_FAR_AWAY; // too far away, may be error
        }
    }
    else    //有回环检测的话
    {
        if ( _norm >= _max_norm_lp)   //大于2，就说明运动范围远
        {
            return TOO_FAR_AWAY;
        }
    }

    if ( _norm <= _keyframe_threshold )   // 0.1
    {
        return TOO_CLOSE;// too adjacent frame
    }
    // 向g2o中增加这个顶点与上一帧联系的边
    // 顶点部分
    // 顶点只需设定id即可
    if (is_loops == false)
    {
        g2o::VertexSE3 *_v = new g2o::VertexSE3();
        _v->setId( f2.frameID );
        _v->setEstimate( Eigen::Isometry3d::Identity() );
        opti.addVertex(_v);
    }
    // 边部分
    g2o::EdgeSE3* _edge = new g2o::EdgeSE3();
    // 连接此边的两个顶点id
    _edge->setVertex( 0, opti.vertex(f1.frameID ));
    _edge->setVertex( 1, opti.vertex(f2.frameID ));
    _edge->setRobustKernel( new g2o::RobustKernelHuber() );
    // 信息矩阵
    Eigen::Matrix<double, 6, 6> _information = Eigen::Matrix< double, 6,6 >::Identity();
    // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
    // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
    // 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
    _information(0,0) = _information(1,1) = _information(2,2) = 100;
    _information(3,3) = _information(4,4) = _information(5,5) = 100;
    // 也可以将角度设大一些，表示对角度的估计更加准确
    _edge->setInformation( _information );
    // 边的估计即是pnp求解之结果
    Eigen::Isometry3d T = CV2Eigen( _result.rvec, _result.tvec );
    // edge->setMeasurement( T );

    _edge->setMeasurement( T.inverse() );
    // 将此边加入图中
    opti.addEdge(_edge);
    return KEYFRAME;
}


/**
 * @brief                           检测近距离的回环，检测当前帧与前五帧之间的关系，看是否需要跳过
 * @param[in]       frames          vector<关键帧>
 * @param[in]       currFrame       当前帧
 * @param[in,out]   opti            g2o求解器
 */
void checkNearbyLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti )
{

    static ParameterReader _pd;
    static int _nearby_loops = atoi( _pd.getData("nearby_loops").c_str() );//nearby_loops = 5

    // 就是把currFrame和 frames里末尾几个测一遍
    if ( frames.size() <= _nearby_loops )
    {
        // no enough keyframes, check everyone
        for (size_t i=0; i<frames.size(); i++)
        {
            checkKeyframes( frames[i], currFrame, opti, true );
        }
    }
    else
    {
        // 关键帧数量大于5帧，check the nearest ones，比较最近的五帧
        for (size_t i = frames.size() - _nearby_loops; i<frames.size(); i++)
        {
            checkKeyframes( frames[i], currFrame, opti, true );
        }
    }
}


/**
 * @brief                           随机检测回环
 * @param[in]       frames          vector<关键帧>
 * @param[in]       currFrame       当前帧
 * @param[in,out]   opti            g2o求解器
 */
void checkRandomLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti )
{
    static ParameterReader _pd;
    static int _random_loops = atoi( _pd.getData("random_loops").c_str() );//random_loops = 5
    srand( (unsigned int) time(NULL) );
    // 随机取一些帧进行检测

    if ( frames.size() <= _random_loops )
    {
        // no enough keyframes, check everyone
        for (size_t i=0; i<frames.size(); i++)
        {
            checkKeyframes( frames[i], currFrame, opti, true );
        }
    }
    else
    {
        // randomly check loops
        for (int i=0; i<_random_loops; i++)
        {
            int index = rand()%frames.size();
            checkKeyframes( frames[index], currFrame, opti, true );
        }
    }
}


/**
 * @brief                   度量运动的大小
 * @param[in]       rvec
 * @param[in]       tvec
 * @return
 */
double normofTransform( cv::Mat rvec, cv::Mat tvec )
{
    return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}



