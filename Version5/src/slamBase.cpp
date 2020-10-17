/*************************************************************************
	> File Name: Version4/src/slamBase.cpp
	> Author: Gaoqiang xiong
	> Mail: 1142008606@qq.com
    > 说明：为本slam框架提供实现功能的库
	> Created Time: 2020年10月9日
 ************************************************************************/


#include "slamBase.h"


/* *
 * @brief image2PointCloud函数，用于2D图片到3D点云图片的转换
 * @param[in]   rgb图片、深度图片、相机内参数
 * @param[out]  点云智能指针
 * @return 0; 成功执行
 * @return -1; 失败，读取图像
 */
PointCloud::Ptr image2PointCloud(cv::Mat &rgb, cv::Mat &depth, CAMERA_INTRINSIC_PARAMETERS &camera) {

    // 使用智能指针，创建一个空点云。这种指针用完会自动释放。
    PointCloud::Ptr _cloud(new PointCloud);
    // 遍历深度图
    for (int m = 0; m < depth.rows; m++)
        for (int n = 0; n < depth.cols; n++) {
            // 获取深度图中(m,n)处的值
            ushort _d = depth.ptr<ushort>(m)[n];
            // _d 可能没有值，若如此，跳过此点，回到循环26行
            if (_d == 0)
                continue;
            // _d 存在值，则向点云增加一个点
            PointT _p;

            // 计算这个点的空间坐标
            _p.z =  double(_d) / camera.scale;
            _p.x =  (n - camera.cx) * _p.z / camera.fx;
            _p.y =  (m - camera.cy) * _p.z / camera.fy;
            //_p.y = - (m - camera.cy) * _p.z / camera.fy;    //p.y换成-号，让点云为正的

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


cv::Point3f point2dTo3d(cv::Point3f &point, CAMERA_INTRINSIC_PARAMETERS &camera) {

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


void computeKeyPointsAndDesp(FRAME &frame) {

    // 构建提取器，默认两者都为 ORB
    cv::Ptr<cv::FeatureDetector> _detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> _descriptor = cv::ORB::create();

    _detector->detect(frame.rgb, frame.kp);  //提取图像1关键点
    if (!frame.kp.data()) {
        cout << "关键点提取失败，没有内容,Err() \n";
        return;
    }

    cout << "Key points of image: " << frame.kp.size() << endl;

    // 计算描述子
    _descriptor->compute(frame.rgb, frame.kp, frame.desp);

}


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
    for (size_t i = 0; i < _matches.size(); i++) {
        if (_matches[i].distance < minDis)
            minDis = _matches[i].distance;
    }
    cout << "min dis = " << minDis << endl;

    for (size_t i = 0; i < _matches.size(); i++) {
        if (_matches[i].distance < 10 * minDis)
            _goodMatches.push_back(_matches[i]);
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
    for (size_t i = 0; i < _goodMatches.size(); i++) {
        // query 是第一个, train 是第二个
        cv::Point2f _p = frame1.kp[_goodMatches[i].queryIdx].pt;
        // 获取d是要小心！x是向右的，y是向下的，所以y才是行，x是列！
        ushort _d = frame1.depth.ptr<ushort>(int(_p.y))[int(_p.x)];
        if (_d == 0)
            continue;
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


Eigen::Isometry3d CV2Eigen(cv::Mat &rvec, cv::Mat &tvec)
{

    // 处理result
    // 将旋转向量转化为旋转矩阵
    cv::Mat _R;
    cv::Rodrigues( rvec, _R );
    Eigen::Matrix3d _r;
    for ( int i=0; i<3; i++ )
        for ( int j=0; j<3; j++ )
            _r(i,j) = _R.at<double>(i,j);

    // 将平移向量和旋转矩阵转换成变换矩阵
    Eigen::Isometry3d _T = Eigen::Isometry3d::Identity();
    Eigen::AngleAxisd _angle(_r);

    _T.rotate(_angle);    //按照angle来旋转
    _T(0,3) = tvec.at<double>(0,0);
    _T(1,3) = tvec.at<double>(0,1);
    _T(2,3) = tvec.at<double>(0,2);

    return _T;
}

PointCloud::Ptr joinPointCloud( PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera )
{

    //新加入的帧转换为点云
    PointCloud::Ptr _newCloud = image2PointCloud( newFrame.rgb, newFrame.depth, camera );

    // 合并点云
    PointCloud::Ptr _output (new PointCloud());  //堆区开辟一个输出点云
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
 * @brief 读取当前帧的函数
 * @param index 当前帧的索引
 * @param pd 参数读取
 * @return FRAME 当前帧的rgb图像和depth图像
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
    return _f;
}

double normofTransform( cv::Mat rvec, cv::Mat tvec )
{
    return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}



