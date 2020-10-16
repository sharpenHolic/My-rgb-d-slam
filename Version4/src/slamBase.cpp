/*************************************************************************
	> File Name: Version4/src/slamBase.cpp
	> Author: Gaoqiang xiong
	> Mail: 1142008606@qq.com
    > 说明：为本slam框架提供实现功能的库
	> Created Time: 2020年10月9日
 ************************************************************************/

#include "slamBase.h"


//使用命名空间为XGQ，这个名字应该取的更好，暂时取这个名字
//改为myslam吧
namespace myslam
{

    /* *
     * @brief image2PointCloud函数，用于2D图片到3D点云图片的转换
     * @param[in]   rgb图片、深度图片、相机内参数
     * @param[out]  点云智能指针
     * @return 0; 成功执行
     * @return -1; 失败，读取图像
     */
    PointCloud::Ptr image2PointCloud(cv::Mat &rgb, cv::Mat &depth, CAMERA_INTRINSIC_PARAMETERS &camera) {

        // 使用智能指针，创建一个空点云。这种指针用完会自动释放。
        PointCloud::Ptr cloud(new PointCloud);
        // 遍历深度图
        for (int m = 0; m < depth.rows; m++)
            for (int n = 0; n < depth.cols; n++) {
                // 获取深度图中(m,n)处的值
                ushort d = depth.ptr<ushort>(m)[n];
                // d 可能没有值，若如此，跳过此点
                if (d == 0)
                    continue;
                // d 存在值，则向点云增加一个点
                PointT p;

                // 计算这个点的空间坐标
                p.z = double(d) / camera.scale;
                p.x = (n - camera.cx) * p.z / camera.fx;
                p.y = (m - camera.cy) * p.z / camera.fy;

                // 从rgb图像中获取它的颜色
                // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
                p.b = rgb.ptr<uchar>(m)[n * 3];
                p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
                p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

                // 把p加入到点云中
                cloud->points.push_back(p);
            }
        // 设置并保存点云
        cloud->height = 1;
        cloud->width = cloud->points.size();
        cout << "point cloud size = " << cloud->points.size() << endl;
        cloud->is_dense = false;
        //pcl::io::savePCDFile("./pointcloud.pcd", *cloud);
        // 清除数据并退出
        //cloud->points.clear();
        //cout << "Point cloud saved." << endl;
        return cloud;
    }


    cv::Point3f point2dTo3d(cv::Point3f &point, CAMERA_INTRINSIC_PARAMETERS &camera) {

        //初始化输出3d点的类型，以及为其开辟栈区内存空间
        cv::Point3f p;

        //相机投影的公式
        //相机投影公式出问题了，少除了个camera.scale
        //p.z = double(point.z);
        p.z = double(point.z) / camera.scale;
        p.x = (point.x - camera.cx) * p.z / camera.fx;
        p.y = (point.y - camera.cy) * p.z / camera.fy;

        return p;
    }


    void computeKeyPointsAndDesp(FRAME &frame) {

        // 构建提取器，默认两者都为 ORB
        //cv::ORB::create(这里面有参数可选)
        cv::Ptr<cv::FeatureDetector> _detector = cv::ORB::create();
        cv::Ptr<cv::DescriptorExtractor> _descriptor = cv::ORB::create();

        _detector->detect(frame.rgb, frame.kp);  //提取图像1关键点
        //判断关键点是否提取成功
        if (!frame.kp.data()) {
            cout << "关键点提取失败，没有内容,Err() \n";
            return;
        }

        cout << "Key points of image: " << frame.kp.size() << endl;

        // 计算描述子
        _descriptor->compute(frame.rgb, frame.kp, frame.desp);
        //判断描述子是否提取成功
        if(!frame.desp.data){
            cout << "关键点提取失败，没有内容,Err() \n";
            return;
        }

    }


    RESULT_OF_PNP estimateMotion(FRAME &frame1, FRAME &frame2, CAMERA_INTRINSIC_PARAMETERS &camera)
    {

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
        // 求解pnp
        RESULT_OF_PNP res;
        cv::solvePnPRansac(_pts_obj, _pts_img, _cameraMatrix, cv::Mat(), res.rvec, res.tvec, -1, 100, 8.0, 0.99,
                           _inliers);
        res.inliers = _inliers.rows;
        cout << "inliers: " << res.inliers << endl;
        //cout << "R=" << res.rvec << endl;
        //cout << "t=" << res.tvec << endl;

        return res;
    }

    // estimateMotion 计算两个帧之间的运动
// 输入：帧1和帧2
// 输出：rvec 和 tvec
    RESULT_OF_PNP estimateMotion_gaobo( FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera )
    {
        static ParameterReader pd;
        vector< cv::DMatch > matches;
        cv::BFMatcher matcher;
        matcher.match( frame1.desp, frame2.desp, matches );

        cout<<"find total "<<matches.size()<<" matches."<<endl;
        vector< cv::DMatch > goodMatches;
        double minDis = 9999;
        double good_match_threshold = atof( pd.getData( "good_match_threshold" ).c_str() );
        for ( size_t i=0; i<matches.size(); i++ )
        {
            if ( matches[i].distance < minDis )
                minDis = matches[i].distance;
        }

        for ( size_t i=0; i<matches.size(); i++ )
        {
            if (matches[i].distance < good_match_threshold*minDis)
                goodMatches.push_back( matches[i] );
        }

        cout<<"good matches: "<<goodMatches.size()<<endl;
        // 第一个帧的三维点
        vector<cv::Point3f> pts_obj;
        // 第二个帧的图像点
        vector< cv::Point2f > pts_img;

        // 相机内参
        for (size_t i=0; i<goodMatches.size(); i++)
        {
            // query 是第一个, train 是第二个
            cv::Point2f p = frame1.kp[goodMatches[i].queryIdx].pt;
            // 获取d是要小心！x是向右的，y是向下的，所以y才是行，x是列！
            ushort d = frame1.depth.ptr<ushort>( int(p.y) )[ int(p.x) ];
            if (d == 0)
                continue;
            pts_img.push_back( cv::Point2f( frame2.kp[goodMatches[i].trainIdx].pt ) );

            // 将(u,v,d)转成(x,y,z)
            cv::Point3f pt ( p.x, p.y, d );
            cv::Point3f pd = point2dTo3d( pt, camera );
            pts_obj.push_back( pd );
        }

        double camera_matrix_data[3][3] = {
                {camera.fx, 0, camera.cx},
                {0, camera.fy, camera.cy},
                {0, 0, 1}
        };

        cout<<"solving pnp"<<endl;
        // 构建相机矩阵
        cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
        cv::Mat rvec, tvec, inliers;
        // 求解pnp
        cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 8.0, 0.99, inliers );

        RESULT_OF_PNP result;
        result.rvec = rvec;
        result.tvec = tvec;
        result.inliers = inliers.rows;

        return result;
    }

};




