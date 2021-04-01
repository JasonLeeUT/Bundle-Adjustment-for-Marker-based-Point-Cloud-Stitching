#ifndef COMMON_INCLUDE_H
#define COMMON_INCLUDE_H

#if _MSC_VER >= 1600
#pragma execution_character_set("utf-8")
#endif

// define the commonly included file to avoid a long include list
// for Eigen
//#include <Eigen/Core>
//#include <Eigen/Geometry>
//using Eigen::Vector2d;
//using Eigen::Vector3d;

// for cv
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//using cv::Mat;

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/filters/voxel_grid.h> 
#include <pcl/filters/filter.h>             //滤波相关头文件
#include <pcl/features/normal_3d.h>         //法线特征头文件
#include <pcl/registration/icp.h>           //ICP类相关头文件
#include <pcl/registration/icp_nl.h>        //非线性ICP 相关头文件
#include <pcl/registration/transforms.h>      //变换矩阵类头文件
#include <boost/make_shared.hpp>              //boost指针相关头文件
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

// std 
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>
#include <tchar.h>
#include <stack>
#include <iterator>

// for use glog in windows ref:https://hpc.nih.gov/development/glog.html
#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <windows.h>

using namespace std;
 
#ifdef _DEBUG
#pragma comment(lib,"opencv_calib3d320d.lib")
#pragma comment(lib,"opencv_core320d.lib")
#pragma comment(lib,"opencv_highgui320d.lib")
#pragma comment(lib,"opencv_imgproc320d.lib")
#pragma comment(lib,"opencv_imgcodecs320d.lib")

#pragma comment(lib,"pcl_io_debug.lib")
#pragma comment(lib,"pcl_io_ply_debug.lib")
#else
#pragma comment(lib,"opencv_calib3d320.lib")
#pragma comment(lib,"opencv_core320.lib")
#pragma comment(lib,"opencv_highgui320.lib")
#pragma comment(lib,"opencv_imgproc320.lib")
#pragma comment(lib,"opencv_imgcodecs320.lib")
#endif //_DEBUG
#endif