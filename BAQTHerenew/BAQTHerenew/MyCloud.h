#ifndef MYCLOUD_H
#define MYCLOUD_H
#include "common_include.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>
#include "tbb/task_scheduler_init.h" 
#include "tbb/blocked_range.h"
#include "tbb/parallel_for.h"
#include "opencv2/core/eigen.hpp"
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


namespace amnl
{
	class MyCloud
	{
	public:
		MyCloud();
		~MyCloud();

		//pointcloud data
		PointCloudT::Ptr cloud;
		PointCloudT::Ptr trans_cloud;
		//file name
		std::string filename;
		//dir name
		std::string dirname;
		//show or not in UI
		bool visible;

		// access to the private members
		cv::Mat getRotationMatrix() { return rotationMatrix_; };
		cv::Mat getTranslationVector() { return translationVector_; };
		std::vector<cv::Point3f> getMarkers() { return markers_; };

		void setRotationMatrix(cv::Mat inputMatrix) { rotationMatrix_ = inputMatrix; };
		void setTranslationVector(cv::Mat inputMatrix) { translationVector_ = inputMatrix; };
		void setMarkers(std::vector<cv::Point3f> inputMarkers) { markers_ = inputMarkers; };
		void RTtransformation(cv::Mat& matR, cv::Mat& matT, BOOL bNormal = false);
	private:
		//markers
		std::vector<cv::Point3f> markers_;//这个存的是相机坐标系里的三维点，还没有存世界坐标系里的
		//std::vector<cv::Point3f> trans_markers;
		//for rotation to reference 
		cv::Mat rotationMatrix_;    //rot 3x3 CV_64FC1
									//for trans to reference 
		cv::Mat translationVector_;    //trans 3x1 CV_64FC1
	};
}
#endif // MYCLOUD_H