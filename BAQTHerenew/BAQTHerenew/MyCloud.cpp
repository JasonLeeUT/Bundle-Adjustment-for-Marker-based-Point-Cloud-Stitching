#include "MyCloud.h"

namespace amnl
{
	MyCloud::MyCloud()
	{
		visible = true;
		rotationMatrix_ = cv::Mat::eye(3, 3, CV_64FC1);
		translationVector_ = cv::Mat::zeros(3, 1, CV_64FC1);

	}


	MyCloud::~MyCloud()
	{
	}


	void MyCloud::RTtransformation(cv::Mat& matR, cv::Mat& matT, BOOL bNormal)
	{
		double* pdT = matT.ptr<double>(0);
		double* pdR = matR.ptr<double>(0);
		//Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> rotation_Matrix;
		//Eigen::Matrix3d rotation_Matrix(matR.data);
		//cv::cv2eigen(matR, rotation_Matrix);
		Eigen::Translation3f init_translation(pdT[0],pdT[1],pdT[2]);
		Eigen::Matrix4f translation = Eigen::Matrix4f::Identity();
		translation(0, 0) = pdR[0];
		translation(0, 1) = pdR[1];
		translation(0, 2) = pdR[2];
		translation(1, 0) = pdR[3];
		translation(1, 1) = pdR[4];
		translation(1, 2) = pdR[5];
		translation(2, 0) = pdR[6];
		translation(2, 1) = pdR[7];
		translation(2, 2) = pdR[8];
		translation(0, 3) = pdT[0];
		translation(1, 3) = pdT[1];
		translation(2, 3) = pdT[2];
		pcl::transformPointCloud(*cloud, *trans_cloud, translation);
	}
}

