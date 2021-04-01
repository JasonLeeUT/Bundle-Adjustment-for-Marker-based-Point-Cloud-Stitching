#pragma once
#include "MyCloud.h"
#include <QtCore/QString> //用到了
//#include <QPainter>
#include "ellipsedetection.h"
#include "reconstruction.h"
//#include <QTime>
#define EPS_thr 0.2
#define Error_sig 100.f
#define INIT_REG_ERR 1.f
#define RASC_ITER_NUM 100
#define PT_ERR_THR 0.5f
#define REG_AVG_ERR 0.2f
#define REG_ERR_TOR 1.f
using namespace amnl;
class TransformationCalculation
{
public:
	TransformationCalculation();
	~TransformationCalculation();

	void Calculation(MyCloud* pMycloud, cv::Mat left_image, cv::Mat right_image, 
		cv::Mat& matCameraMatrix1,
		cv::Mat& matLensDistortion1,
		cv::Mat& matR1,
		cv::Mat& matT1,
		cv::Mat& matCameraMatrix2,
		cv::Mat& matLensDistortion2,
		cv::Mat& matR2,
		cv::Mat& matT2, double& err, std::vector<cv::Point3f> m_totalmarkerPts); //入口函数

	void DetectEllipse2(cv::Mat image, std::vector<cv::Point2f>& vec_centers, std::vector<float>& vec_axisa, std::vector<float>& vec_axisb, std::vector<float>& vec_angle);
	//找椭圆
	void Calculate3DPoints(std::vector<cv::Point2f>& left_imgPt, std::vector<cv::Point2f>& right_imgPt, cv::Mat& matCameraMatrix1,
		cv::Mat& matLensDistortion1,
		cv::Mat& matR1,
		cv::Mat& matT1,
		cv::Mat& matCameraMatrix2,
		cv::Mat& matLensDistortion2,
		cv::Mat& matR2,
		cv::Mat& matT2, std::vector<cv::Point3f>& target_3DPt, double& err, float fDist);
	//根据左右两张图片来计算三维点的坐标
	void SaveRT(QString strFilename, cv::Mat& matR, cv::Mat& matT);
	//存储RT矩阵
	void calculateRT(std::vector<cv::Point3f>& vec_referPts, std::vector<cv::Point3f>& vec_targetPts, cv::Mat& matR, cv::Mat& matT, double& err, float f_Threshold = EPS_thr);
	//根据对应点来计算RT矩阵
	void MatchPts(std::vector<cv::Point3f>& vec_referPts, std::vector<cv::Point3f>& vec_targetPts, cv::Mat& matR, cv::Mat& matT, double& Err, float f_Threshold = EPS_thr);

	void PoseSVD(std::vector<cv::Point3f>& vec_referPts, std::vector<cv::Point3f>& vec_targetPts, cv::Mat& matR, cv::Mat& matT, double& Err);

	int Random_generate(int a, int b);
	//产生随机数
	void GetRand(UINT n, std::vector<UINT>& vectRandNum);
	void GetFundamentalMatrix(
		cv::Mat& matCameraMatrix1,
		cv::Mat& matR1,
		cv::Mat& matT1,
		cv::Mat& matCameraMatrix2,
		cv::Mat& matR2,
		cv::Mat& matT2,
		cv::Mat& matF,
		cv::Mat& matE);

	void Addmarkers(std::vector<cv::Point3f>& vectRefPts3D,
		std::vector<cv::Point3f>& m_totalmarkerPts,
		cv::Mat m_R,
		cv::Mat m_T,
		float fError = REG_ERR_TOR);

private:
	Reconstruction::Ptr reconstructor;
	QString m_rootpath;
	bool m_Rocord;
	int m_index;
	QString time_cost = "0";
	EllipseDetection::Ptr ellipsedetector;

	//point to epipolar distance when 3D reconstruting
	float m_fEpipolarDist;
};

