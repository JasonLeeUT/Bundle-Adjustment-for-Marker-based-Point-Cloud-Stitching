#ifndef ELLIPSEDETECTION_H
#define ELLIPSEDETECTION_H
#include "common_include.h"
namespace amnl
{
	using std::stack;

	struct EdgePoints
	{
		double* position;
		int x;
		int y;
		EdgePoints* next;
	};

	struct Edge
	{
		EdgePoints* start;
		Edge* next;
	};


	struct EdgePoints_i
	{
		unsigned char* position;
		int x;
		int y;
		EdgePoints_i* next;
	};

	struct Edge_i
	{
		EdgePoints_i* start;
		Edge_i* next;
	};


	struct EllipsePoint
	{
		cv::Point2f Pt;
		float a;
		float b;
		float angle;

		EllipsePoint()
		{
			Pt = cv::Point2f(0.f, 0.f);
			a = 0;
			b = 0;
			angle = 0;
		};

		EllipsePoint(cv::Point2f Pt_in, float a_in, float b_in, float angle_in)
		{
			Pt = Pt_in;
			a = a_in;
			b = b_in;
			angle = angle_in;
		};
	};


#define AXIS_RATIO_THR 5
#define MAX_NAME_LENGTH 255
#define ELLIPSE_MIN_EDGE_POINT_NUM 12

#define ELLIPSE_MIN_EDGE_POINT_NUM3 6

#define AXIS_MIN_LENGTH 2.f
#define AXIS_MAX_ROTIO 3.f
#define EDGE_GRAD_THR 3.f
#define ELLIPSE_DEGREE_THR 0.075f
#define IS_CONCENTRIC_ELLIPSE 0

#define ADAPTIVE_THRESH_WINDOW_SIZE 25
#define ADAPTIVE_THRESH_C -10.f
#define BINARY_THRESH 40


	class EllipseDetection
	{
	public:
		typedef std::shared_ptr<EllipseDetection> Ptr;
		EllipseDetection();
		~EllipseDetection();

	public:
		void DetectEllipse(
			std::vector<EllipsePoint>& vectEllipses,
			cv::Mat pImg,
			double axis_thr = AXIS_MIN_LENGTH,
			double axisratio_thr = AXIS_MAX_ROTIO,
			double grad_thr = EDGE_GRAD_THR,
			double ellipse_thr = ELLIPSE_DEGREE_THR,
			bool bconcentricprior = IS_CONCENTRIC_ELLIPSE);

	private:

		void FittingEllipse2(
			std::vector<EllipsePoint>& vectEllipses,
			std::vector<std::vector<EdgePoints>>& vvectEPts,
			cv::Mat dx,
			cv::Mat dy,
			double axis_thr,
			double axisratio_thr,
			double ellipse_thr,
			bool bConcentricCircle);

		void DisplayElipses(
			cv::Mat img,
			std::vector<EllipsePoint> EllipsePts);

	private:
		double MulSum64f(cv::Mat src1, cv::Mat src2);

		void CutEdge(cv::Mat src, int iMargin = 4);

		void TribleGEMM64f(cv::Mat src1, cv::Mat src2, cv::Mat src3, cv::Mat dst);

		void SqrtMat64f(cv::Mat src, cv::Mat dst);

		void MatLeftDivision64f(cv::Mat M, cv::Mat B, cv::Mat mpts);

		void RebuildConic(cv::Mat src, cv::Mat dst);

		void DebuildConic(cv::Mat src, cv::Mat dst);

		void FindConnectedDomains2(cv::Mat mat, std::vector<std::vector<EdgePoints>>& vvectEPts, int iMinPixelNum = ELLIPSE_MIN_EDGE_POINT_NUM);


		bool m_bAutoGradThr;

	};
}
#endif // ELLIPSEDETECTION_H
