#include "TransformationCalculation.h"



TransformationCalculation::TransformationCalculation()
{
	//point to epipolar dist when 3D reconstruting
	m_fEpipolarDist = 2.f;
	m_Rocord = true;
	m_index = 1;
	//m_rootpath = QString("D:\\AMNL3D\\"); changed 10.4 jianfeng
	m_rootpath = QString("D:\\VSProject\\BAQT\\");
}
//类的构造矩阵

TransformationCalculation::~TransformationCalculation()
{
}
//类的析构矩阵

//入口函数
void TransformationCalculation::Calculation(MyCloud* pMycloud, cv::Mat left_image, cv::Mat right_image, 
	cv::Mat& matCameraMatrix1,
	cv::Mat& matLensDistortion1,
	cv::Mat& matR1,
	cv::Mat& matT1,
	cv::Mat& matCameraMatrix2,
	cv::Mat& matLensDistortion2,
	cv::Mat& matR2,
	cv::Mat& matT2, double& err, std::vector<cv::Point3f> m_totalmarkerPts)
{
	std::vector<cv::Point2f> Marker_centers1, Marker_centers2;
	std::vector<float> Marker_axisas1, Marker_axisas2;
	std::vector<float> Marker_axisbs1, Marker_axisbs2;
	std::vector<float> Marker_angles1, Marker_angles2;

	DetectEllipse2(left_image, Marker_centers1, Marker_axisas1, Marker_axisbs1, Marker_angles1);//找到左图中的椭圆圆心
	DetectEllipse2(right_image, Marker_centers2, Marker_axisas2, Marker_axisbs2, Marker_angles2);//找到右图中的椭圆圆心

	std::vector<cv::Point3f> Target_points; //marker点云
	Calculate3DPoints(Marker_centers1, Marker_centers2, matCameraMatrix1, matLensDistortion1, matR1, matT1, matCameraMatrix2, matLensDistortion2, matR2, matT2, Target_points, err, m_fEpipolarDist);//通过左右两边的点来计算对应的三维点
	pMycloud->setMarkers(Target_points); //记录下当前pose的mark值
	
	if (pMycloud->getMarkers().size()>=3)
	{
		cv::Mat mat_R = cv::Mat::eye(3, 3, CV_64FC1);
		cv::Mat mat_T = cv::Mat::zeros(3, 1, CV_64FC1);
		if (m_totalmarkerPts.size() == 0)
		{
			err = 0.5f;
			if (m_Rocord) //1：第一个矩阵
			{
				QString str;
				//str = m_rootpath + QString("REP\\AutoScanRT\\") + QString::number(m_index) + QString(".rep");
				str = QString("D:\\VSProject\\BA\\") + QString("REP\\AutoScanRT\\") + QString::number(m_index) + QString(".rep");
				SaveRT(str, mat_R, mat_T);
				m_index++;
			}
			
			pMycloud->RTtransformation(mat_R, mat_T);
			pMycloud->setRotationMatrix(mat_R);
			pMycloud->setTranslationVector(mat_T);
		}
		else
		{
		
			calculateRT(m_totalmarkerPts, Target_points, mat_R, mat_T, err);

			if (err < 1.0f)
			{
				if (m_Rocord)
				{
					QString str;
					//str = m_rootpath + QString("REP\\AutoScanRT\\") + QString::number(m_index) + QString(".rep");
					str = QString("D:\\VSProject\\BA\\") + QString("REP\\AutoScanRT\\") + QString::number(m_index) + QString(".rep");
					SaveRT(str, mat_R, mat_T);
					m_index++;
				}
				pMycloud->RTtransformation(mat_R, mat_T);
				pMycloud->setRotationMatrix(mat_R);
				pMycloud->setTranslationVector(mat_T);
			}
		}
	}
	
}

void TransformationCalculation::DetectEllipse2(cv::Mat image, std::vector<cv::Point2f>& vec_centers, std::vector<float>& vec_axisa, std::vector<float>& vec_axisb, std::vector<float>& vec_angle)
{
	std::vector<EllipsePoint> vectLEllipse;
	ellipsedetector->DetectEllipse(vectLEllipse,image);

	for (int i = 0; i < vectLEllipse.size(); i++)
	{
		vec_centers.push_back(vectLEllipse.at(i).Pt);
		vec_axisa.push_back(vectLEllipse.at(i).a);
		vec_axisb.push_back(vectLEllipse.at(i).b);
		vec_angle.push_back(vectLEllipse.at(i).angle);
	}
}

bool more(const int &m1, const int &m2)
{
	return m1 > m2;
}

void TransformationCalculation::Calculate3DPoints(std::vector<cv::Point2f>& left_imgPt, std::vector<cv::Point2f>& right_imgPt, 
	cv::Mat& matCameraMatrix1,
	cv::Mat& matLensDistortion1,
	cv::Mat& matR1,
	cv::Mat& matT1, //已知左右两点算出来的椭圆点，来找出对应点，并计算三维空间点的位置
	cv::Mat& matCameraMatrix2,
	cv::Mat& matLensDistortion2,
	cv::Mat& matR2,
	cv::Mat& matT2, 
	std::vector<cv::Point3f>& target_3DPt, double& err, float fDist)
{
	if (left_imgPt.size() < 3 || right_imgPt.size() < 3)
	{
		err = Error_sig;
		return;
	}

	cv::Mat img(2056,2464,CV_8UC3, cv::Scalar(0, 0, 0));
	//cv::Mat img1(2056, 2464, CV_8UC3, cv::Scalar(0, 0, 0));

	//pt1
	cv::Mat matImagePts1(left_imgPt.size(), 1, CV_32FC2, left_imgPt.data());
	cv::Mat matUndistImagePts1(left_imgPt.size(), 1, CV_32FC2);
	cv::undistortPoints(
		matImagePts1,
		matUndistImagePts1,
		matCameraMatrix1,
		matLensDistortion1,
		cv::Mat::eye(cv::Size(3, 3), CV_64FC1),
		matCameraMatrix1);  //marker点坐标矫正
	cv::Point point;
	//for (int i = 0; i < left_imgPt.size(); i++)
	//{
	//	float* l_1 = matUndistImagePts1.ptr<float>(i);
	//	point.x = l_1[0];
	//	point.y = l_1[1];
	//	cv::circle(img, point, 8, cv::Scalar(0, 0, 255),4);
	//}
	//cv::imwrite("img1.bmp", img);
	
	//pt2
	cv::Mat matImagePts2(right_imgPt.size(), 1, CV_32FC2, right_imgPt.data());
	cv::Mat matUndistImagePts2(right_imgPt.size(), 1, CV_32FC2);
	cv::undistortPoints(
		matImagePts2,
		matUndistImagePts2,
		matCameraMatrix2,
		matLensDistortion2,
		cv::Mat::eye(cv::Size(3, 3), CV_64FC1),
		matCameraMatrix2);

	//for (int i = 0; i < right_imgPt.size(); i++)
	//{
	//	float* l_1 = matUndistImagePts2.ptr<float>(i);
	//	point.x = l_1[0];
	//	point.y = l_1[1];
	//	cv::putText(img1, to_string(i), point, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 23, 0), 4, 8);
	//	cv::circle(img1, point, 8, cv::Scalar(0, 0, 255),4);
	//}

	cv::Mat matE(3, 3, CV_64FC1, cv::Scalar(0.0f));
	cv::Mat matF(3, 3, CV_64FC1, cv::Scalar(0.0f));
	GetFundamentalMatrix(
		matCameraMatrix1,
		matR1,
		matT1,
		matCameraMatrix2,
		matR2,
		matT2,
		matF,
		matE); //求基本矩阵
	cv::Mat L1(int(left_imgPt.size()), 1, CV_32FC3, cv::Scalar(0.0f));
	cv::computeCorrespondEpilines(matUndistImagePts1, 1, matF, L1);  //求极线

	std::vector<int> vectMatchedPtIndex1_tmp, vectMatchedPtIndex2_tmp;
	
	double a, b, c;
	cv::Point point1, point2;
	for (int i = 0; i<left_imgPt.size(); i++)
	{
		float* pL1 = L1.ptr<float>(i);
		a = pL1[0];
		b = pL1[1];
		c = pL1[2];
		point1.x = 0;
		point2.x = 2464;
		float k = -a / b;
		point1.y = -c / b;
		point2.y = k*point2.x - c / b;  //画图
		//cv::putText(img1, to_string(i), point1, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 23, 0), 4, 8);
		//cv::line(img1, point1, point2, cv::Scalar(0, 0, 255), 4, CV_AA);
		BOOL bFind = FALSE;
		int Index = 0;
		float DistThr = fDist;
		for (int j = 0; j<right_imgPt.size(); j++)
		{
			float* pfUndistPts2 = matUndistImagePts2.ptr<float>(j);
			cv::Point2f TempPt2D;
			TempPt2D.x = pfUndistPts2[0];
			TempPt2D.y = pfUndistPts2[1];
			float dist = fabs(a*TempPt2D.x + b*TempPt2D.y + c) / sqrt(a*a + b*b); //点到直线距离
			if (dist<DistThr)
			{
				DistThr = dist;
				Index = j;  //假如有多个点的时候，我们只选择最后一个点，二维图像极线匹配
				bFind = TRUE;
			}
		}

		if (bFind)
		{
			vectMatchedPtIndex1_tmp.push_back(i);
			vectMatchedPtIndex2_tmp.push_back(Index);
		}
	}
	//cv::imwrite("img2.bmp", img1);
	if (vectMatchedPtIndex1_tmp.size() == 0)
	{
		err = Error_sig;
		//TRACE(_T("Error: non matched points are found!"));
		return ;  //读下一个图
	}

	std::vector<cv::Point2f> vectMatchedPt1, vectMatchedPt2; //这个是挑选出来的点
	cv::Point2f Pt1, Pt2;
	for (int i = 0; i<vectMatchedPtIndex1_tmp.size(); i++)
	{
		Pt1 = left_imgPt.at(vectMatchedPtIndex1_tmp.at(i));
		vectMatchedPt1.push_back(Pt1); //保存坐标
		Pt2 = right_imgPt.at(vectMatchedPtIndex2_tmp.at(i));
		vectMatchedPt2.push_back(Pt2);
	}

	cv::Point2f Pt3, Pt4;
	vector<int> del_repeat;
	for (int i = 0; i < vectMatchedPt1.size()-1; i++)
	{
		Pt1 = vectMatchedPt1[i];
		Pt2 = vectMatchedPt2[i];
		for (int j = i+1; j < vectMatchedPt1.size(); j++)
		{
			Pt3 = vectMatchedPt1[j];
			Pt4 = vectMatchedPt2[j];
			if (i != j)
			{
				float dDist = sqrt((Pt3.x - Pt1.x)*(Pt3.x - Pt1.x) + (Pt3.y - Pt1.y)*(Pt3.y - Pt1.y));
				float dDist1 = sqrt((Pt4.x - Pt2.x)*(Pt4.x - Pt2.x) + (Pt4.y - Pt2.y)*(Pt4.y - Pt2.y));
				if (dDist < 1 || dDist1 < 1)
				{
					del_repeat.push_back(j);
				}
			}
		}// 挨个比较每个点对之间点的距离，如果有两个点相同，则说明一张图的两个点对应了另一张图中的同一个点，然后我们把序号小的留下来，序号大的那一对删掉
	}

	sort(del_repeat.begin(), del_repeat.end(), more);
	
	vector<int>::iterator it, it1;
	/*
	for (it = ++del_repeat.begin(); it != del_repeat.end();)
	{
		it1 = find(del_repeat.begin(), it, *it);    
		if (it1 != it)
			it = del_repeat.erase(it);
		else
			it++;
	}


	for (int i = 0; i < del_repeat.size(); i++)
	{
		vectMatchedPt1.erase(vectMatchedPt1.begin() + del_repeat[i]);
		vectMatchedPt2.erase(vectMatchedPt2.begin() + del_repeat[i]);
	}
	*/ //only work if del_repeat not empty
	if (del_repeat.size()>0) {
		for (it = ++del_repeat.begin(); it != del_repeat.end();)
		{
			it1 = find(del_repeat.begin(), it, *it);
			if (it1 != it)
				it = del_repeat.erase(it);
			else
				it++;
		}


		for (int i = 0; i < del_repeat.size(); i++)
		{
			vectMatchedPt1.erase(vectMatchedPt1.begin() + del_repeat[i]);
			vectMatchedPt2.erase(vectMatchedPt2.begin() + del_repeat[i]);
		}
	}


	/*QImage imge;
	imge.load("D:/AMNL3D/ScanImages/Camera1/2_Measurement_13.bmp");
	QPixmap pixL;
	pixL = QPixmap::fromImage(imge);
	QFont font("Arial", 20, QFont::Bold, true);
	font.setPointSize(50);
	QPainter p(&pixL);
	p.setFont(font);
	for (int i = 0; i < vectMatchedPt1.size(); i++)
	{
		point = vectMatchedPt1[i];
		p.setPen(QPen(Qt::blue, 6.0));
		p.drawEllipse(point.x, point.y, 4, 4);
		p.setPen(QPen(Qt::red, 5.0));
		p.drawText(QPoint(point.x, point.y), QString::number(i));
	}
	pixL.save("img3.png", "PNG");
	
	QImage imge1;
	imge1.load("D:/AMNL3D/ScanImages/Camera2/2_Measurement_13.bmp");
	QPixmap pixR;
	pixR = QPixmap::fromImage(imge1);
	font.setPointSize(50);
	QPainter q(&pixR);
	q.setFont(font);
	for (int i = 0; i < vectMatchedPt2.size(); i++)
	{
		point = vectMatchedPt2[i];
		q.setPen(QPen(Qt::blue, 6.0));
		q.drawEllipse(point.x, point.y, 4, 4);
		q.setPen(QPen(Qt::red, 5.0));
		q.drawText(QPoint(point.x, point.y), QString::number(i));
	}
	pixR.save("img4.png", "PNG");*/

	FILE* op = fopen("D:\\VSProject\\BA\\REP\\points\\23.txt", "w");

	for (int i = 0; i < vectMatchedPt1.size(); i++)
	{
		cv::Point3f targetPt; //这里应该就是左右两图里面找到的点的集合了，然后对应的就是对应的targetPt，所以其实这一步计算的是左右两图
		reconstructor->unprojectImage2World(vectMatchedPt1[i], vectMatchedPt2[i], matCameraMatrix1, matLensDistortion1,matR1,matT1,matCameraMatrix2,
			matLensDistortion2,matR2,matT2, targetPt);  //从左右两图重构三维点
		target_3DPt.push_back(targetPt);
		//fprintf(op,"%f %f %f %f %f\n", vectMatchedPt1[i].x, vectMatchedPt1[i].y, targetPt.x, targetPt.y, targetPt.z); //这里写的三维数据还是未转换前的
		fprintf(op, "%f %f\n", vectMatchedPt1[i].x, vectMatchedPt1[i].y);
	}
	fclose(op);

}

void TransformationCalculation::GetFundamentalMatrix(cv::Mat& matCameraMatrix1, cv::Mat& matR1, cv::Mat& matT1, cv::Mat& matCameraMatrix2,
	cv::Mat& matR2, cv::Mat& matT2, cv::Mat& matF, cv::Mat& matE)
{
	cv::Mat matR = matR2*matR1.t();
	cv::Mat matT = matT2 - matR*matT1;
	cv::Mat matT_inv_t(3, 3, CV_64FC1, cv::Scalar(0.0f));
	matT_inv_t.at<double>(0, 1) = -matT.at<double>(2, 0);
	matT_inv_t.at<double>(0, 2) = matT.at<double>(1, 0);
	matT_inv_t.at<double>(1, 0) = matT.at<double>(2, 0);
	matT_inv_t.at<double>(1, 2) = -matT.at<double>(0, 0);
	matT_inv_t.at<double>(2, 0) = -matT.at<double>(1, 0);
	matT_inv_t.at<double>(2, 1) = matT.at<double>(0, 0);
	matE = matT_inv_t*matR;
	matF = matCameraMatrix2.t().inv()*matE*matCameraMatrix1.inv();
}

void TransformationCalculation::SaveRT(QString strFilename, cv::Mat& matR, cv::Mat& matT)
{
	
	QByteArray ba = strFilename.toLatin1();
	char* ch;
	ch = ba.data();
	FILE* file = fopen(ch, "w");

	//FILE* file = fopen("E:\\VSProject\\BAQT\\REP\\AutoScanRT\\1.txt", "w");
	for (int i = 0; i < 3; i++)
	{
		double* pdR = matR.ptr<double>(i);
		fprintf(file, "%3.8f %3.8f %3.8f\n", pdR[0], pdR[1], pdR[2]);
	}
	double* pdT = matT.ptr<double>(0);  
	fprintf(file, "%3.8f %3.8f %3.8f\n", pdT[0], pdT[1], pdT[2]);
	fclose(file);
}
//////// 计算三维点和点云之间的对应关系
void TransformationCalculation::calculateRT(std::vector<cv::Point3f>& vec_referPts, std::vector<cv::Point3f>& vec_targetPts, cv::Mat& matR, cv::Mat& matT, double& err, float f_Threshold)
{
	//第一个参数是总的地图中点的数量，第二个参数是新计算出来的点的数量，然后对他们进行配对，并计算出相应的RT矩阵来，可问题在于，你有配对的，
	//但你也有未配对的点啊，这之中的关系如何去捋清楚，是个问题
	if (vec_referPts.size() < 3 || vec_targetPts.size() < 3)
	{
		//TRACE(_T("Error: number of input points is less than 3!"));
		err = Error_sig;
		return;
	}
	//Fix dist matrix
	cv::Mat matReferPtsDist(vec_referPts.size(), vec_referPts.size(), CV_32FC1, cv::Scalar(0.0f));
	for (UINT i = 0; i<vec_referPts.size(); i++)
	{
		float *pf = matReferPtsDist.ptr<float>(i);
		cv::Point3f pt1 = vec_referPts.at(i);
		for (UINT j = 0; j < vec_referPts.size(); j++)
		{
			cv::Point3f pt2 = vec_referPts.at(j);
			float dDist = sqrt((pt2.x - pt1.x)*(pt2.x - pt1.x) + (pt2.y - pt1.y)*(pt2.y - pt1.y) + (pt2.z - pt1.z)*(pt2.z - pt1.z));
			pf[j] = dDist;
		}
	}

	//Trans dist matrix
	cv::Mat matTargetPtsDist(vec_targetPts.size(), vec_targetPts.size(), CV_32FC1, cv::Scalar(0.0f));
	for (UINT i = 0; i<vec_targetPts.size(); i++)
	{
		float *pf = matTargetPtsDist.ptr<float>(i);
		cv::Point3f pt1 = vec_targetPts.at(i);
		for (UINT j = 0; j<vec_targetPts.size(); j++)
		{
			cv::Point3f pt2 = vec_targetPts.at(j);
			float dDist = sqrt((pt2.x - pt1.x)*(pt2.x - pt1.x) + (pt2.y - pt1.y)*(pt2.y - pt1.y) + (pt2.z - pt1.z)*(pt2.z - pt1.z));
			pf[j] = dDist;
		}
	}

	//label matrix
	cv::Mat matLabel(vec_targetPts.size(), vec_referPts.size(), CV_32SC1, cv::Scalar(0));

	for (UINT i = 0; i<vec_targetPts.size(); i++)
	{
		float* pfTransDist = matTargetPtsDist.ptr<float>(i);
		for (UINT j = 0; j<vec_targetPts.size(); j++)
		{
			float fTransDist = pfTransDist[j];
			for (UINT m = 0; m<vec_referPts.size(); m++)
			{
				float* pfFixDist = matReferPtsDist.ptr<float>(m);
				for (UINT n = 0; n<vec_referPts.size(); n++)
				{
					float fFixDist = pfFixDist[n];
					if (( /*MY_EQ(fTransDist,fFixDist)*/fabs(fTransDist - fFixDist) < f_Threshold)
						&& (!/*MY_EQ(fFixDist, 0.0f)*/fabs(fFixDist) < f_Threshold))
						matLabel.at<int>(i, m) += 1;
				}
			}
		}
	}

	//Match pts from dist
	std::vector<cv::Point3f> vectMatchReferPts, vectMatchTargetPts;
	for (UINT i = 0; i<vec_targetPts.size(); i++)
	{
		int* piLabelValue = matLabel.ptr<int>(i);
		int iMax = 0;
		int iMaxIndex = 0;
		for (UINT j = 0; j<vec_referPts.size(); j++)
		{
			int iLabelValue = piLabelValue[j];
			if (iLabelValue > iMax)
			{
				iMax = iLabelValue;
				iMaxIndex = j;
			}
		}
		if (iMax >= 2)
		{
			vectMatchTargetPts.push_back(vec_targetPts.at(i)); //计算得到的相机参考系中的三维点
			vectMatchReferPts.push_back(vec_referPts.at(iMaxIndex)); //地图中的三维点
		}
	}

	if (vectMatchTargetPts.size()<3)
	{
		err = Error_sig;
		return;
	}
	MatchPts(vectMatchReferPts, vectMatchTargetPts, matR, matT, err);
}

void TransformationCalculation::MatchPts(std::vector<cv::Point3f>& vec_referPts, std::vector<cv::Point3f>& vec_targetPts, cv::Mat& matR, cv::Mat& matT, double& Err, float f_Threshold)
{
	if (vec_referPts.size() != vec_targetPts.size())
	{
		//TRACE(_T("Error: number of input points are not equal!"));
		Err = Error_sig;
		return;
	}
	else if (vec_referPts.size()<3 || vec_targetPts.size()<3)
	{
		//TRACE(_T("Error: number of input points is less than 3!"));
		Err = Error_sig;
		return;
	}
	if (matR.rows != 3 || matR.cols != 3 || matR.type() != CV_64FC1)
	{
		matR.create(3, 3, CV_64FC1);
	}
	if (matT.rows != 3 || matT.cols != 1 || matR.type() != CV_64FC1)
	{
		matT.create(3, 1, CV_64FC1);
	}

	UINT uiPointNum = vec_referPts.size();

	//establish dist matrix
	cv::Mat matDistRefer(uiPointNum, uiPointNum, CV_32FC1, cv::Scalar(0.0f));
	cv::Mat matDistTarget(uiPointNum, uiPointNum, CV_32FC1, cv::Scalar(0.0f));

	for (UINT i = 0; i<uiPointNum; i++)
	{
		cv::Point3f FixPt = vec_referPts.at(i);
		cv::Point3f TransPt = vec_targetPts.at(i);

		float* pfFixDist = matDistRefer.ptr<float>(i);
		float* pfTranDist = matDistTarget.ptr<float>(i);

		for (UINT j = 0; j<uiPointNum; j++)
		{
			if (j != i)
			{
				cv::Point3f FixPt_ex = vec_referPts.at(j);
				cv::Point3f TransPt_ex = vec_targetPts.at(j);

				pfFixDist[j] = sqrt(
					(FixPt.x - FixPt_ex.x)*(FixPt.x - FixPt_ex.x) +
					(FixPt.y - FixPt_ex.y)*(FixPt.y - FixPt_ex.y) +
					(FixPt.z - FixPt_ex.z)*(FixPt.z - FixPt_ex.z));

				pfTranDist[j] = sqrt(
					(TransPt.x - TransPt_ex.x)*(TransPt.x - TransPt_ex.x) +
					(TransPt.y - TransPt_ex.y)*(TransPt.y - TransPt_ex.y) +
					(TransPt.z - TransPt_ex.z)*(TransPt.z - TransPt_ex.z));
			}
		}
	}

	//select most possible
	cv::Mat matDistDiff(uiPointNum, uiPointNum, CV_32FC1, cv::Scalar(0.0f));
	matDistDiff = matDistRefer - matDistTarget;
	std::vector<cv::Point3f> vectFilterReferPts, vectFilterTargetPts;
	for (UINT i = 0; i < uiPointNum; i++)
	{
		float* pfDistDiff = matDistDiff.ptr<float>(i);
		UINT uiInRange = 0;

		for (UINT j = 0; j < uiPointNum; j++)
		{
			if (pfDistDiff[j] < /*MY_EPS*/f_Threshold)
				uiInRange++;
		}

		if (uiInRange > 2)
		{
			vectFilterReferPts.push_back(vec_referPts.at(i));
			vectFilterTargetPts.push_back(vec_targetPts.at(i));
		}
	}

	//obtain pose
	uiPointNum = vectFilterReferPts.size();
	if (3 == uiPointNum)
	{
		PoseSVD(
			vectFilterReferPts,
			vectFilterTargetPts,
			matR,
			matT,
			Err);
	}
	else if (3 < uiPointNum)
	{
		int iIterNum = RASC_ITER_NUM;
		int iIterIndex = 1;
		int iMinPtsNum = 3;
		int iRestNumThr = 3;

		double dErr_temp = Error_sig;
		Err = Error_sig;

		cv::Mat matR_temp = cv::Mat::eye(3, 3, CV_64FC1);
		cv::Mat matT_temp = cv::Mat::zeros(3, 1, CV_64FC1);

		while (iIterIndex<iIterNum)
		{
			std::vector<UINT> vectRandNum;
			GetRand(uiPointNum, vectRandNum);

			std::vector<cv::Point3f> vectFixPts_cur;
			std::vector<cv::Point3f> vectTransPts_cur;
			for (UINT i = 0; i < iMinPtsNum; i++)
			{
				vectFixPts_cur.push_back(vectFilterReferPts.at(vectRandNum.at(i)));
				vectTransPts_cur.push_back(vectFilterTargetPts.at(vectRandNum.at(i)));
			}

			PoseSVD(
				vectFixPts_cur,
				vectTransPts_cur,
				matR_temp,
				matT_temp,
				dErr_temp);
			if (dErr_temp < Err)
			{
				Err = dErr_temp;
				matR = matR_temp.clone();
				matT = matT_temp.clone();
			}

			if (dErr_temp < INIT_REG_ERR)
			{
				for (UINT i = iMinPtsNum; i<uiPointNum; i++)
				{
					cv::Mat matPt_Fix(3, 1, CV_64FC1);
					cv::Mat matPt_Trans(3, 1, CV_64FC1);
					cv::Mat matDistErr(3, 1, CV_64FC1);
					UINT uiPtIndex = vectRandNum.at(i);
					matPt_Fix.at<double>(0, 0) = vectFilterReferPts.at(uiPtIndex).x;
					matPt_Fix.at<double>(1, 0) = vectFilterReferPts.at(uiPtIndex).y;
					matPt_Fix.at<double>(2, 0) = vectFilterReferPts.at(uiPtIndex).z;
					matPt_Trans.at<double>(0, 0) = vectFilterTargetPts.at(uiPtIndex).x;
					matPt_Trans.at<double>(1, 0) = vectFilterTargetPts.at(uiPtIndex).y;
					matPt_Trans.at<double>(2, 0) = vectFilterTargetPts.at(uiPtIndex).z;
					matDistErr = matPt_Fix - matR_temp*matPt_Trans - matT_temp;
					double dDistErr = cv::norm(matDistErr);
					if (dDistErr < PT_ERR_THR)
					{
						vectFixPts_cur.push_back(vectFilterReferPts.at(uiPtIndex));
						vectTransPts_cur.push_back(vectFilterTargetPts.at(uiPtIndex));
					}
				}

				if (vectFixPts_cur.size() > iRestNumThr)
				{
					PoseSVD(
						vectFixPts_cur,
						vectTransPts_cur,
						matR_temp,
						matT_temp,
						dErr_temp);

					Err = dErr_temp;
					matR = matR_temp.clone();
					matT = matT_temp.clone();

					if (dErr_temp < REG_AVG_ERR)
						break;
				}
			}

			iIterIndex++;
		}
	}
}

void TransformationCalculation::PoseSVD(std::vector<cv::Point3f>& vec_referPts, std::vector<cv::Point3f>& vec_targetPts, cv::Mat& matR, cv::Mat& matT, double& Err)
{
	if (vec_referPts.size() != vec_targetPts.size())
	{
		//TRACE(_T("Error: number of input points are not equal!"));
		Err = Error_sig;
		return;
	}
	else if (vec_referPts.size()<3 || vec_targetPts.size()<3)
	{
		//TRACE(_T("Error: umber of input points is less than 3!"));
		Err = Error_sig;
		return;
	}
	if (matR.rows != 3 || matR.cols != 3 || matR.type() != CV_64FC1)
	{
		matR.create(3, 3, CV_64FC1);
	}
	if (matT.rows != 3 || matT.cols != 1 || matR.type() != CV_64FC1)
	{
		matT.create(3, 1, CV_64FC1);
	}

	UINT uiPointNum = vec_referPts.size();

	cv::Point3f FixPts_Centroid;
	FixPts_Centroid.x = 0.0f;
	FixPts_Centroid.y = 0.0f;
	FixPts_Centroid.z = 0.0f;
	cv::Point3f TransPts_Centroid;
	TransPts_Centroid.x = 0.0f;
	TransPts_Centroid.y = 0.0f;
	TransPts_Centroid.z = 0.0f;

	for (int i = 0; i<uiPointNum; i++)
	{
		FixPts_Centroid.x = FixPts_Centroid.x + vec_referPts[i].x;
		FixPts_Centroid.y = FixPts_Centroid.y + vec_referPts[i].y;
		FixPts_Centroid.z = FixPts_Centroid.z + vec_referPts[i].z;
		TransPts_Centroid.x = TransPts_Centroid.x + vec_targetPts[i].x;
		TransPts_Centroid.y = TransPts_Centroid.y + vec_targetPts[i].y;
		TransPts_Centroid.z = TransPts_Centroid.z + vec_targetPts[i].z;
	}

	TransPts_Centroid.x = TransPts_Centroid.x / float(uiPointNum);
	TransPts_Centroid.y = TransPts_Centroid.y / float(uiPointNum);
	TransPts_Centroid.z = TransPts_Centroid.z / float(uiPointNum);

	FixPts_Centroid.x = FixPts_Centroid.x / float(uiPointNum);
	FixPts_Centroid.y = FixPts_Centroid.y / float(uiPointNum);
	FixPts_Centroid.z = FixPts_Centroid.z / float(uiPointNum);

	//establish the H matrix
	cv::Mat matFixCoVar(vec_referPts.size(), 3, CV_64FC1);
	cv::Mat matTransCoVar(vec_targetPts.size(), 3, CV_64FC1);

	for (UINT i = 0; i<uiPointNum; i++)
	{
		double* pdFix = matFixCoVar.ptr<double>(i);
		double* pdTrans = matTransCoVar.ptr<double>(i);

		pdFix[0] = vec_referPts[i].x - FixPts_Centroid.x;
		pdFix[1] = vec_referPts[i].y - FixPts_Centroid.y;
		pdFix[2] = vec_referPts[i].z - FixPts_Centroid.z;

		pdTrans[0] = vec_targetPts[i].x - TransPts_Centroid.x;
		pdTrans[1] = vec_targetPts[i].y - TransPts_Centroid.y;
		pdTrans[2] = vec_targetPts[i].z - TransPts_Centroid.z;
	}

	cv::Mat matH(3, 3, CV_64FC1);
	matH = matFixCoVar.t()*matTransCoVar;

	//H matrix svd decompose
	cv::Mat matU, matSDiag, matV;
	cv::SVD::compute(matH, matSDiag, matU, matV);

	//determine rotation matrix
	cv::Mat matS = cv::Mat::eye(3, 3, CV_64FC1);
	if (cv::determinant(matU)*cv::determinant(matV.t()) < 0)
	{
		matS.at<double>(2, 2) = -1.0f;
	}
	matR = matU*matS*matV;

	//determine rotation matrix
	cv::Mat matReferPts(3, uiPointNum, CV_64FC1);
	cv::Mat matTargetPts(3, uiPointNum, CV_64FC1);

	double* pd_F_x = matReferPts.ptr<double>(0);
	double* pd_F_y = matReferPts.ptr<double>(1);
	double* pd_F_z = matReferPts.ptr<double>(2);
	double* pd_T_x = matTargetPts.ptr<double>(0);
	double* pd_T_y = matTargetPts.ptr<double>(1);
	double* pd_T_z = matTargetPts.ptr<double>(2);
	for (UINT i = 0; i<uiPointNum; i++)
	{
		pd_F_x[i] = vec_referPts[i].x;
		pd_F_y[i] = vec_referPts[i].y;
		pd_F_z[i] = vec_referPts[i].z;

		pd_T_x[i] = vec_targetPts[i].x;
		pd_T_y[i] = vec_targetPts[i].y;
		pd_T_z[i] = vec_targetPts[i].z;
	}

	cv::Mat matTargetPts_after(3, uiPointNum, CV_64FC1);
	matTargetPts_after = matR*matTargetPts;

	cv::Mat matT_temp(3, uiPointNum, CV_64FC1);
	matT_temp = matReferPts - matTargetPts_after;
	matT = matT_temp*cv::Mat::ones(uiPointNum, 1, CV_64FC1)*cv::Mat(1, 1, CV_64FC1, cv::Scalar(1.0f / double(uiPointNum)));

	//determine average error
	cv::Mat matErr(3, uiPointNum, CV_64FC1);
	matErr = matReferPts - matTargetPts_after - matT*cv::Mat::ones(1, uiPointNum, CV_64FC1);

	double* pd_Err_x = matErr.ptr<double>(0);
	double* pd_Err_y = matErr.ptr<double>(1);
	double* pd_Err_z = matErr.ptr<double>(2);

	double d_x, d_y, d_z;
	Err = 0.0f;
	for (UINT i = 0; i<uiPointNum; i++)
	{
		d_x = pd_Err_x[i];
		d_y = pd_Err_y[i];
		d_z = pd_Err_z[i];
		Err += sqrt(d_x*d_x + d_y*d_y + d_z*d_z);
	}
	Err = Err / double(uiPointNum);
}

int TransformationCalculation::Random_generate(int a, int b)
{
	int area = 0;
	int ret = 0;
	//
	area = b - a + 1;
	ret = (int)(rand()*area / (1.0 * RAND_MAX) + a);
	return ret;
}

void TransformationCalculation::GetRand(UINT n, std::vector<UINT>& vectRandNum)
{
	int i;
	int p;
	int tmp;

	int* arr = new int[n];
	for (i = 0; i<n; i++)
	{
		arr[i] = i;
	}

	for (i = n - 1; i>0; i--)
	{
		p = Random_generate(0, i);
		tmp = arr[p];
		arr[p] = arr[i];
		arr[i] = tmp;
	}

	vectRandNum.clear();
	for (int i = 0; i<n; i++)
	{
		vectRandNum.push_back(arr[i]);
	}

	delete[]arr;
}

void TransformationCalculation::Addmarkers(std::vector<cv::Point3f>& vectRefPts3D, //markers
	std::vector<cv::Point3f>& m_totalmarkerPts,
	cv::Mat m_R,
	cv::Mat m_T,
	float fError)
{
	if (vectRefPts3D.size() < 3)
	{
		return;
	}

	std::vector<cv::Point3f> TargetPoints;
	for (int i = 0; i < vectRefPts3D.size(); i++)
	{
		TargetPoints.push_back(vectRefPts3D.at(i)); //markers
	}
	vectRefPts3D.clear();

	//�Ѳο���ϲ���һ���µĲο���
	cv::Point3f temp, temp1, temp2;
	double* pdR = (double*)m_R.data;
	double* pdT = (double*)m_T.data;
	FILE* op = fopen("D:\\VSProject\\BA\\REP\\points\\23.txt", "r");
	FILE* ob = fopen("D:\\VSProject\\BA\\REP\\points\\21.txt", "w");
	/*
	if (op == NULL)
	{
		printf("Open file 23 failed.\n");
	}
	else
	{
		printf("Open file 23 succeed.\n");
	}
	if (ob == NULL)
	{
		printf("Open file 21 failed.\n");
	}
	else
	{
		printf("Open file 21 succeed.\n");
	}
	*/
	double obs[2];
	unsigned short nump =0 ;
	for (int i = 0; i<TargetPoints.size(); i++)
	{ // 可能有重复计算了，感觉，之前找好了
		temp = TargetPoints.at(i);
		temp1.x = pdR[0] * temp.x + pdR[1] * temp.y + pdR[2] * temp.z + pdT[0];
		temp1.y = pdR[3] * temp.x + pdR[4] * temp.y + pdR[5] * temp.z + pdT[1];
		temp1.z = pdR[6] * temp.x + pdR[7] * temp.y + pdR[8] * temp.z + pdT[2];

		//���� temp1�����е�m_totalTargets�ľ��룬���С����ֵ ��˵�����иñ�־�㲻�ϲ�������õ�Ϊ�µı�־�㣬����
		float minDist = 100.0f;  //
		int minIndex = -1;
		for (int nIndex = 0; nIndex<m_totalmarkerPts.size(); nIndex++)
		{
			temp2 = m_totalmarkerPts.at(nIndex);
			float dist = sqrt((temp2.x - temp1.x)*(temp2.x - temp1.x) +
				(temp2.y - temp1.y)*(temp2.y - temp1.y) +
				(temp2.z - temp1.z)*(temp2.z - temp1.z));
			if (dist<minDist)
			{
				minDist = dist;
				minIndex = nIndex; //找的是距离最小的点，但是我觉得，是不是这个距离设置有点大
			}
		}
		fscanf(op,"%lf %lf", &obs[0], &obs[1]);

		if (minDist>fError)
		{
			vectRefPts3D.push_back(temp1); //这个push回markers 这个点是一个新的点，那么就是要放进去，如果这个点是一个旧点，我们就取它之前计算出来的指

			m_totalmarkerPts.push_back(temp1); //这个往m-total里面添数
			//nump = m_totalmarkerPts.size() + 1;
			nump = m_totalmarkerPts.size()-1;
		}
		else
		{
			vectRefPts3D.push_back(m_totalmarkerPts.at(minIndex));
			nump = minIndex;
		}
		fprintf(ob,"%f %f %hu\n", obs[0], obs[1], nump);
	}

	fclose(op);
	fclose(ob);
	//return;
}