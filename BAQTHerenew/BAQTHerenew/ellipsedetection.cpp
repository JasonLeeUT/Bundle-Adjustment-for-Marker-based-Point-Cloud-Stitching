#include "ellipsedetection.h"

namespace amnl
{
	double PI = 3.141592653589793;

	EllipseDetection::EllipseDetection()
	{
		m_bAutoGradThr = true;
	}


	EllipseDetection::~EllipseDetection()
	{
	}

	void EllipseDetection::DetectEllipse(
		std::vector<EllipsePoint>& vectEllipses,
		cv::Mat pImg,
		double axis_thr,
		double axisratio_thr,
		double grad_thr,
		double ellipse_thr,
		bool bconcentricprior)
	{
		if (vectEllipses.size() > 0)
			vectEllipses.clear();

		//create image

		cv::Mat img;
		cv::Mat img2;
		img = pImg.clone();
		img2 = pImg.clone();

		//load images
		if (img.empty())
		{
			printf("Error: Can't Read Image!!");
			abort();
		}

		if (img.channels() == 3)
		{
			//rgb to gray
			cvtColor(img, img, CV_RGB2GRAY);
		}

		img.convertTo(img, CV_64FC1);

		//guassian convolution
		double d[5] = { 0.2707, 0.6065, 0, -0.6065, -0.2707 };
		double g[5] = { 0.1353, 0.6065, 1, 0.6065, 0.1353 };
		cv::Mat dm = cv::Mat(1, 5, CV_64FC1, d);
		cv::Mat gm = cv::Mat(1, 5, CV_64FC1, g);
		cv::Mat dtranspose = cv::Mat(5, 1, CV_64FC1);
		cv::Mat gtranspose = cv::Mat(5, 1, CV_64FC1);
		cv::transpose(dm, dtranspose);//转置dm并计入dtransopose;
		cv::transpose(gm, gtranspose);
		cv::Mat opxm = cv::Mat(5, 5, CV_64FC1);
		cv::Mat opym = cv::Mat(5, 5, CV_64FC1);
		opxm = gtranspose*dm;
		opym = dtranspose*gm;
		cv::Mat dx = cv::Mat(img.rows, img.cols, CV_64FC1);
		cv::Mat dy = cv::Mat(img.rows, img.cols, CV_64FC1);
		cv::filter2D(img, dx, -1, opxm);//卷积
		cv::filter2D(img, dy, -1, opym);

		cv::Mat gradient = cv::Mat(img.rows, img.cols, CV_64FC1);

		for (size_t i = 0; i < dx.rows; i++)
		{
			double* ptrx = dx.ptr<double>(i);
			double* ptry = dy.ptr<double>(i);
			double* ptrg = gradient.ptr<double>(i);
			for (int j = 0; j < dx.cols; j++)
			{
				ptrg[j] = sqrt(ptrx[j] * ptrx[j] + ptry[j] * ptry[j]);
			}
		}

		CutEdge(gradient);//去除边缘;

		double maxValue = 255;

		cv::Mat gradient_bw = cv::Mat(gradient.rows, gradient.cols, gradient.type());


		cv::Mat gradient_adp_src;
		cv::Mat gradient_adp_dst;
		gradient.convertTo(gradient_adp_src, CV_8UC1);

		cv::threshold(gradient_adp_src,gradient_adp_dst,BINARY_THRESH,255,cv::THRESH_BINARY);

#ifdef _DEBUG
																																											  //create image
		cv::Mat img_bw2;
		gradient_adp_dst.convertTo(img_bw2, img.channels());
		//DisplayElipses(img_bw2, vectEllipses);//显示二值图像;
#endif

		gradient_adp_dst.convertTo(gradient_bw, CV_64FC1);


		//obtain edges
		std::vector<std::vector<EdgePoints>> vvectEPts;
		FindConnectedDomains2(gradient_bw, vvectEPts, int(floor(2.f*PI*axis_thr)));//获得边缘点

																				   //fitting ellipse
		FittingEllipse2(vectEllipses, vvectEPts, dx, dy, axis_thr,
			axisratio_thr, ellipse_thr, bconcentricprior);

#ifdef _DEBUG
		//DisplayElipses(img2, vectEllipses);
#endif

		return;
	}

	void EllipseDetection::FittingEllipse2(
		std::vector<EllipsePoint>& vectEllipses,
		std::vector<std::vector<EdgePoints>>& vvectEPts,
		cv::Mat dx,
		cv::Mat dy,
		double axis_thr,
		double axisratio_thr,
		double ellipse_thr,
		bool bConcentricCircle)
	{
		if (vvectEPts.size() == 0)
			return;

		std::vector<EllipsePoint> vectEllipses_t;
		if (vectEllipses.size() > 0)
			vectEllipses.clear();

		cv::Mat stdCenter = cv::Mat(1, 2, CV_64FC1);
		cv::Mat dC = cv::Mat(3, 3, CV_64FC1);
		for (int iEdg = 0; iEdg < vvectEPts.size(); iEdg++)
		{
			int pointtotal = vvectEPts.at(iEdg).size();
			std::vector<EdgePoints> vectEPt_cur = vvectEPts.at(iEdg);

			if (pointtotal < ELLIPSE_MIN_EDGE_POINT_NUM)
				continue;

			//alocate memory
			cv::Mat Ix = cv::Mat(pointtotal, 1, CV_64FC1);
			cv::Mat Iy = cv::Mat(pointtotal, 1, CV_64FC1);
			cv::Mat X = cv::Mat(pointtotal, 1, CV_64FC1);
			cv::Mat Y = cv::Mat(pointtotal, 1, CV_64FC1);
			cv::Mat ctempt = cv::Mat(pointtotal, 1, CV_64FC1);
			cv::Mat c = cv::Mat(pointtotal, 1, CV_64FC1);
			cv::Mat M = cv::Mat(pointtotal, 2, CV_64FC1);
			cv::Mat B = cv::Mat(pointtotal, 1, CV_64FC1);
			cv::Mat mpts = cv::Mat(2, 1, CV_64FC1);

			//init
			for (int i = 0; i<1; i++)
			{
				double* ptrX = X.ptr<double>(i);
				double* ptrY = Y.ptr<double>(i);
				double* ptrIx = Ix.ptr<double>(i);
				double *ptrIy = Iy.ptr<double>(i);
				for (int j = 0; j < vectEPt_cur.size(); j++)
				{
					//image start point is (0,0) and in matlab is (1,1)
					ptrX[j] = vectEPt_cur.at(j).x;
					ptrY[j] = vectEPt_cur.at(j).y;
					ptrIx[j] = dx.at<double>(vectEPt_cur.at(j).y, vectEPt_cur.at(j).x);
					ptrIy[j] = dy.at<double>(vectEPt_cur.at(j).y, vectEPt_cur.at(j).x);
				}
			}

			ctempt = Ix.mul(X);
			c = Iy.mul(Y);
			B = ctempt + c;

			//init M
			for (int i = 0; i < M.rows; i++)
			{
				double* ptrM = M.ptr<double>(i);
				double* ptrIx = Ix.ptr<double>(i);
				double* ptrIy = Iy.ptr<double>(i);
				for (int j = 0; j < M.cols; j++)
				{
					if (j == 0)
					{
						*ptrM = -(*ptrIy);
						ptrIy++;
					}
					else
					{
						*ptrM = (*ptrIx);
						ptrIx++;
					}
					ptrM++;
				}
			}

			//obtain mpts = M\B
			MatLeftDivision64f(M, B, mpts);
			cv::Mat H = cv::Mat(3, 3, CV_64FC1);
			cv::setIdentity(H);
			H.at<double>(0, 2) = mpts.at<double>(0, 0);
			H.at<double>(1, 2) = mpts.at<double>(1, 0);
			cv::Mat HTranspose = cv::Mat(3, 3, CV_64FC1);
			cv::transpose(H, HTranspose);
			cv::Mat ABC = cv::Mat(3, pointtotal, CV_64FC1);


			//init ABC (ABC=[a b c]')
			for (int i = 0; i < ABC.rows; i++)
			{
				double *ptr = ABC.ptr<double>(i);
				double *a = Ix.ptr<double>(0);
				double *b = Iy.ptr<double>(0);
				double *c = B.ptr<double>(0);
				for (int j = 0; j<ABC.cols; j++)
				{
					if (i == 0)
					{
						*ptr++ = *a++;
					}
					else
					{
						if (i == 1)
						{
							*ptr++ = *b++;
						}
						else
						{
							*ptr++ = -*c++;
						}
					}
				}
			}

			// Lnorm = ((H)'*[a b c]')';
			cv::Mat Lnorm = cv::Mat(3, pointtotal, CV_64FC1);
			Lnorm = HTranspose*ABC;

			//a(Ix) = Lnorm(1,:)';b(Iy) = Lnorm(2,:)';c(B) = Lnorm(3,:)';
			for (int i = 0; i < Lnorm.rows; i++)
			{
				double *ptr = Lnorm.ptr<double>(i);
				double *a = Ix.ptr<double>(0);
				double *b = Iy.ptr<double>(0);
				double *c = B.ptr<double>(0);
				for (int j = 0; j < Lnorm.cols; j++)
				{
					if (i == 0)
						*a++ = *ptr++;
					else
					{
						if (i == 1)
							*b++ = *ptr++;
						else
							*c++ = *ptr++;
					}
				}
			}

			cv::Mat a2 = cv::Mat(pointtotal, 1, CV_64FC1);
			cv::Mat ab = cv::Mat(pointtotal, 1, CV_64FC1);
			cv::Mat b2 = cv::Mat(pointtotal, 1, CV_64FC1);
			cv::Mat ac = cv::Mat(pointtotal, 1, CV_64FC1);
			cv::Mat bc = cv::Mat(pointtotal, 1, CV_64FC1);
			a2 = Ix.mul(Ix);
			ab = Ix.mul(Iy);
			b2 = Iy.mul(Iy);
			ac = Ix.mul(B);
			bc = Iy.mul(B);

			double aa[25] =
			{
				MulSum64f(a2, a2), MulSum64f(a2, ab), MulSum64f(a2, b2), MulSum64f(a2, ac), MulSum64f(a2, bc),
				MulSum64f(a2, ab), MulSum64f(ab, ab), MulSum64f(ab, b2), MulSum64f(ab, ac), MulSum64f(ab, bc),
				MulSum64f(a2, b2), MulSum64f(ab, b2), MulSum64f(b2, b2), MulSum64f(b2, ac), MulSum64f(b2, bc),
				MulSum64f(a2, ac), MulSum64f(ab, ac), MulSum64f(b2, ac), MulSum64f(ac, ac), MulSum64f(ac, bc),
				MulSum64f(a2, bc), MulSum64f(ab, bc), MulSum64f(b2, bc), MulSum64f(ac, bc), MulSum64f(bc, bc)
			};

			cv::Mat BB_t = cv::Mat(B.rows, B.cols, B.type());
			BB_t = B.mul(B);
			double bb[5] =
			{
				-MulSum64f(BB_t, a2),
				-MulSum64f(BB_t, ab),
				-MulSum64f(BB_t, b2),
				-MulSum64f(BB_t, ac),
				-MulSum64f(BB_t, bc)
			};

			cv::Mat AA = cv::Mat(5, 5, CV_64FC1, aa);
			cv::Mat BB = cv::Mat(5, 1, CV_64FC1, bb);


			//cheching
			if (cv::determinant(AA) < 10e-10)
				continue;

			cv::Mat U = cv::Mat(AA.rows, AA.cols, CV_64FC1, CvScalar(0));
			cv::Mat S1 = cv::Mat(AA.rows, AA.cols, CV_64FC1, CvScalar(0));
			cv::Mat V = cv::Mat(AA.rows, AA.cols, CV_64FC1, CvScalar(0));
			cv::Mat VT = cv::Mat(AA.rows, AA.cols, CV_64FC1, CvScalar(0));
			cv::Mat S = cv::Mat(AA.rows, AA.cols, CV_64FC1, CvScalar(0));

			cv::SVDecomp(AA, S1, U, VT, 0);

			cv::transpose(VT, V);

			for (int i = 0; i < S.rows; i++)
			{
				S.at<double>(i, i) = S1.at<double>(i);
			}

			U.convertTo(U, -1.f, -1.f);
			V.convertTo(V, -1.f, -1.f);


			cv::Mat Sinv = cv::Mat(S.rows, S.cols, S.type(), CvScalar(0));
			cv::invert(S, Sinv, CV_SVD);


			//iAA = V*Sinv*U';
			cv::Mat VmultiSinv = cv::Mat(V.rows, Sinv.cols, CV_64FC1);
			VmultiSinv = (1.f*V)*Sinv;
			cv::Mat iAA = cv::Mat(VmultiSinv.rows, U.rows, CV_64FC1);
			cv::Mat UT = cv::Mat(AA.rows, AA.cols, CV_64FC1, CvScalar(0));
			cv::transpose(U, UT);
			iAA = (1.f*VmultiSinv)*UT;

			//sol = (iAA*BB); sol = [sol;1];
			cv::Mat sol = cv::Mat(iAA.rows, BB.cols, CV_64FC1);
			sol = (1.f*iAA)*BB;
			cv::Mat sol1 = cv::Mat(iAA.rows + 1, BB.cols, CV_64FC1);
			memcpy(sol1.ptr<double>(0), sol.ptr<double>(0), iAA.rows * CV_ELEM_SIZE(sol.type()));
			*sol1.ptr<double>(sol.rows) = 1;

			//dCnorm = rebuildConic(sol1);
			cv::Mat dCnorm = cv::Mat(3, 3, CV_64FC1);
			RebuildConic(sol1, dCnorm);

			// dC = (H)*dCnorm*(H)';
			dC = (1.f*H)*dCnorm;
			dC = dC*HTranspose;

			//error estimation
			cv::Mat s;
			s = sol.clone();
			cv::Mat BBB_t = cv::Mat(B.rows, B.cols, B.type());
			BBB_t = BB_t.mul(B);
			double BTB = MulSum64f(BBB_t, B);

			cv::Mat s_t = cv::Mat(s.cols, s.rows, CV_64FC1);
			cv::transpose(s, s_t);
			cv::Mat sAAs = cv::Mat(1, 1, CV_64FC1);
			cv::Mat sBB = cv::Mat(1, 1, CV_64FC1);
			TribleGEMM64f(s_t, AA, s, sAAs);
			sBB = (1.f*s_t)*BB;

			//R = (s'*AA*s-2*s'*BB+BTB)/(length(a)-5);
			double R = (sAAs.at<double>(0, 0) - 2 * sBB.at<double>(0, 0) + BTB) /
				(Ix.rows - 5);
			cv::Mat cvar2_constantVariance = cv::Mat(AA.rows, AA.cols, CV_64FC1);
			cv::Mat AAInv = cv::Mat(AA.rows, AA.cols, CV_64FC1);
			cv::invert(AA, AAInv);
			AAInv.convertTo(cvar2_constantVariance, -1, R);
			cv::Mat cvar2_constantVariance22 = cv::Mat(2, 2, CV_64FC1);
			cvar2_constantVariance22.at<double>(0, 0) = cvar2_constantVariance.at<double>(3, 3);
			cvar2_constantVariance22.at<double>(0, 1) = cvar2_constantVariance.at<double>(3, 4);
			cvar2_constantVariance22.at<double>(1, 0) = cvar2_constantVariance.at<double>(4, 3);
			cvar2_constantVariance22.at<double>(1, 1) = cvar2_constantVariance.at<double>(4, 4);
			cv::Mat S2 = cv::Mat(2, 2, CV_64FC1, CvScalar(0));
			cv::Mat V2 = cv::Mat(2, 2, CV_64FC1);
			cv::Mat V2T = cv::Mat(2, 2, CV_64FC1);
			cv::Mat U2 = cv::Mat(2, 2, CV_64FC1);
			cv::Mat S22 = cv::Mat(2, 2, CV_64FC1);
			cv::SVD::compute(cvar2_constantVariance22, S22, U2, V2T);
			for (int i = 0; i < S2.rows; i++)
			{
				S2.at<double>(i, i) = S22.at<double>(i);
			}
			cv::transpose(V2T, V2);
			V2.convertTo(V2, -1.f, -1.f);
			SqrtMat64f(S2, S2);
			S2.convertTo(S2, -1.f, 0.25f);
			stdCenter.at<double>(0, 0) = S2.at<double>(0, 0);
			stdCenter.at<double>(0, 1) = S2.at<double>(1, 1);

			//angleIncertitude = double( atan2( double( cvmGet( V2, 1, 0 ) ), 
			//	double( cvmGet( V2, 0, 0 ) ) ) );

			double *stdCenter1 = stdCenter.ptr<double>(0);
			double *stdCenter2 = stdCenter.ptr<double>(0) + 1;

			//ellipse degree 0.075
			if (((*stdCenter1) > ellipse_thr) ||
				((*stdCenter1) == -1) ||
				((*stdCenter2) > ellipse_thr))
				continue;

			cv::Mat C = cv::Mat(3, 3, CV_64FC1);
			cv::invert(dC, C);

			C.convertTo(C, -1, 1.f / C.at<double>(2, 2));

			cv::Mat par = cv::Mat(1, 6, CV_64FC1);
			DebuildConic(C, par);
			cv::Mat Ellipse;
			Ellipse = par.clone();
			double Center[2];
			double parArr[6];
			memcpy(parArr, par.ptr<double>(0), 6 * sizeof(double));
			Center[0] = (parArr[1] * parArr[4] - 2.f * parArr[2] * parArr[3]) /
				(4.f * parArr[0] * parArr[2] - parArr[1] * parArr[1]);
			Center[1] = (parArr[1] * parArr[3] - 2.f * parArr[0] * parArr[4]) /
				(4.f * parArr[0] * parArr[2] - parArr[1] * parArr[1]);

			double centrex = 0.f, centrey = 0.f, axea = 0.f, axeb = 0.f, angle = 0.f;
			double thetarad = 0.5f * atan2(parArr[1], parArr[0] - parArr[2]);
			double cost = cos(thetarad);
			double sint = sin(thetarad);
			double sin_squared = sint * sint;
			double cos_squared = cost * cost;
			double cos_sin = sint * cost;
			double Ao = parArr[5];
			double Au = parArr[3] * cost + parArr[4] * sint;
			double Av = -parArr[3] * sint + parArr[4] * cost;
			double Auu = parArr[0] * cos_squared + parArr[2] * sin_squared + parArr[1] * cos_sin;
			double Avv = parArr[0] * sin_squared + parArr[2] * cos_squared - parArr[1] * cos_sin;

			if (Auu == 0 || Avv == 0)
				continue;

			double tuCentre = -Au / (2.f * Auu);
			double tvCentre = -Av / (2.f * Avv);
			double wCentre = Ao - Auu * tuCentre * tuCentre - Avv * tvCentre * tvCentre;
			double uCentre = tuCentre * cost - tvCentre * sint;
			double vCentre = tuCentre * sint + tvCentre * cost;
			double Ru = -wCentre / Auu;
			double Rv = -wCentre / Avv;
			Ru = sqrt(abs(Ru)) * (Ru > 0 ? 1 : (Ru < 0 ? -1 : 0));
			Rv = sqrt(abs(Rv)) * (Rv > 0 ? 1 : (Rv < 0 ? -1 : 0));
			centrex = uCentre;
			centrey = vCentre;
			axea = Ru;
			axeb = Rv;
			angle = thetarad;

			bool temp1 = axea > axis_thr && axeb > axis_thr;
			bool temp2 = ((axea > axeb ? axea : axeb) / (axea < axeb ? axea : axeb)) < axisratio_thr;

			if (!temp1 || !temp2 || centrex<1 || centrey<1 || axea<0 || axea<0)
				continue;

			//obtaining ellipse param
			cv::Point2f center;
			center.y = float(centrey);
			center.x = float(centrex);
			vectEllipses_t.push_back(EllipsePoint(center, axea, axeb, angle));
		}

		//
		std::vector<EllipsePoint> vectEllipses_tt;
		if (bConcentricCircle)
		{
			for (int i = 0; i < vectEllipses_t.size(); i++)
			{
				EllipsePoint EPt_i = vectEllipses_t.at(i);
				bool bFind = false;
				for (int j = 0; j < vectEllipses_t.size(); j++)
				{
					EllipsePoint EPt_j = vectEllipses_t.at(j);
					if (i != j)
					{
						cv::Point2f pti = EPt_i.Pt;
						cv::Point2f ptj = EPt_j.Pt;
						float fDist = sqrt((pti.x - ptj.x)*(pti.x - ptj.x) + (pti.y - ptj.y)*(pti.y - ptj.y));
						if (fDist < (EPt_i.a + EPt_i.b) / 2.f &&
							fDist < (EPt_j.a + EPt_j.b) / 2.f)
						{
							bFind = true;
							if ((EPt_i.a + EPt_i.b) < (EPt_j.a + EPt_j.b))
								vectEllipses_tt.push_back(EPt_i);
						}
					}
				}

				if (!bFind)
					vectEllipses_tt.push_back(EPt_i);
			}
		}
		else
			for (int i = 0; i < vectEllipses_t.size(); i++)
				vectEllipses_tt.push_back(vectEllipses_t.at(i));



		for (int i = 0; i < vectEllipses_tt.size(); i++)
		{
			EllipsePoint EPt = vectEllipses_tt.at(i);
			vectEllipses.push_back(EPt);
		}

	}

	void EllipseDetection::MatLeftDivision64f(cv::Mat M, cv::Mat B, cv::Mat mpts)
	{
		double mpts1 = 0, mpts2 = 0;

		cv::Mat a = cv::Mat(M.rows, 1, CV_64FC1);
		cv::Mat b = cv::Mat(M.rows, 1, CV_64FC1);
		double* ptra = a.ptr<double>(0);
		double* ptrb = b.ptr<double>(0);
		double* ptrM = M.ptr<double>(0);

		for (int i = 0; i < M.rows; i++)
		{
			*ptra = *ptrM;
			ptra++;
			ptrM++;

			*ptrb = *ptrM;
			ptrb++;
			ptrM++;
		}

		mpts2 =
			(MulSum64f(a, B) * MulSum64f(a, b) -
				MulSum64f(b, B) * MulSum64f(a, a)) /
				(MulSum64f(a, b) * MulSum64f(a, b) -
					MulSum64f(b, b) * MulSum64f(a, a));

		mpts2 =
			(MulSum64f(a, B) * MulSum64f(b, b) -
				MulSum64f(b, B) * MulSum64f(a, b)) /
				(MulSum64f(a, a) * MulSum64f(b, b) -
					MulSum64f(a, b) * MulSum64f(a, b));

		mpts.at<double>(0, 0) = mpts1;
		mpts.at<double>(1, 0) = mpts2;
	}

	double EllipseDetection::MulSum64f(cv::Mat src1, cv::Mat src2)
	{
		cv::Mat dst = cv::Mat(src1.rows, src2.cols, CV_64FC1);
		dst = src1.mul(src2);
		return cv::sum(dst).val[0];
	}

	void EllipseDetection::RebuildConic(cv::Mat src, cv::Mat dst)
	{
		double* ptrsrc = src.ptr<double>(0);
		double* ptrdst = dst.ptr<double>(0);
		ptrdst[0] = ptrsrc[0];
		ptrdst[1] = ptrsrc[1] / 2.f;
		ptrdst[2] = ptrsrc[3] / 2.f;
		ptrdst[3] = ptrsrc[1] / 2.f;
		ptrdst[4] = ptrsrc[2];
		ptrdst[5] = ptrsrc[4] / 2.f;
		ptrdst[6] = ptrsrc[3] / 2.f;
		ptrdst[7] = ptrsrc[4] / 2.f;
		ptrdst[8] = ptrsrc[5];
	}

	void EllipseDetection::TribleGEMM64f(cv::Mat src1, cv::Mat src2, cv::Mat src3, cv::Mat dst)
	{
		cv::Mat temp = cv::Mat(src1.rows, src2.cols, CV_64FC1);
		temp = src1*src2;
		dst = temp*src3;
		return;
	}

	void EllipseDetection::SqrtMat64f(cv::Mat src, cv::Mat dst)
	{
		for (int i = 0; i<src.rows; i++)
		{
			double* ptrsrc = src.ptr<double>(i);
			double* ptrdst = dst.ptr<double>(i);
			for (int j = 0; j<src.cols; j++)
			{
				*ptrdst = sqrt(*ptrsrc);
				ptrsrc++;
				ptrdst++;
			}
		}
	}

	void EllipseDetection::DebuildConic(cv::Mat src, cv::Mat dst)
	{
		double* ptrsrc = src.ptr<double>(0);
		double* ptrdst = dst.ptr<double>(0);
		ptrdst[0] = ptrsrc[0];
		ptrdst[1] = ptrsrc[1] * 2.f;
		ptrdst[2] = ptrsrc[4];
		ptrdst[3] = ptrsrc[2] * 2.f;
		ptrdst[4] = ptrsrc[7] * 2.f;
		ptrdst[5] = ptrsrc[8];
	}

	void EllipseDetection::FindConnectedDomains2(
		cv::Mat mat,
		std::vector<std::vector<EdgePoints>>& vvectEPts,
		int iMinPixelNum)
	{
		for (int i = 0; i < vvectEPts.size(); i++)
			vvectEPts.at(i).clear();
		vvectEPts.clear();

		//buiding a new image
		cv::Mat mat2;//新建二值图像的智能指针计数;
		mat2 = mat.clone();

		int total = 0;

		//init the list first element
		int iStepRow = mat2.step / sizeof(double);
		//find the edges
		for (int row = 0; row < mat2.rows; row++)
		{
			double* ptr = mat2.ptr<double>(row);

			for (int col = 0; col < mat2.cols; col++)
			{
				if (*ptr != 0)//位置点是否边缘;
				{
					EdgePoints tempEP;
					tempEP.position = ptr;
					tempEP.x = col;
					tempEP.y = row;
					tempEP.next = NULL;

					std::vector<EdgePoints> S;
					S.push_back(tempEP);
					std::stack<EdgePoints> S_cnt;//栈;
					S_cnt.push(tempEP);

					*tempEP.position = 0;

					for (;;)//需break停止的无限循环，主要用于加速边缘的遍历;
					{
						EdgePoints LastEP = S_cnt.top();
						S_cnt.pop();
						EdgePoints LastEP_next;
						int iCount = 0;

						if ((LastEP.x + 1) < mat2.cols)
						{
							if (*(LastEP.position + 1))//判断下一个位置是否为边缘;
							{
								LastEP_next.position = LastEP.position + 1;
								LastEP_next.x = LastEP.x + 1;
								LastEP_next.y = LastEP.y;
								LastEP_next.next = NULL;

								S.push_back(LastEP_next);
								S_cnt.push(LastEP_next);
								++iCount;

								*(LastEP_next.position) = 0;
							}
						}

						if ((LastEP.x - 1) > 0)
						{
							if (*(LastEP.position - 1))//判断上一个位置是否为边缘
							{
								LastEP_next.position = LastEP.position - 1;
								LastEP_next.x = LastEP.x - 1;
								LastEP_next.y = LastEP.y;
								LastEP_next.next = NULL;

								S.push_back(LastEP_next);
								S_cnt.push(LastEP_next);
								++iCount;

								*(LastEP_next.position) = 0;
							}
						}

						if ((LastEP.y + 1) < mat2.rows)
						{
							if (*(LastEP.position + iStepRow))
							{
								LastEP_next.position = LastEP.position + iStepRow;
								LastEP_next.x = LastEP.x;
								LastEP_next.y = LastEP.y + 1;
								LastEP_next.next = NULL;

								S.push_back(LastEP_next);
								S_cnt.push(LastEP_next);
								++iCount;

								*(LastEP_next.position) = 0;
							}
						}

						if ((LastEP.y - 1) > 0)
						{
							if (*(LastEP.position - iStepRow))
							{
								LastEP_next.position = LastEP.position - iStepRow;
								LastEP_next.x = LastEP.x;
								LastEP_next.y = LastEP.y - 1;
								LastEP_next.next = NULL;

								S.push_back(LastEP_next);
								S_cnt.push(LastEP_next);
								++iCount;

								*(LastEP_next.position) = 0;
							}
						}



						if (S_cnt.size() == 0)
							break;
					}

					if (S.size() > iMinPixelNum)
					{
						vvectEPts.push_back(S);
						total++;
					}
				}//if( *ptr != 0 )结束;
				ptr++;
			}
		}

		return;
	}

	void EllipseDetection::CutEdge(cv::Mat src, int iMargin)
	{
		int w = src.cols, h = src.rows;

		for (int row = 0; row < iMargin; row++)
		{
			double* data = src.ptr<double>(row);
			for (int col = 0; col < w; col++)
			{
				data[col] = 0;
			}
		}

		for (int row = h - iMargin; row < h; row++)
		{
			double* data = src.ptr<double>(row);
			for (int col = 0; col < w; col++)
			{
				data[col] = 0;
			}
		}

		for (int row = iMargin; row < h - iMargin; row++)
		{
			double* data = src.ptr<double>(row);
			for (int col = 0; col < iMargin; col++)
			{
				data[col] = 0;
			}
		}

		for (int row = iMargin; row < h - iMargin; row++)
		{
			double* data = src.ptr<double>(row);
			for (int col = w - iMargin; col < w; col++)
			{
				data[col] = 0;
			}
		}
	}

	void EllipseDetection::DisplayElipses(
		cv::Mat img,
		std::vector<EllipsePoint> vectEllipsePts)
	{
		if (vectEllipsePts.empty() == true)
		{
			cvNamedWindow("the result");
			imshow("the result", img);
			cvWaitKey(0);
			cvDestroyWindow("the result");
		}
		else
		{
			int num = (int)(vectEllipsePts.size());
			printf("Number of elipses that have been detected:%d\n", num);
			printf("Centers of elipses that have been detected:\n");
			for (int i = 0; i < num; i++)
			{
				EllipsePoint EPt = vectEllipsePts.at(i);
				cv::Point2f center = EPt.Pt;
				int axea = int(floor(EPt.a + 0.5f));
				int axeb = int(floor(EPt.b + 0.5f));
				int centrex = int(floor(center.x + 0.5f));
				int centrey = int(floor(center.y + 0.5f));
				double angle = double(EPt.angle);

				printf("(%d, %d)\n", centrex, centrey);

				CvScalar scalar = cvScalar(30, 0, 0);
				CvPoint point = cvPoint(centrex, centrey);
				CvSize size = cvSize(axea, axeb);
				double angle_deg = angle * 180 / PI;
				cv::ellipse(img, point, size, angle_deg, 0, 360, scalar);
				CvPoint up = cvPoint(centrex, centrey - 5);
				CvPoint down = cvPoint(centrex, centrey + 5);
				CvPoint left = cvPoint(centrex - 5, centrey);
				CvPoint right = cvPoint(centrex + 5, centrey);
				cv::line(img, up, down, scalar);
				cv::line(img, left, right, scalar);

				//cv::Point2f point1((center.x+axea*cos(angle)),(center.y+axea*sin(angle)));
				//float dist=sqrt((point1.x-center.x)*(point1.x-center.x)+(point1.y-center.y)*(point1.y-center.y));
				//cv::Mat image01(img);//杨瑞文修改;
				//cvtColor(image01,image01,CV_GRAY2BGR);
				//cv::line(image01,cv::Point2f(0,0),cv::Point2f(100,200),cv::Scalar(0,255,0));
				//cv::line(image01,center,point1,cv::Scalar(0,255,0));
				//cv::imwrite("../Ellipse.bmp",image01);
			}
			cvNamedWindow("the result", CV_WINDOW_AUTOSIZE);
			imshow("the result", img);
			//cv::Mat image01(img);//杨瑞文修改;
			//cv::imwrite("../Ellipse.bmp",image01);
			cvWaitKey(0);
			cvDestroyWindow("the result");
		}
	}

}
