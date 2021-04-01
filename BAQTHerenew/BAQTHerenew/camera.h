#ifndef CAMERA_H
#define CAMERA_H

#include "common_include.h"
#include "config.h"

namespace amnl
{
	class Camera
	{
	public:
		typedef std::shared_ptr<Camera> Ptr;
		Camera();
		//Camera(Mat cammatrix, Mat distortioncoeffs, Mat rot, Mat trans) :
		//	camMatrix_(cammatrix), distortionCoeffs_(distortioncoeffs), rotationMatrix_(rot), translationVector_(trans) {}

		~Camera();

		// access to the private members
		cv::Mat getCamMatrix() { return camMatrix_; };
		cv::Mat getDistortionCoeffs() { return distortionCoeffs_; };
		cv::Mat getRotationMatrix() { return rotationMatrix_; };
		cv::Mat getTranslationVector() { return translationVector_; };
		cv::Mat getRMatrix() { return R_; };
		cv::Mat getPMatrix() { return P_; };
		cv::Mat getQMatrix() { return Q_; };

		void setCamMatrix(cv::Mat inputMatrix) { camMatrix_ = inputMatrix; };
		void setDistortionCoeffs(cv::Mat inputMatrix) { distortionCoeffs_ = inputMatrix; };
		void setRotationMatrix(cv::Mat inputMatrix) { rotationMatrix_ = inputMatrix; };
		void setTranslationVector(cv::Mat inputMatrix) { translationVector_ = inputMatrix; };
		void setRMatrix(cv::Mat inputMatrix) { R_ = inputMatrix; };
		void setPMatrix(cv::Mat inputMatrix) { P_ = inputMatrix; };
		void setQMatrix(cv::Mat inputMatrix) { Q_ = inputMatrix; };

		//from config file
		bool loadCamMatrix(string path, string name);
		bool loadDistortionCoeffs(string path, string name);
		bool loadRotationMatrix(string path, string name);
		bool loadTranslationVector(string path, string name);
		bool loadRMatrix(string path, string name);
		bool loadPMatrix(string path, string name);
		bool loadQMatrix(string path, string name);

	private:
		//internal parameters
		cv::Mat camMatrix_ = cv::Mat::eye(3, 3, CV_64FC1); //camera matrix 3x3 CV_64FC1
		double fx_, fy_, cx_, cy_; 
		cv::Mat distortionCoeffs_;  //3 step radial distortion and 2 step tangential distortion 1x5 CV_64FC1
		//external parameters
		cv::Mat rotationMatrix_;    //rot 3x3 CV_64FC1
		cv::Mat translationVector_;    //trans 3x1 CV_64FC1
		//the refication parameters
		cv::Mat R_;
		cv::Mat P_;
		cv::Mat Q_;
		//others
		int width_;
		int height_;
		//
	};

}
#endif // CAMERA_H
