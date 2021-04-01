#include "camera.h"
namespace amnl
{
Camera::Camera()
{
	camMatrix_ = cv::Mat::eye(3, 3, CV_64FC1);
	distortionCoeffs_ = cv::Mat::zeros(1, 5, CV_64FC1);
	rotationMatrix_ = cv::Mat::eye(3, 3, CV_64FC1);
	translationVector_ = cv::Mat::zeros(3, 1, CV_64FC1);
	fx_ = 0.0;
	fy_ = 0.0;
	cx_ = 0.0;
	cy_ = 0.0;
	width_ = 0.0;
	height_ = 0.0;
}


Camera::~Camera()
{
}

bool Camera::loadCamMatrix(string path,string name)
{
	Config::setParameterFile(path);
	cv::Mat camMatrix = Config::get<cv::Mat>(name);
	camMatrix_ = cv::Mat::eye(3, 3, CV_64FC1);
	camMatrix_ = camMatrix;
	return true;
}

bool Camera::loadDistortionCoeffs(string path,string name)
{
	Config::setParameterFile(path);
	distortionCoeffs_ = Config::get<cv::Mat>(name);
	return true;
}

bool Camera::loadRotationMatrix(string path,string name)
{
	Config::setParameterFile(path);
	rotationMatrix_ = Config::get<cv::Mat>(name);
	return true;
}

bool Camera::loadTranslationVector(string path, string name)
{
	Config::setParameterFile(path);
	translationVector_ = Config::get<cv::Mat>(name);
	return true;
}

bool Camera::loadRMatrix(string path, string name)
{
	Config::setParameterFile(path);
	R_ = Config::get<cv::Mat>(name);
	return true;
}

bool Camera::loadPMatrix(string path, string name)
{
	Config::setParameterFile(path);
	P_ = Config::get<cv::Mat>(name);
	return true;
}

bool Camera::loadQMatrix(string path, string name)
{
	Config::setParameterFile(path);
	Q_ = Config::get<cv::Mat>(name);
	return true;
}

}
