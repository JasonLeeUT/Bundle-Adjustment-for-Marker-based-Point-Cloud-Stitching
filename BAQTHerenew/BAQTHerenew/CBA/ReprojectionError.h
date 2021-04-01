#ifndef SnavelyReprojection_H
#define SnavelyReprojection_H

#include <iostream>
#include "ceres/ceres.h"


//#include "../utils/rotation.h"
#include "../CBA/rotation.h"
#include "projection.h"

class ReprojectionError {
public:
	ReprojectionError(double observation_x, double observation_y) : observed_x(observation_x),
		observed_y(observation_y) {}

	template<typename T>
	bool operator()(const T *const pose,
		const T *const point, T *residuals) const {
		T predictions[2];
		//CamProjectionWithDistortion(pose, camera, point, predictions);
		CamProjectionWithDistortion(pose, point, predictions);
		residuals[0] = predictions[0] - T(observed_x);
		residuals[1] = predictions[1] - T(observed_y);
		return true;
	}

	static ceres::CostFunction *Create(const double observed_x, const double observed_y) {
		return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6, 3>(
			new ReprojectionError(observed_x, observed_y)));
	}

private:
	double observed_x;
	double observed_y; //这里 observed_x observed_y 是我们的输入的常数，是不需要优化的
};
/*
class ReprojectionError {
public:
    ReprojectionError(double observation_x, double observation_y) : observed_x(observation_x),
                                                                    observed_y(observation_y) {}

    template<typename T>
    bool operator()(const T *const pose, const T *const camera,
                    const T *const point, T *residuals) const {
        T predictions[2];
        CamProjectionWithDistortion(pose, camera, point, predictions);
        residuals[0] = predictions[0] - T(observed_x);
        residuals[1] = predictions[1] - T(observed_y);
        return true;
    }

    static ceres::CostFunction *Create(const double observed_x, const double observed_y) {
        return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6, 9, 3>(
                new ReprojectionError(observed_x, observed_y)));   
    }

private:
    double observed_x;
    double observed_y; //这里 observed_x observed_y 是我们的输入的常数，是不需要优化的
};
*/
/*
class ReprojectionError {
public:
	ReprojectionError(double observation_x, double observation_y) : observed_x_(observation_x),
		observed_y_(observation_y) {}

	template<typename T>
	bool operator()(const T* const pose, const T* const camera,
		const T* const point, T* residuals) const {
		T predictions[2];
		CamProjectionWithDistortion(pose, camera, point, predictions);
		residuals[0] = predictions[0] - T(observed_x_);
		residuals[1] = predictions[1] - T(observed_y_);
		return true;
	}

	static ceres::CostFunction *Create(const double observed_x, const double observed_y) {
		double a = observed_x;
		double b = observed_y;
		return (new ceres::AutoDiffCostFunction<ReprojectionError,2,6,9,3>(
			new ReprojectionError(a, b)));
	}

private:
	double observed_x_;
	double observed_y_; //这里 observed_x observed_y 是我们的输入的常数，是不需要优化的
};
*/
class StereoReprojectionError {
public:
    StereoReprojectionError(double observation_x, double observation_y) : observed_x(observation_x),
                                                                          observed_y(observation_y) {}

    template<typename T>
    bool operator()(const T *const relative_pose, const T *const pose, const T *const camera,
                    const T *const point, T *residuals) const {
        T predictions[2];
        CamProjectionWithDistortion(relative_pose, pose, camera, point, predictions);
        residuals[0] = predictions[0] - T(observed_x);
        residuals[1] = predictions[1] - T(observed_y);
        return true;
    }

    static ceres::CostFunction *Create(const double observed_x, const double observed_y) {
        return (new ceres::AutoDiffCostFunction<StereoReprojectionError, 2, 6, 6, 9, 3>(
                new StereoReprojectionError(observed_x, observed_y)));
    }

private:
    double observed_x;
    double observed_y;
};

#endif // SnavelyReprojection.h

