#ifndef BALPROBLEM_H
#define BALPROBLEM_H

#include <string>
#include <iostream>
class BALProblem {
public:
	BALProblem() {}
	explicit BALProblem(const std::string &filename, bool use_quaternions = false);

	~BALProblem() {
		delete[] point_index_;
		//delete[] camera_index_;
		delete[] observations_;
		delete[] parameters_;
	}

	void WriteToFile(const std::string &filename) const;

	void WriteToPLYFile(const std::string &filename) const;

	void Normalize();

	void Perturb(const double rotation_sigma,
		const double translation_sigma,
		const double point_sigma);

	// Intrinsic parameters
	//int camera_block_size() const { return 9; }

	// Extrinsic parameters
	//int pose_block_size() const { return use_quaternions_ ? 7 : 6; }
	int pose_block_size() const { return 6; } // Modified 22/10/19 , I never use it but it gets true. I don't know why and that is so wiered.

	int point_block_size() const { return 3; }

	//int num_cameras() const { return num_cameras_; }

	int num_poses() const { return num_poses_; }

	int num_points() const { return num_points_; }

	int num_observations() const { return num_observations_; }

	int num_parameters() const { return num_parameters_; }

	//const int *camera_index() const { return camera_index_; }

	const int *pose_index() const { return pose_index_; }

	const int *point_index() const { return point_index_; }

	const double *observations() const { return observations_; }

	const double *parameters() const { return parameters_; }

	//const double *cameras() const { return parameters_; }

	//const double *poses() const { return parameters_ + camera_block_size() * num_cameras_; }
	const double *poses() const { return parameters_; }

	const double *points() const { return poses() + pose_block_size() * num_poses_; }

	//double *mutable_cameras() { return parameters_; }

	//double *mutable_poses() { return parameters_ + camera_block_size() * num_cameras_; }
	double *mutable_poses() { return parameters_ ; }

	double *mutable_points() { return mutable_poses() + pose_block_size() * num_poses_; }
	/*
	double *mutable_camera_for_observation(int i) {
		return mutable_cameras() + camera_index_[i] * camera_block_size();
	}
	*/
	double *mutable_relative_pose_for_observation(int i) {
		//        assert(camera_index_[i]>0);
		//return mutable_poses() + (camera_index_[i] - 1) * pose_block_size();
		return mutable_poses() ;
	}
	/*
	double *mutable_pose_for_observation(int i) {
		return mutable_poses() + (num_cameras_ - 1 + pose_index_[i]) * pose_block_size();
	}
	*/

	double *mutable_pose_for_observation(int i) {
		return mutable_poses() + pose_index_[i] * pose_block_size();
	}
	double *mutable_point_for_observation(int i) {
		return mutable_points() + point_index_[i] * point_block_size();
	}
	/*
	const double *camera_for_observation(int i) const {
		return cameras() + camera_index_[i] * camera_block_size();
	}*/

	const double *pose_for_observation(int i) const {
		return poses() + pose_index_[i] * pose_block_size();
	}

	const double *point_for_observation(int i) const {
		return points() + point_index_[i] * point_block_size();
	}

	int *mutable_point_index() { return point_index_; }

	int *mutable_pose_index() { return pose_index_; }

	//int *mutable_camera_index() { return camera_index_; }

	double *mutable_parameters() { return parameters_; }

	double *mutable_observations() { return observations_; }

	//void setParaNums(int num_observations, int num_cameras, int num_poses, int num_points);
	void setParaNums(int num_observations, int num_poses, int num_points);
private:
	void PoseToAxisAndCenter(const double *pose,
		double *angle_axis,
		double *center) const;

	void AxisAndCenterToPose(const double *angle_axis,
		const double *center,
		double *pose) const;

	//int num_cameras_;
	int num_poses_;
	int num_points_;
	int num_observations_;
	int num_parameters_;
	bool use_quaternions_;
	int *point_index_;
	int *pose_index_;
	//int *camera_index_;
	double *observations_;
	double *parameters_;
};

//还是直接尝试 cameraindex = 0, num_cameras_ = 0这样不就可以了吗？那么BAproblem就不用改了
/*
class BALProblem {
public:
    BALProblem() {}
    explicit BALProblem(const std::string &filename, bool use_quaternions = false);

    ~BALProblem() {
        delete[] point_index_;
        delete[] camera_index_;
        delete[] observations_;
        delete[] parameters_;
    }

    void WriteToFile(const std::string &filename) const;

    void WriteToPLYFile(const std::string &filename) const;

    void Normalize();

    void Perturb(const double rotation_sigma,
                 const double translation_sigma,
                 const double point_sigma);

    // Intrinsic parameters
    int camera_block_size() const { return 9; }

    // Extrinsic parameters
    //int pose_block_size() const { return use_quaternions_ ? 7 : 6; }
	int pose_block_size() const { return 6; } // Modified 22/10/19 , I never use it but it gets true. I don't know why and that is so wiered.

    int point_block_size() const { return 3; }

    int num_cameras() const { return num_cameras_; }

    int num_poses() const { return num_poses_; }

    int num_points() const { return num_points_; }

    int num_observations() const { return num_observations_; }

    int num_parameters() const { return num_parameters_; }

    const int *camera_index() const { return camera_index_; }

    const int *pose_index() const { return pose_index_; }

    const int *point_index() const { return point_index_; }

    const double *observations() const { return observations_; }

    const double *parameters() const { return parameters_; }

    const double *cameras() const { return parameters_; }

    const double *poses() const { return parameters_ + camera_block_size() * num_cameras_; }

    const double *points() const { return poses() + pose_block_size() * num_poses_; }

    double *mutable_cameras() { return parameters_; }

    double *mutable_poses() { return parameters_ + camera_block_size() * num_cameras_; }

    double *mutable_points() { return mutable_poses() + pose_block_size() * num_poses_; }

    double *mutable_camera_for_observation(int i) {
        return mutable_cameras() + camera_index_[i] * camera_block_size();
    }

    double *mutable_relative_pose_for_observation(int i) {
//        assert(camera_index_[i]>0);
        return mutable_poses() + (camera_index_[i] - 1) * pose_block_size();
    }

    double *mutable_pose_for_observation(int i) {
        return mutable_poses() + (num_cameras_ - 1 + pose_index_[i]) * pose_block_size();
    }

    double *mutable_point_for_observation(int i) {
        return mutable_points() + point_index_[i] * point_block_size();
    }

    const double *camera_for_observation(int i) const {
        return cameras() + camera_index_[i] * camera_block_size();
    }

    const double *pose_for_observation(int i) const {
        return poses() + pose_index_[i] * pose_block_size();
    }

    const double *point_for_observation(int i) const {
        return points() + point_index_[i] * point_block_size();
    }

    int *mutable_point_index() { return point_index_; }

    int *mutable_pose_index() { return pose_index_; }

    int *mutable_camera_index() { return camera_index_; }

    double *mutable_parameters() { return parameters_; }

    double *mutable_observations() { return observations_; }

    void setParaNums(int num_observations, int num_cameras, int num_poses, int num_points);

private:
    void PoseToAxisAndCenter(const double *pose,
                             double *angle_axis,
                             double *center) const;

    void AxisAndCenterToPose(const double *angle_axis,
                             const double *center,
                             double *pose) const;

    int num_cameras_;
    int num_poses_;
    int num_points_;
    int num_observations_;
    int num_parameters_;
    bool use_quaternions_;
    int *point_index_;
    int *pose_index_;
    int *camera_index_;
    double *observations_;
    double *parameters_;
};
*/
#endif // BALProblem.h
