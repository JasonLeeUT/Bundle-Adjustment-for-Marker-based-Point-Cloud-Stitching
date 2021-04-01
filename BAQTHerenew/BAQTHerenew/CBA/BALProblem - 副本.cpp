#include "BALProblem.h"

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>
#include <Eigen/Core>

#include "../utils/random.h"
#include "../utils/rotation.h"

typedef Eigen::Map<Eigen::VectorXd> VectorRef;
typedef Eigen::Map<const Eigen::VectorXd> ConstVectorRef;

template<typename T>
void FscanfOrDie(FILE *fptr, const char *format, T *value) {
    int num_scanned = fscanf(fptr, format, value);
    if (num_scanned != 1)
        std::cerr << "Invalid UW data file. ";
}

void PerturbPoint3(const double sigma, double *point) {
    for (int i = 0; i < 3; ++i)
        point[i] += RandNormal() * sigma;
}

double Median(std::vector<double> *data) {
    int n = data->size();
    std::vector<double>::iterator mid_point = data->begin() + n / 2;
    std::nth_element(data->begin(), mid_point, data->end());
    return *mid_point;
}

BALProblem::BALProblem(const std::string &filename, bool use_quaternions) {
    FILE *fptr = fopen(filename.c_str(), "r");

    if (fptr == nullptr) {
        std::cerr << "Error: unable to open file " << filename;
        return;
    };
    // This wil die horribly on invalid files. Them's the breaks.
    FscanfOrDie(fptr, "%d", &num_observations_);
    FscanfOrDie(fptr, "%d", &num_cameras_);
    FscanfOrDie(fptr, "%d", &num_poses_);
    FscanfOrDie(fptr, "%d", &num_points_);

    std::cout << "Header: " << num_observations_ << " "
              << num_cameras_ << " "
              << num_poses_ << " "
              << num_points_ << " ";

    camera_index_ = new int[num_observations_];
    pose_index_ = new int[num_observations_];
    point_index_ = new int[num_observations_];
    observations_ = new double[2 * num_observations_];



    for (int i = 0; i < num_observations_; ++i) {
        FscanfOrDie(fptr, "%d", camera_index_ + i);
        FscanfOrDie(fptr, "%d", pose_index_ + i);
        FscanfOrDie(fptr, "%d", point_index_ + i);
        for (int j = 0; j < 2; ++j)
            FscanfOrDie(fptr, "%lf", observations_ + 2 * i + j);
    }

    for (int i = 0; i < num_parameters_; ++i)
        FscanfOrDie(fptr, "%lf", parameters_ + i);

    fclose(fptr);

    use_quaternions_ = use_quaternions;
    if (use_quaternions) {
        // Switch the angle-axis rotations to quaternions.
        num_parameters_ = 9 * num_cameras_ + 7 * num_poses_ + 3 * num_points_;
        double *quaternion_parameters = new double[num_parameters_];
        double *original_cursor = parameters_;
        double *quaternion_cursor = quaternion_parameters;
        for (int i = 0; i < num_cameras_; ++i)
            *quaternion_cursor++ = *original_cursor++;
        for (int i = 0; i < num_poses_; ++i) {
            AngleAxisToQuaternion(original_cursor, quaternion_cursor);
            quaternion_cursor += 4;
            original_cursor += 3;
            for (int j = 4; j < 7; ++j)
                *quaternion_cursor++ = *original_cursor++;
        }
        // Copy the rest of the points.
        for (int i = 0; i < 3 * num_points_; ++i)
            *quaternion_cursor++ = *original_cursor++;
        // Swap in the quaternion parameters.
        delete[] parameters_;
        parameters_ = quaternion_parameters;
    }
}

void BALProblem::WriteToFile(const std::string &filename) const {
    FILE *fptr = fopen(filename.c_str(), "w");

    if (fptr == nullptr) {
        std::cerr << "Error: unable to open file " << filename;
        return;
    }
    fprintf(fptr, "%d %d %d %d\n", num_observations_, num_cameras_, num_poses_, num_points_);

    for (int i = 0; i < num_observations(); ++i) {
        fprintf(fptr, "%d %d %d", camera_index_[i], pose_index_[i], point_index_[i]);
        for (int j = 0; j < 2; ++j)
            fprintf(fptr, " %g", observations_[2 * i + j]);
        fprintf(fptr, "\n");
    }
    for (int i = 0; i < num_cameras(); ++i) {
        const double *camera = cameras() + 9 * i;
        for (int j = 0; j < 9; ++j)
            fprintf(fptr, "%g\n", camera[j]);
    }
    for (int i = 0; i < num_poses(); ++i) {
        double angleaxis[6];
        if (use_quaternions_) {
            QuaternionToAngleAxis(poses() + 7 * i, angleaxis);
            memcpy(angleaxis + 3, poses() + 7 * i + 4, 3 * sizeof(double));
        } else {
            memcpy(angleaxis, poses() + 6 * i, 6 * sizeof(double));
        }
        for (double item : angleaxis)
            fprintf(fptr, "%g\n", item);
    }
    for (int i = 0; i < num_points(); ++i) {
        const double *point = points() + i * point_block_size();
        for (int j = 0; j < point_block_size(); ++j)
            fprintf(fptr, "%g\n", point[j]);
    }

    fclose(fptr);
}

// Write the problem to a PLY file for inspection in Meshlab or CloudCompare
void BALProblem::WriteToPLYFile(const std::string &filename) const {
    std::ofstream of(filename.c_str());

    of << "ply"
       << '\n' << "format ascii 1.0"
       << '\n' << "element vertex " << num_poses_ + num_points_
       << '\n' << "property float x"
       << '\n' << "property float y"
       << '\n' << "property float z"
       << '\n' << "property uchar red"
       << '\n' << "property uchar green"
       << '\n' << "property uchar blue"
       << '\n' << "end_header" << std::endl;

    // Export extrinsic data (i.e. camera centers) as green points.
    double angle_axis[3];
    double center[3];
    for (int i = 0; i < num_poses(); ++i) {
        const double *pose = poses() + pose_block_size() * i;
        PoseToAxisAndCenter(pose, angle_axis, center);
        of << center[0] << ' ' << center[1] << ' ' << center[2]
           << " 0 255 0" << '\n';
    }

    // Export the structure (i.e. 3D Points) as white points.
//    const double *points = parameters_ + pose_block_size() * num_poses_;
    for (int i = 0; i < num_points(); ++i) {
        const double *point = points() + point_block_size() * i;
        of << point[0] << ' ' << point[1] << ' ' << point[2]
           << " 255 255 255" << "\n";
    }
    of.close();
}

void BALProblem::PoseToAxisAndCenter(const double *pose,
                                     double *angle_axis,
                                     double *center) const {
    VectorRef angle_axis_ref(angle_axis, 3);
    if (use_quaternions_)
        QuaternionToAngleAxis(pose, angle_axis);
    else
        angle_axis_ref = ConstVectorRef(pose, 3);
    // c = -R't
    Eigen::VectorXd inverse_rotation = -angle_axis_ref;
    AngleAxisRotatePoint(inverse_rotation.data(),
                         pose + pose_block_size() - 3,
                         center);
    VectorRef(center, 3) *= -1.0;
}

void BALProblem::AxisAndCenterToPose(const double *angle_axis,
                                     const double *center,
                                     double *pose) const {
    ConstVectorRef angle_axis_ref(angle_axis, 3);
    if (use_quaternions_)
        AngleAxisToQuaternion(angle_axis, pose);
    else
        VectorRef(pose, 3) = angle_axis_ref;
    // t = -R * c 
    AngleAxisRotatePoint(angle_axis, center, pose + pose_block_size() - 6);
    VectorRef(pose + pose_block_size() - 3, 3) *= -1.0;
}

void BALProblem::Normalize() {
    // Compute the marginal median of the geometry
    std::vector<double> tmp(num_points_);
    Eigen::Vector3d median;
    double *points = mutable_points();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < num_points_; ++j)
            tmp[j] = points[3 * j + i];
        median(i) = Median(&tmp);
    }
    for (int i = 0; i < num_points_; ++i) {
        VectorRef point(points + 3 * i, 3);
        tmp[i] = (point - median).lpNorm<1>();
    }
    const double median_absolute_deviation = Median(&tmp);
    // Scale so that the median absolute deviation of the resulting
    // reconstruction is 100
    const double scale = 100.0 / median_absolute_deviation;

    // X = scale * (X - median)
    for (int i = 0; i < num_points_; ++i) {
        VectorRef point(points + 3 * i, 3);
        point = scale * (point - median);
    }

    double *poses = mutable_poses();
    double angle_axis[3];
    double center[3];
    for (int i = 0; i < num_poses_; ++i) {
        double *pose = poses + pose_block_size() * i;
        PoseToAxisAndCenter(pose, angle_axis, center);
        // center = scale * (center - median)
        VectorRef(center, 3) = scale * (VectorRef(center, 3) - median);
        AxisAndCenterToPose(angle_axis, center, pose);
    }
}

void BALProblem::Perturb(const double rotation_sigma,
                         const double translation_sigma,
                         const double point_sigma) {
    assert(point_sigma >= 0.0);
    assert(rotation_sigma >= 0.0);
    assert(translation_sigma >= 0.0);

    double *points = mutable_points();
    if (point_sigma > 0)
        for (int i = 0; i < num_points_; ++i)
            PerturbPoint3(point_sigma, points + 3 * i);
    for (int i = 0; i < num_poses_; ++i) {
        double *pose = mutable_poses() + pose_block_size() * i;
        double angle_axis[3];
        double center[3];
        // Perturb in the rotation of the pose in the angle-axis
        // representation
        PoseToAxisAndCenter(pose, angle_axis, center);
        if (rotation_sigma > 0.0)
            PerturbPoint3(rotation_sigma, angle_axis);
        AxisAndCenterToPose(angle_axis, center, pose);
        if (translation_sigma > 0.0)
            PerturbPoint3(translation_sigma, pose + pose_block_size() - 3);
    }
}

void BALProblem::setParaNums(int num_observations, int num_cameras, int num_poses, int num_points) {
    num_observations_ = num_observations;
    num_cameras_ = num_cameras;
    num_poses_ = num_poses;
    num_points_ = num_points;
    camera_index_ = new int[num_observations_];
    pose_index_ = new int[num_observations_];
    point_index_ = new int[num_observations_];
    observations_ = new double[2 * num_observations_];
    num_parameters_ = 9 * num_cameras_ + 6 * num_poses_ + 3 * num_points_;
    parameters_ = new double[num_parameters_];
}


