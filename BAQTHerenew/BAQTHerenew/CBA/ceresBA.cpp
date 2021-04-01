//
// Created by cherichy on 2019/3/3.
//
#include "ceresBA.h"
#include "ReprojectionError.h"

void ceresBA::SetLinearSolver(ceres::Solver::Options *options, const BundleParams &params) {
    CHECK(ceres::StringToLinearSolverType(params.linear_solver, &options->linear_solver_type));
    CHECK(ceres::StringToSparseLinearAlgebraLibraryType(params.sparse_linear_algebra_library,
                                                        &options->sparse_linear_algebra_library_type));
    CHECK(ceres::StringToDenseLinearAlgebraLibraryType(params.dense_linear_algebra_library,
                                                       &options->dense_linear_algebra_library_type));
    options->num_linear_solver_threads = params.num_threads;
}

void ceresBA::SetOrdering(BALProblem *bal_problem, ceres::Solver::Options *options, const BundleParams &params) {
    const int num_points = bal_problem->num_points();
    const int point_block_size = bal_problem->point_block_size();
    double *points = bal_problem->mutable_points();

    //const int num_cameras = bal_problem->num_cameras();
   // const int camera_block_size = bal_problem->camera_block_size();
    //double *cameras = bal_problem->mutable_cameras();

    const int num_poses = bal_problem->num_poses();
    const int pose_block_size = bal_problem->pose_block_size();
    double *poses = bal_problem->mutable_poses();

    if (params.ordering == "automatic")
        return;

    auto *ordering = new ceres::ParameterBlockOrdering;

    // The points come before the poses
    for (int i = 0; i < num_points; ++i)
        ordering->AddElementToGroup(points + point_block_size * i, 0);
    for (int i = 0; i < num_poses; ++i)
        ordering->AddElementToGroup(poses + pose_block_size * i, 1);
    /*for (int i = 0; i < num_cameras; ++i)
        ordering->AddElementToGroup(cameras + camera_block_size * i, 2);
*/
    options->linear_solver_ordering.reset(ordering);
}

void ceresBA::SetMinimizerOptions(ceres::Solver::Options *options, const BundleParams &params) {
    options->max_num_iterations = params.num_iterations;
    options->minimizer_progress_to_stdout = true;
    options->num_threads = params.num_threads;
    // options->eta = params.eta;
    // options->max_solver_time_in_seconds = params.max_solver_time;
    CHECK(StringToTrustRegionStrategyType(params.trust_region_strategy,
                                          &options->trust_region_strategy_type));
}

void ceresBA::SetSolverOptionsFromFlags(BALProblem *bal_problem,
                                        const BundleParams &params, ceres::Solver::Options *options) {
    SetMinimizerOptions(options, params);
    SetLinearSolver(options, params);
    SetOrdering(bal_problem, options, params);
}

void ceresBA::BuildProblem(BALProblem *bal_problem, ceres::Problem *problem, const BundleParams &params) {
    // Observations is 2 * num_observations long array observations
    // [u_1, u_2, ... u_n], where each u_i is two dimensional, the x
    // and y position of the observation.
    const double *observations = bal_problem->observations();
    //const int num_camera = bal_problem->num_cameras();
    //const int *cam_index = bal_problem->camera_index();
    for (int i = 0; i < bal_problem->num_observations(); ++i) {
        double *pose = bal_problem->mutable_pose_for_observation(i);
        //double *camera = bal_problem->mutable_camera_for_observation(i);
        double *point = bal_problem->mutable_point_for_observation(i);

        // If enabled use Huber's loss function.
        ceres::CostFunction *cost_function;
        ceres::LossFunction *loss_function = params.robustify ? new ceres::HuberLoss(1.0) : nullptr;
        //if (num_camera == 1 || cam_index[i] == 0) {
			//double a = observations[2 * i + 0];
			//double b = observations[2 * i + 1];
			
            cost_function = ReprojectionError::Create(observations[2 * i + 0], observations[2 * i + 1]);
			//cost_function = ReprojectionError::Create(100, 200); //It seems there is no this class??? 
           // problem->AddResidualBlock(cost_function, loss_function, pose, camera, point);
			problem->AddResidualBlock(cost_function, loss_function, pose, point);
      //  } else {
           // cost_function = StereoReprojectionError::Create(observations[2 * i + 0], observations[2 * i + 1]);
           // double *relative_pose = bal_problem->mutable_relative_pose_for_observation(i);
           // problem->AddResidualBlock(cost_function, loss_function, relative_pose, pose, camera, point);
       // }
    }
}

double ceresBA::SolveProblem(const std::string &filename, const BundleParams &params, bool printSummary) {
    BALProblem bal_problem(filename);
    return SolveProblem(bal_problem, params, printSummary);
}

double ceresBA::SolveProblem(BALProblem &bal_problem, const BundleParams &params, bool printSummary) {
    std::cout << "bal problem have " << bal_problem.num_poses() << " poses and "
              << bal_problem.num_points() << " points. " << std::endl;
    std::cout << "Forming " << bal_problem.num_observations() << " observatoins. " << std::endl;

    // store the initial 3D cloud points and camera pose..
    if (!params.initial_ply.empty())
        bal_problem.WriteToPLYFile(params.initial_ply);

    ceres::Problem problem;
    BuildProblem(&bal_problem, &problem, params);
    ceres::Solver::Options options;
    SetSolverOptionsFromFlags(&bal_problem, params, &options);
    options.gradient_tolerance = 1e-8;
    options.function_tolerance = 1e-8;
    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);
    if (printSummary)
        std::cout << summary.FullReport() << "\n";
    // write the result into a .ply file.
    if (!params.final_ply.empty())
        bal_problem.WriteToPLYFile(
                params.final_ply);

    return summary.final_cost;
}
