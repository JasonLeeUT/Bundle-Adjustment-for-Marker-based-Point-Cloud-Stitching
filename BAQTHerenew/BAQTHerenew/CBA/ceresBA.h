//
// Created by cherichy on 2019/3/3.
//

#ifndef BI_CALIB_CERESBA_H
#define BI_CALIB_CERESBA_H

#include <iostream>
#include <fstream>
#include <ceres/ceres.h>

#include "BALProblem.h"
#include "BundleParams.h"

class ceresBA {
public:
    static double SolveProblem(BALProblem &bal_problem, const BundleParams &params, bool printSummary = true);

    static double SolveProblem(const std::string &filename, const BundleParams &params, bool printSummary = true);

private:
    static void SetLinearSolver(ceres::Solver::Options *options, const BundleParams &params);

    static void SetOrdering(BALProblem *bal_problem, ceres::Solver::Options *options, const BundleParams &params);

    static void SetMinimizerOptions(ceres::Solver::Options *options, const BundleParams &params);

    static void
    SetSolverOptionsFromFlags(BALProblem *bal_problem, const BundleParams &params, ceres::Solver::Options *options);

    static void BuildProblem(BALProblem *bal_problem, ceres::Problem *problem, const BundleParams &params);
};


#endif //BI_CALIB_CERESBA_H
