/*
 * Copyright 2018 Simbe Robotics, Inc.
 * Author: Steve Macenski (stevenmacenski@gmail.com)
 */

#ifndef KARTO_CERESSOLVER_H
#define KARTO_CERESSOLVER_H

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <vector>
#include <unordered_map>
#include <utility>

#include <karto_sdk/Mapper.h>
#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>
#include <cmath>
#include <math.h>
#include <set>

#include "../include/slam_toolbox/toolbox_types.hpp"
#include "ceres_utils.h"

namespace solver_plugins
{

using namespace ::toolbox_types;

inline Eigen::Matrix3d ConvertToEigenMatrix(const karto::Matrix3& matrix)
{
    Eigen::Matrix3d eigen_matrix;
    eigen_matrix << matrix(0,0), matrix(0,1), matrix(0,2),
                   matrix(1,0), matrix(1,1), matrix(1,2),
                   matrix(2,0), matrix(2,1), matrix(2,2);
    return eigen_matrix;
}

class CeresSolver : public karto::ScanSolver
{

public:
  CeresSolver();
  virtual ~CeresSolver();

public:
  virtual const karto::ScanSolver::IdPoseVector& GetCorrections() const; //Get corrected poses after optimization
  virtual void Compute(); //Solve
  virtual void Clear(); //Resets the corrections
  virtual void Reset(); //Resets the solver plugin clean

  virtual void AddNode(karto::Vertex<karto::LocalizedRangeScan>* pVertex); //Adds a node to the solver
  virtual void AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge); //Adds a constraint to the solver
  virtual std::unordered_map<int, Eigen::Vector3d>* getGraph(); //Get graph stored
  virtual void RemoveNode(kt_int32s id); //Removes a node from the solver correction table
  virtual void RemoveConstraint(kt_int32s sourceId, kt_int32s targetId); // Removes constraints from the optimization problem

  virtual void ModifyNode(const int& unique_id, Eigen::Vector3d pose); // change a node's pose
  virtual void GetNodeOrientation(const int& unique_id, double& pose); // get a node's current pose yaw

  void AddInterpolationConstraints(int source_id, int target_id);

private:
  // karto
  karto::ScanSolver::IdPoseVector corrections_;
  
  // gps
  std::set<int> nodes_with_gps_;
  double gps_factor_weight_;
  double interpolation_factor_;

  // ceres
  ceres::Solver::Options options_;
  ceres::Problem::Options options_problem_;
  ceres::LossFunction* loss_function_;
  ceres::Problem* problem_;
  ceres::LocalParameterization* angle_local_parameterization_;
  bool was_constant_set_, debug_logging_;

  // graph
  std::unordered_map<int, Eigen::Vector3d>* nodes_;
  std::unordered_map<size_t, ceres::ResidualBlockId>* blocks_;
  std::unordered_map<int, Eigen::Vector3d>::iterator first_node_;
  boost::mutex nodes_mutex_;
};
}

#endif
