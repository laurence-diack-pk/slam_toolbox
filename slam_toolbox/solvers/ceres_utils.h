/*
 * Copyright 2018 Simbe Robotics
 * Author: Steve Macenski
 */

#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>
#include <cmath>
#include <utility>

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
inline std::size_t GetHash(const int& x, const int& y)
{
  return ((std::hash<double>()(x) ^ (std::hash<double>()(y) << 1)) >> 1);
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

// Normalizes the angle in radians between [-pi and pi).
template <typename T> inline T NormalizeAngle(const T& angle_radians)
{
  T two_pi(2.0 * M_PI);
  return angle_radians - two_pi * ceres::floor((angle_radians + T(M_PI)) / two_pi);
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

class AngleLocalParameterization
{
 public:
  template <typename T>
  bool operator()(const T* theta_radians, const T* delta_theta_radians, T* theta_radians_plus_delta) const
  {
    *theta_radians_plus_delta = NormalizeAngle(*theta_radians + *delta_theta_radians);
    return true;
  }

  static ceres::LocalParameterization* Create()
  {
    return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization, 1, 1>);
  }
};

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

template <typename T>
Eigen::Matrix<T, 2, 2> RotationMatrix2D(T yaw_radians)
{
  const T cos_yaw = ceres::cos(yaw_radians);
  const T sin_yaw = ceres::sin(yaw_radians);
  Eigen::Matrix<T, 2, 2> rotation;
  rotation << cos_yaw, -sin_yaw, sin_yaw, cos_yaw;
  return rotation;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

class PoseGraph2dErrorTerm
{
 public:
  PoseGraph2dErrorTerm(double x_ab, double y_ab, double yaw_ab_radians, const Eigen::Matrix3d& sqrt_information)
      : p_ab_(x_ab, y_ab), yaw_ab_radians_(yaw_ab_radians), sqrt_information_(sqrt_information)
  {
  }

  template <typename T>
  bool operator()(const T* const x_a, const T* const y_a, const T* const yaw_a, const T* const x_b, const T* const y_b, const T* const yaw_b, T* residuals_ptr) const
  {
    const Eigen::Matrix<T, 2, 1> p_a(*x_a, *y_a);
    const Eigen::Matrix<T, 2, 1> p_b(*x_b, *y_b);
    Eigen::Map<Eigen::Matrix<T, 3, 1> > residuals_map(residuals_ptr);
    residuals_map.template head<2>() = RotationMatrix2D(*yaw_a).transpose() * (p_b - p_a) - p_ab_.cast<T>();
    residuals_map(2) = NormalizeAngle((*yaw_b - *yaw_a) - static_cast<T>(yaw_ab_radians_));
    // Scale the residuals by the square root information matrix to account for the measurement uncertainty.
    residuals_map = sqrt_information_.template cast<T>() * residuals_map;
    return true;
  }

  static ceres::CostFunction* Create(double x_ab, double y_ab, double yaw_ab_radians, const Eigen::Matrix3d& sqrt_information) 
  {
    return (new ceres::AutoDiffCostFunction<PoseGraph2dErrorTerm, 3, 1, 1, 1, 1, 1, 1>(new PoseGraph2dErrorTerm(x_ab, y_ab, yaw_ab_radians, sqrt_information)));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  // The position of B relative to A in the A frame.
  const Eigen::Vector2d p_ab_;
  // The orientation of frame B relative to frame A.
  const double yaw_ab_radians_;
  // The inverse square root of the measurement covariance matrix.
  const Eigen::Matrix3d sqrt_information_;
};

class GPSPoseErrorTerm
{
public:
    GPSPoseErrorTerm(
        double x_gps, double y_gps, double yaw_gps,
        const Eigen::Matrix3d& sqrt_information)
        : p_gps_(x_gps, y_gps),
          yaw_gps_(yaw_gps),
          sqrt_information_(sqrt_information)
    {
    }

    template<typename T>
    bool operator()(
        const T* const x, const T* const y, const T* const yaw,
        T* residuals) const
    {
        // Position error
        residuals[0] = *x - T(p_gps_.x());
        residuals[1] = *y - T(p_gps_.y());
        // Heading error with angle normalization
        residuals[2] = NormalizeAngle(*yaw - T(yaw_gps_));

        // Scale by the full information matrix
        Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals_map(residuals);
        residuals_map = sqrt_information_.template cast<T>() * residuals_map;
        return true;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    const Eigen::Vector2d p_gps_;
    const double yaw_gps_;
    const Eigen::Matrix3d sqrt_information_;
};

struct InterpolationErrorTerm {
    InterpolationErrorTerm(double t, double weight)
        : t_(t), weight_(weight) {}

    template <typename T>
    bool operator()(const T* const p1_x, const T* const p1_y, const T* const p1_theta,
                   const T* const p2_x, const T* const p2_y, const T* const p2_theta,
                   T* residuals) const {
        // Linear interpolation between poses
        residuals[0] = weight_ * ((*p2_x) - ((*p1_x) * (T(1.0) - t_) + (*p2_x) * t_));
        residuals[1] = weight_ * ((*p2_y) - ((*p1_y) * (T(1.0) - t_) + (*p2_y) * t_));
        residuals[2] = weight_ * ((*p2_theta) - ((*p1_theta) * (T(1.0) - t_) + (*p2_theta) * t_));
        return true;
    }

    static ceres::CostFunction* Create(double t, double weight) {
        return new ceres::AutoDiffCostFunction<InterpolationErrorTerm, 3, 1, 1, 1, 1, 1, 1>(
            new InterpolationErrorTerm(t, weight));
    }

private:
    const double t_;      // Interpolation factor (0 to 1)
    const double weight_; // Weight of the interpolation constraint
};
