// bundle_adjust_tag_window.cpp
//
// Build with Ceres + OpenCV (OpenCV only used for convenient types; you may
// remove).
//
// Basic idea:
// - We optimize per-frame tag poses (angle-axis(3) + t(3))
// - For each corner observation, add reprojection residual
// - Optionally add a Z-height constraint residual (penalty) for each frame's
// translation.z
//
// Note: You must populate `observations` and initial `poses` from your data
// (solvePnP results).
//
#pragma once
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "../include/calib_base.h"
#include "apriltag_batch_process.h"
// --------------------------- Cost functor ---------------------------
struct ReprojectionError {
  ReprojectionError(const cv::Point2d& observed_px,
                    const cv::Point3d& point_tag, double fx, double fy,
                    double cx, double cy,
                    const std::array<double, 5>& dist_coeffs)
      : observed_(observed_px),
        point_tag_(point_tag),
        fx_(fx),
        fy_(fy),
        cx_(cx),
        cy_(cy),
        k1_(dist_coeffs[0]),
        k2_(dist_coeffs[1]),
        p1_(dist_coeffs[2]),
        p2_(dist_coeffs[3]),
        k3_(dist_coeffs[4]) {}

  template <typename T>
  bool operator()(const T* const angle_axis,  // 3
                  const T* const trans,       // 3
                  T* residuals) const {
    // Rotate point from tag frame to camera frame using angle-axis
    T p[3];
    T point[3];
    point[0] = T(point_tag_.x);
    point[1] = T(point_tag_.y);
    point[2] = T(point_tag_.z);

    ceres::AngleAxisRotatePoint(angle_axis, point, p);

    // Apply translation
    p[0] += trans[0];
    p[1] += trans[1];
    p[2] += trans[2];

    // project to normalized coordinates
    T xp = p[0] / p[2];
    T yp = p[1] / p[2];

    // radial distortion
    T r2 = xp * xp + yp * yp;
    T r4 = r2 * r2;
    T r6 = r4 * r2;
    T radial = T(1.0) + T(k1_) * r2 + T(k2_) * r4 + T(k3_) * r6;

    // tangential
    T x_tang = T(2.0) * T(p1_) * xp * yp + T(p2_) * (r2 + T(2.0) * xp * xp);
    T y_tang = T(p1_) * (r2 + T(2.0) * yp * yp) + T(2.0) * T(p2_) * xp * yp;

    T x_distorted = xp * radial + x_tang;
    T y_distorted = yp * radial + y_tang;

    // to pixel
    T u = T(fx_) * x_distorted + T(cx_);
    T v = T(fy_) * y_distorted + T(cy_);

    // residuals: reprojection error
    residuals[0] = u - T(observed_.x);
    residuals[1] = v - T(observed_.y);

    return true;
  }

  static ceres::CostFunction* Create(const cv::Point2d& observed_px,
                                     const cv::Point3d& point_tag, double fx,
                                     double fy, double cx, double cy,
                                     const std::array<double, 5>& dist_coeffs) {
    // angle-axis (3) + trans (3) -> 2 residuals
    return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 3, 3>(
        new ReprojectionError(observed_px, point_tag, fx, fy, cx, cy,
                              dist_coeffs)));
  }

  cv::Point2d observed_;
  cv::Point3d point_tag_;
  double fx_, fy_, cx_, cy_;
  double k1_, k2_, p1_, p2_, k3_;
};

// --------------------------- Height constraint functor
// ---------------------------
struct HeightConstraint {
  HeightConstraint(double desired_z, double weight)
      : z0_(desired_z), w_(weight) {}
  template <typename T>
  bool operator()(const T* const trans, T* residual) const {
    // trans[2] is z
    residual[0] = T(w_) * (trans[2] - T(z0_));
    return true;
  }
  static ceres::CostFunction* Create(double desired_z, double weight) {
    return (new ceres::AutoDiffCostFunction<HeightConstraint, 1, 3>(
        new HeightConstraint(desired_z, weight)));
  }
  double z0_;
  double w_;
};

// --------------------------- Data structures ---------------------------

// // Observations for one frame: 4 corners (order must match object points
// order) struct FrameObservation {
//   // image points for corners (e.g., order: [rt, lt, lb, rb] or as your
//   detector
//   // uses)
//   std::vector<cv::Point2d> corners_px;  // expected size 4
// };

// // Simple helper to create tag corner 3D points in tag-local coords
// inline std::vector<cv::Point3d> MakeTagCorners3D(double tag_size_m) {
//   double hs = tag_size_m / 2.0;
//   // Match the order used by your detector. Ensure consistent order.
//   // Here we follow: 0: right-top, 1: left-top, 2: left-bottom, 3:
//   right-bottom std::vector<cv::Point3d> pts = {
//       {hs, hs, 0.0}, {-hs, hs, 0.0}, {-hs, -hs, 0.0}, {hs, -hs, 0.0}};
//   return pts;
// }

// --------------------------- Main example ---------------------------

// int main(int argc, char** argv) {
//   google::InitGoogleLogging(argv[0]);
//   // ---------- USER DATA TODO ----------
//   // Replace the following placeholders with your actual data load routine.
//   // - observations: vector<FrameObservation> length = Nframes (sliding
//   window)
//   // - initial poses: for each frame, an initial angle-axis (3) and trans (3)
//   // array
//   //   e.g., from solvePnP results:
//   //
//   // Here we show a dummy structure; the user must fill these from real data.
//   std::vector<FrameObservation> observations;
//   std::vector<std::array<double, 6>>
//       poses_initial;  // [angle_axis(3), trans(3)]

//   // Example: populate two dummy frames (YOU MUST REPLACE WITH REAL)
//   // observations.resize(N); poses_initial.resize(N);
//   //
//   // ---------- END USER DATA TODO ----------

//   // For demonstration, we'll abort if no data provided.
//   if (observations.empty() || poses_initial.empty()) {
//     std::cerr << "No data provided. Fill observations[] and poses_initial[] "
//                  "from your pipeline.\n";
//     std::cerr << "Each frame should have 4 corner pixel observations, and one
//     "
//                  "initial pose (angle-axis + tvec).\n";
//     return -1;
//   }

//   const int N = static_cast<int>(observations.size());

//   // Camera intrinsics (fill with your real intrinsics; ensure they match the
//   // cropped image)
//   double fx = 0, fy = 0, cx = 0, cy = 0;
//   // Example:
//   // fx = 1400; fy = 1400; cx = 1224; cy = 1024;
//   //
//   // You must set these values appropriately (e.g., from cameraMatrix).
//   // If you have distortion coefficients, fill them (k1,k2,p1,p2,k3).
//   std::array<double, 5> dist = {0, 0, 0, 0, 0};  // fill real distCoeffs

//   // Tag size in meters (e.g., 0.10 m)
//   double tag_size_m = 0.10;

//   // Height constraint parameters: desired Z (in same units as translation)
//   and
//   // weight If your translations are in meters and you want to constrain z to
//   // 0.215 m (21.5 cm):
//   double desired_tag_z = 0.215;  // meters
//   double height_weight =
//       1000.0;  // large weight -> strong constraint. Tune as needed.

//   // Build the problem
//   ceres::Problem problem;

//   // Parameter blocks: for each frame we have angle-axis (3) and translation
//   // (3). We'll store pointers into a contiguous vector for convenience.
//   std::vector<std::array<double, 3>> angle_axis_params(N);
//   std::vector<std::array<double, 3>> trans_params(N);
//   for (int i = 0; i < N; ++i) {
//     // initialize with provided initial values
//     angle_axis_params[i][0] = poses_initial[i][0];
//     angle_axis_params[i][1] = poses_initial[i][1];
//     angle_axis_params[i][2] = poses_initial[i][2];

//     trans_params[i][0] = poses_initial[i][3];
//     trans_params[i][1] = poses_initial[i][4];
//     trans_params[i][2] = poses_initial[i][5];

//     // Add parameter blocks to problem
//     problem.AddParameterBlock(angle_axis_params[i].data(), 3);
//     problem.AddParameterBlock(trans_params[i].data(), 3);
//   }

//   // Optionally, if you want to keep some frames fixed (e.g., first frame as
//   // anchor), set them constant:
//   // problem.SetParameterBlockConstant(angle_axis_params[0].data());
//   // problem.SetParameterBlockConstant(trans_params[0].data());

//   // Prepare object points for a tag (same for all frames since tag-local
//   coords
//   // are fixed)
//   std::vector<cv::Point3d> tag_corners = MakeTagCorners3D(tag_size_m);

//   // Add reprojection residuals for each observed corner in each frame
//   for (int i = 0; i < N; ++i) {
//     const auto& obs = observations[i];
//     if (obs.corners_px.size() != tag_corners.size()) {
//       std::cerr << "Frame " << i
//                 << " has unexpected corners count: " << obs.corners_px.size()
//                 << "\n";
//       return -1;
//     }

//     for (size_t k = 0; k < tag_corners.size(); ++k) {
//       const cv::Point2d observed = obs.corners_px[k];
//       const cv::Point3d pt3 = tag_corners[k];

//       ceres::CostFunction* cost =
//           ReprojectionError::Create(observed, pt3, fx, fy, cx, cy, dist);
//       // robust loss
//       ceres::LossFunction* loss = new ceres::HuberLoss(1.0);

//       problem.AddResidualBlock(cost, loss, angle_axis_params[i].data(),
//                                trans_params[i].data());
//     }

//     // Add height constraint residual on translation z: big weight to enforce
//     z
//     // ~ desired_tag_z
//     ceres::CostFunction* height_cost =
//         HeightConstraint::Create(desired_tag_z, height_weight);
//     problem.AddResidualBlock(height_cost, nullptr, trans_params[i].data());
//     // if you want robust on height, provide a loss instead of nullptr
//   }

//   // Solver options
//   ceres::Solver::Options options;
//   options.max_num_iterations = 200;
//   options.linear_solver_type = ceres::DENSE_SCHUR;  // small problem -> ok
//   options.minimizer_progress_to_stdout = true;
//   options.num_threads = 4;

//   ceres::Solver::Summary summary;
//   ceres::Solve(options, &problem, &summary);
//   std::cout << summary.FullReport() << "\n";

//   // Output optimized poses
//   for (int i = 0; i < N; ++i) {
//     std::cout << "Frame " << i << " optimized angle-axis: ["
//               << angle_axis_params[i][0] << ", " << angle_axis_params[i][1]
//               << ", " << angle_axis_params[i][2] << "] \n";
//     std::cout << "           optimized trans: [" << trans_params[i][0] << ",
//     "
//               << trans_params[i][1] << ", " << trans_params[i][2] << "] \n";
//   }

//   return 0;
// }
