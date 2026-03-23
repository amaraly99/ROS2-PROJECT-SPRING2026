#pragma once
// ─── OV²SLAM Front-End Precision Configuration ────────────────────────────────
// Controls the scalar type used for bearing vectors stored per Keypoint.
// Halving from double (8 bytes) to float (4 bytes) reduces the memory footprint
// of keypoint data by 50%, improving L2 cache utilisation on the RPi5's
// 512 KB per-core L2 cache.
//
// CRITICAL CONSTRAINT: Ceres cost functions use automatic differentiation which
// requires double.  DO NOT use Scalar inside any ceres:: cost functor or in
// Sophus::SE3d pose types that feed into the optimizer.
//
// Compile-time selection:
//   Float32 mode:  colcon build ... --cmake-args -DOV2SLAM_FLOAT32=ON
//   Float64 mode:  colcon build ...          (default — fully equivalent)
//
#ifdef OV2SLAM_USE_FLOAT32
  using Scalar = float;
  #define SCALAR_ZERO 0.0f
  #define SCALAR_ONE  1.0f
#else
  using Scalar = double;
  #define SCALAR_ZERO 0.0
  #define SCALAR_ONE  1.0
#endif
