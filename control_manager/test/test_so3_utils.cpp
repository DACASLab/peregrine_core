#include <control_manager/so3_utils.hpp>

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <random>

namespace
{
constexpr double kTol = 1e-12;
}

TEST(So3Utils, HatProducesSkewSymmetricMatrix)
{
  const Eigen::Vector3d v(1.0, -2.0, 3.0);
  const Eigen::Matrix3d S = control_manager::hat(v);

  // Skew-symmetric: S + S^T == 0
  const Eigen::Matrix3d sum = S + S.transpose();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(sum(i, j), 0.0, kTol);
    }
  }

  // Diagonal is zero
  EXPECT_NEAR(S(0, 0), 0.0, kTol);
  EXPECT_NEAR(S(1, 1), 0.0, kTol);
  EXPECT_NEAR(S(2, 2), 0.0, kTol);
}

TEST(So3Utils, VeeIsInverseOfHat)
{
  const Eigen::Vector3d v(3.14, -1.41, 2.72);
  const Eigen::Vector3d result = control_manager::vee(control_manager::hat(v));
  EXPECT_NEAR(result.x(), v.x(), kTol);
  EXPECT_NEAR(result.y(), v.y(), kTol);
  EXPECT_NEAR(result.z(), v.z(), kTol);
}

TEST(So3Utils, HatVeeRoundtripRandomVectors)
{
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> dist(-10.0, 10.0);

  for (int i = 0; i < 100; ++i) {
    const Eigen::Vector3d v(dist(rng), dist(rng), dist(rng));
    const Eigen::Vector3d result = control_manager::vee(control_manager::hat(v));
    EXPECT_NEAR(result.x(), v.x(), kTol);
    EXPECT_NEAR(result.y(), v.y(), kTol);
    EXPECT_NEAR(result.z(), v.z(), kTol);
  }
}

TEST(So3Utils, HatImplementsCrossProduct)
{
  const Eigen::Vector3d v(1.0, 2.0, 3.0);
  const Eigen::Vector3d u(4.0, -5.0, 6.0);

  const Eigen::Vector3d cross = v.cross(u);
  const Eigen::Vector3d hat_cross = control_manager::hat(v) * u;

  EXPECT_NEAR(hat_cross.x(), cross.x(), kTol);
  EXPECT_NEAR(hat_cross.y(), cross.y(), kTol);
  EXPECT_NEAR(hat_cross.z(), cross.z(), kTol);
}

TEST(So3Utils, HatOfZeroIsZeroMatrix)
{
  const Eigen::Matrix3d S = control_manager::hat(Eigen::Vector3d::Zero());
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(S(i, j), 0.0, kTol);
    }
  }
}

TEST(So3Utils, VeeOfZeroMatrixIsZeroVector)
{
  const Eigen::Vector3d v = control_manager::vee(Eigen::Matrix3d::Zero());
  EXPECT_NEAR(v.x(), 0.0, kTol);
  EXPECT_NEAR(v.y(), 0.0, kTol);
  EXPECT_NEAR(v.z(), 0.0, kTol);
}
