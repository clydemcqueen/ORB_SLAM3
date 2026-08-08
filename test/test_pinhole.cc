#include <Eigen/Core>
#include <opencv2/core/types.hpp>

#include "CameraModels/Pinhole.h"
#include "gtest/gtest.h"

// Test fixture for Pinhole camera model tests
class PinholeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Camera parameters: fx, fy, cx, cy
    params = {525.0F, 525.0F, 319.5F, 239.5F};
    pinhole_camera = ORB_SLAM3::Pinhole(params);
  }

  std::vector<float> params;
  ORB_SLAM3::Pinhole pinhole_camera;
};

TEST_F(PinholeTest, ProjectAndUnproject) {
  // 3D point in world coordinates
  cv::Point3f point3D(1.0F, 2.0F, 3.0F);

  // Expected 2D projection
  // x' = fx * (X/Z) + cx = 525.0 * (1.0/3.0) + 319.5 = 175.0 + 319.5 = 494.5
  // y' = fy * (Y/Z) + cy = 525.0 * (2.0/3.0) + 239.5 = 350.0 + 239.5 = 589.5
  cv::Point2f expected_projection(494.5F, 589.5F);

  // Project the 3D point
  cv::Point2f projected_point = pinhole_camera.project(point3D);

  // Check if the projection is correct
  ASSERT_NEAR(projected_point.x, expected_projection.x, 1e-5);
  ASSERT_NEAR(projected_point.y, expected_projection.y, 1e-5);

  // Unproject the 2D point
  cv::Point3f unprojected_point = pinhole_camera.unproject(projected_point);

  // Unprojection is up to scale. The result should be on the Z=1 plane.
  // X = (x' - cx) / fx = (494.5 - 319.5) / 525.0 = 175.0 / 525.0 = 1.0/3.0
  // Y = (y' - cy) / fy = (589.5 - 239.5) / 525.0 = 350.0 / 525.0 = 2.0/3.0
  // Z = 1.0
  cv::Point3f expected_unprojection(1.0F / 3.0F, 2.0F / 3.0F, 1.0F);

  // Check if the unprojection is correct
  ASSERT_NEAR(unprojected_point.x, expected_unprojection.x, 1e-5);
  ASSERT_NEAR(unprojected_point.y, expected_unprojection.y, 1e-5);
  ASSERT_NEAR(unprojected_point.z, expected_unprojection.z, 1e-5);

  // The original 3D point and the unprojected point (scaled) should be the same
  ASSERT_NEAR(unprojected_point.x * point3D.z, point3D.x, 1e-5);
  ASSERT_NEAR(unprojected_point.y * point3D.z, point3D.y, 1e-5);
}

TEST_F(PinholeTest, ProjectAndUnprojectEigen) {
  // 3D point in world coordinates
  Eigen::Vector3f point3D(1.0F, 2.0F, 3.0F);

  // Expected 2D projection
  Eigen::Vector2f expected_projection(494.5F, 589.5F);

  // Project the 3D point
  Eigen::Vector2f projected_point = pinhole_camera.project(point3D);

  // Check if the projection is correct
  ASSERT_NEAR(projected_point.x(), expected_projection.x(), 1e-5);
  ASSERT_NEAR(projected_point.y(), expected_projection.y(), 1e-5);

  // Unproject the 2D point (using cv::Point2f version)
  cv::Point2f projected_point_cv(projected_point.x(), projected_point.y());
  Eigen::Vector3f unprojected_point = pinhole_camera.unprojectEig(projected_point_cv);

  // Expected unprojection (on Z=1 plane)
  Eigen::Vector3f expected_unprojection(1.0F / 3.0F, 2.0F / 3.0F, 1.0F);

  // Check if the unprojection is correct
  ASSERT_NEAR(unprojected_point.x(), expected_unprojection.x(), 1e-5);
  ASSERT_NEAR(unprojected_point.y(), expected_unprojection.y(), 1e-5);
  ASSERT_NEAR(unprojected_point.z(), expected_unprojection.z(), 1e-5);

  // The original 3D point and the unprojected point (scaled) should be the same
  ASSERT_NEAR(unprojected_point.x() * point3D.z(), point3D.x(), 1e-5);
  ASSERT_NEAR(unprojected_point.y() * point3D.z(), point3D.y(), 1e-5);
}
