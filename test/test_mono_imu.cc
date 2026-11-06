#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "System.h"

namespace fs = std::filesystem;

struct FrameResult {
    double timestamp;
    int tracking_state;
    int num_features;
    int num_inliers;
    float tx, ty, tz, qx, qy, qz, qw;
};

namespace {
const std::string VOC_FILE = VOCABULARY_PATH;
const std::string SETTINGS_FILE = SETTINGS_PATH;
const std::string IMAGES_DIR = IMAGES_PATH;
const std::string IMU_FILE = IMU_PATH;
const std::string BASELINE_FILE = BASELINE_PATH;

void loadImages(const std::string& dir_path, std::vector<std::string>& image_files,
                std::vector<double>& timestamps) {
    std::vector<std::pair<double, std::string>> entries;
    for (const auto& entry : fs::directory_iterator(dir_path)) {
        if (entry.path().extension() == ".png") {
            const std::string path_str = entry.path().string();
            const double timestamp = std::stod(entry.path().stem().string()) / 1e9;
            entries.emplace_back(timestamp, path_str);
        }
    }
    std::sort(entries.begin(), entries.end());

    image_files.reserve(entries.size());
    timestamps.reserve(entries.size());
    for (const auto& [timestamp, file] : entries) {
        image_files.push_back(file);
        timestamps.push_back(timestamp);
    }
}

void loadImu(const std::string& csv_path, std::vector<ORB_SLAM3::IMU::Point>& imu_data) {
    std::ifstream f(csv_path);
    ASSERT_TRUE(f.is_open()) << "Failed to open IMU file at " << csv_path;

    std::string line;
    // Skip header: timestamp_ns,gx,gy,gz,ax,ay,az
    std::getline(f, line);
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        double ts_ns = 0.0;
        float gx = 0.0F;
        float gy = 0.0F;
        float gz = 0.0F;
        float ax = 0.0F;
        float ay = 0.0F;
        float az = 0.0F;
        char comma = ',';
        std::stringstream ss(line);
        ss >> ts_ns >> comma >> gx >> comma >> gy >> comma >> gz >> comma >> ax >> comma >> ay >>
            comma >> az;
        imu_data.emplace_back(ax, ay, az, gx, gy, gz, ts_ns / 1e9);
    }
}
}  // namespace

TEST(ORBSLAM3, MonoImuGazebo) {
    std::vector<std::string> image_files;
    std::vector<double> timestamps;
    loadImages(IMAGES_DIR, image_files, timestamps);
    ASSERT_FALSE(image_files.empty()) << "No images found in " << IMAGES_DIR;
    ASSERT_LE(image_files.size(), 500) << "Expected up to 500 images.";

    std::vector<ORB_SLAM3::IMU::Point> all_imu;
    loadImu(IMU_FILE, all_imu);
    ASSERT_FALSE(all_imu.empty()) << "No IMU data loaded from " << IMU_FILE;

    bool create_baseline = !fs::exists(BASELINE_FILE);
    std::vector<FrameResult> baseline;

    if (!create_baseline) {
        std::cout << "Reading baseline from " << BASELINE_FILE << "\n";
        std::ifstream f(BASELINE_FILE);
        ASSERT_TRUE(f.is_open());
        std::string line;
        std::getline(f, line);
        while (std::getline(f, line)) {
            FrameResult r{};
            char comma = ',';
            std::stringstream ss(line);
            ss >> r.timestamp >> comma >> r.tracking_state >> comma >> r.num_features >> comma >>
                r.num_inliers >> comma >> r.tx >> comma >> r.ty >> comma >> r.tz >> comma >> r.qx >>
                comma >> r.qy >> comma >> r.qz >> comma >> r.qw;
            baseline.push_back(r);
        }
        f.close();
        ASSERT_EQ(baseline.size(), image_files.size())
            << "Baseline size mismatch with image count.";
    } else {
        std::cout << "Baseline file not found. Creating one at " << BASELINE_FILE << "\n";
    }

    ORB_SLAM3::System SLAM(VOC_FILE, SETTINGS_FILE, ORB_SLAM3::System::IMU_MONOCULAR, false);

    std::ofstream f_out;
    if (create_baseline) {
        f_out.open(BASELINE_FILE);
        f_out << std::fixed << std::setprecision(9);
        f_out << "timestamp,tracking_state,num_features,num_inliers,tx,ty,tz,qx,qy,qz,qw\n";
    }

    // Start at the last IMU sample at or before the first image timestamp
    size_t imu_idx = 0;
    while (imu_idx < all_imu.size() && all_imu[imu_idx].t <= timestamps[0]) {
        imu_idx++;
    }
    if (imu_idx > 0) {
        imu_idx--;
    }

    // Verify continuous tracking after visual init, and verify metric displacement + orientation
    // once VIBA 2 has completed (anchoring translation to cancel FullInertialBA's unpinned 3D
    // world-origin gauge).
    float pos_tolerance = 0.50F;
    float rot_tolerance = 0.08F;
    int num_features_tolerance = 150;
    int inliers_tolerance = 350;
    bool has_anchor = false;
    Eigen::Vector3f pwc_anchor = Eigen::Vector3f::Zero();
    Eigen::Vector3f pwc_base_anchor = Eigen::Vector3f::Zero();

    std::vector<ORB_SLAM3::IMU::Point> frame_imu;
    for (size_t i = 0; i < image_files.size(); i++) {
        cv::Mat im = cv::imread(image_files[i], cv::IMREAD_UNCHANGED);
        ASSERT_FALSE(im.empty()) << "Failed to load image at " << image_files[i];

        const double tframe = timestamps[i];
        frame_imu.clear();
        if (i > 0) {
            while (imu_idx < all_imu.size() && all_imu[imu_idx].t <= tframe) {
                frame_imu.push_back(all_imu[imu_idx]);
                imu_idx++;
            }
        }

        const auto t_start = std::chrono::steady_clock::now();
        Sophus::SE3f Tcw = SLAM.TrackMonocular(im, tframe, frame_imu);
        const auto t_end = std::chrono::steady_clock::now();

        int state = SLAM.GetTrackingState();
        std::vector<ORB_SLAM3::MapPoint*> tracked_mps = SLAM.GetTrackedMapPoints();
        int num_features = static_cast<int>(tracked_mps.size());
        int num_inliers =
            static_cast<int>(std::count_if(tracked_mps.begin(), tracked_mps.end(),
                                           [](ORB_SLAM3::MapPoint* p) { return p != nullptr; }));

        Eigen::Vector3f trans = Tcw.translation();
        const Eigen::Quaternionf& q = Tcw.unit_quaternion();

        if (create_baseline) {
            f_out << tframe << "," << state << "," << num_features << "," << num_inliers << ","
                  << trans.x() << "," << trans.y() << "," << trans.z() << "," << q.x() << ","
                  << q.y() << "," << q.z() << "," << q.w() << "\n";
        } else {
            const FrameResult& b = baseline[i];

            // Once visual initialization has settled, verify tracking never drops across IMU init /
            // VIBA
            if (i >= 45 && b.tracking_state == 2) {
                EXPECT_EQ(state, b.tracking_state) << "Tracking state mismatch at frame " << i;
                EXPECT_NEAR(num_features, b.num_features, num_features_tolerance)
                    << "Feature count mismatch at frame " << i;
                EXPECT_NEAR(num_inliers, b.num_inliers, inliers_tolerance)
                    << "Inlier count mismatch at frame " << i;
            }

            // Compare metric trajectory and gravity-aligned orientation once VIBA 2 has completed
            if (SLAM.GetTimeFromIMUInit() > 18.0 && state == 2 && b.tracking_state == 2) {
                const Eigen::Vector3f pwc = Tcw.inverse().translation();
                const Eigen::Quaternionf q_base(b.qw, b.qx, b.qy, b.qz);
                const Eigen::Vector3f pwc_base =
                    -(q_base.conjugate() * Eigen::Vector3f(b.tx, b.ty, b.tz));

                if (!has_anchor) {
                    pwc_anchor = pwc;
                    pwc_base_anchor = pwc_base;
                    has_anchor = true;
                }

                const Eigen::Vector3f disp = pwc - pwc_anchor;
                const Eigen::Vector3f disp_base = pwc_base - pwc_base_anchor;
                EXPECT_NEAR(disp.x(), disp_base.x(), pos_tolerance) << "dx mismatch at frame " << i;
                EXPECT_NEAR(disp.y(), disp_base.y(), pos_tolerance) << "dy mismatch at frame " << i;
                EXPECT_NEAR(disp.z(), disp_base.z(), pos_tolerance) << "dz mismatch at frame " << i;

                // Handle quaternion double-cover (q and -q represent the same rotation)
                const float dot = (q.x() * b.qx) + (q.y() * b.qy) + (q.z() * b.qz) + (q.w() * b.qw);
                const float sign = dot < 0.0F ? -1.0F : 1.0F;
                EXPECT_NEAR(sign * q.x(), b.qx, rot_tolerance) << "qx mismatch at frame " << i;
                EXPECT_NEAR(sign * q.y(), b.qy, rot_tolerance) << "qy mismatch at frame " << i;
                EXPECT_NEAR(sign * q.z(), b.qz, rot_tolerance) << "qz mismatch at frame " << i;
                EXPECT_NEAR(sign * q.w(), b.qw, rot_tolerance) << "qw mismatch at frame " << i;
            }
        }

        // Pace at 2x real-time so the background LocalMapping thread can run InitializeIMU and BA
        // deterministically relative to incoming frames.
        if (i + 1 < timestamps.size()) {
            const double frame_dt = (timestamps[i + 1] - tframe) * 0.5;
            const double elapsed = std::chrono::duration<double>(t_end - t_start).count();
            if (elapsed < frame_dt) {
                std::this_thread::sleep_for(std::chrono::duration<double>(frame_dt - elapsed));
            }
        }
    }

    EXPECT_GT(SLAM.GetTimeFromIMUInit(), 25.0)
        << "IMU initialization + VIBA 2 did not complete during the sequence!";

    if (create_baseline) {
        f_out.close();
        std::cout << "Baseline creation complete!\n";
    }

    SLAM.Shutdown();
}
