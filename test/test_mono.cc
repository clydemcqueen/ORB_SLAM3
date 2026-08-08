#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <vector>

#include "System.h"

namespace fs = std::filesystem;

struct FrameResult {
    double timestamp;
    int tracking_state;
    int num_features;
    float tx, ty, tz, qx, qy, qz, qw;
};

namespace {
// Global variables to hold paths passed via macros
const std::string VOC_FILE = VOCABULARY_PATH;
const std::string SETTINGS_FILE = SETTINGS_PATH;
const std::string IMAGES_DIR = IMAGES_PATH;
const std::string BASELINE_FILE = BASELINE_PATH;

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void loadImages(const std::string& dir_path, std::vector<std::string>& image_files,
                std::vector<double>& timestamps) {
    for (const auto& entry : fs::directory_iterator(dir_path)) {
        if (entry.path().extension() == ".png") {
            image_files.push_back(entry.path().string());
        }
    }
    // Sort to ensure sequential processing
    std::sort(image_files.begin(), image_files.end());

    for (const std::string& file : image_files) {
        fs::path p(file);
        std::string filename = p.stem().string();
        double timestamp = std::stod(filename) / 1e9;  // nanoseconds to seconds
        timestamps.push_back(timestamp);
    }
}
}  // namespace

TEST(ORBSLAM3, MonoEvaluationTest) {
    std::vector<std::string> image_files;
    std::vector<double> timestamps;
    loadImages(IMAGES_DIR, image_files, timestamps);
    ASSERT_FALSE(image_files.empty()) << "No images found in " << IMAGES_DIR;
    ASSERT_LE(image_files.size(), 500) << "Expected up to 500 images.";

    // Check if baseline exists
    bool create_baseline = !fs::exists(BASELINE_FILE);
    std::vector<FrameResult> baseline;

    if (!create_baseline) {
        std::cout << "Reading baseline from " << BASELINE_FILE << "\n";
        std::ifstream f(BASELINE_FILE);
        ASSERT_TRUE(f.is_open());
        std::string line;
        // Read header
        std::getline(f, line);
        while (std::getline(f, line)) {
            FrameResult r;
            // timestamp,tracking_state,num_features,tx,ty,tz,qx,qy,qz,qw
            char comma;
            std::stringstream ss(line);
            ss >> r.timestamp >> comma >> r.tracking_state >> comma >> r.num_features >> comma >>
                r.tx >> comma >> r.ty >> comma >> r.tz >> comma >> r.qx >> comma >> r.qy >> comma >>
                r.qz >> comma >> r.qw;
            baseline.push_back(r);
        }
        f.close();
        ASSERT_EQ(baseline.size(), image_files.size())
            << "Baseline size mismatch with image count.";
    } else {
        std::cout << "Baseline file not found. Creating one at " << BASELINE_FILE << "\n";
    }

    // Initialize ORB_SLAM3
    ORB_SLAM3::System SLAM(VOC_FILE, SETTINGS_FILE, ORB_SLAM3::System::MONOCULAR, false);

    std::ofstream f_out;
    if (create_baseline) {
        f_out.open(BASELINE_FILE);
        f_out << std::fixed << std::setprecision(9);
        f_out << "timestamp,tracking_state,num_features,tx,ty,tz,qx,qy,qz,qw\n";
    }

    float pos_tolerance =
        0.1F;  // Increased to 0.1m due to non-determinism and monocular scale drift
    float rot_tolerance = 0.05F;
    int num_features_tolerance = 100;  // TODO this should be much tighter!

    for (size_t i = 0; i < image_files.size(); i++) {
        cv::Mat im = cv::imread(image_files[i], cv::IMREAD_UNCHANGED);
        ASSERT_FALSE(im.empty()) << "Failed to load image at " << image_files[i];

        double tframe = timestamps[i];
        Sophus::SE3f Tcw = SLAM.TrackMonocular(im, tframe);

        int state = SLAM.GetTrackingState();
        int num_features = static_cast<int>(SLAM.GetTrackedMapPoints().size());

        Eigen::Vector3f trans = Tcw.translation();
        Eigen::Quaternionf q = Tcw.unit_quaternion();

        if (create_baseline) {
            f_out << tframe << "," << state << "," << num_features << "," << trans.x() << ","
                  << trans.y() << "," << trans.z() << "," << q.x() << "," << q.y() << "," << q.z()
                  << "," << q.w() << "\n";
        } else {
            const FrameResult& b = baseline[i];

            // Allow state 2 (Tracking) to be exactly equal
            // Actually, we should expect the tracking state to be the same, or at least if baseline
            // was tracking, test should track.
            EXPECT_EQ(state, b.tracking_state) << "Tracking state mismatch at frame " << i;

            if (b.tracking_state == 2) {  // 2 = OK
                EXPECT_NEAR(trans.x(), b.tx, pos_tolerance) << "tx mismatch at frame " << i;
                EXPECT_NEAR(trans.y(), b.ty, pos_tolerance) << "ty mismatch at frame " << i;
                EXPECT_NEAR(trans.z(), b.tz, pos_tolerance) << "tz mismatch at frame " << i;

                EXPECT_NEAR(q.x(), b.qx, rot_tolerance) << "qx mismatch at frame " << i;
                EXPECT_NEAR(q.y(), b.qy, rot_tolerance) << "qy mismatch at frame " << i;
                EXPECT_NEAR(q.z(), b.qz, rot_tolerance) << "qz mismatch at frame " << i;
                EXPECT_NEAR(q.w(), b.qw, rot_tolerance) << "qw mismatch at frame " << i;

                // Feature counts might fluctuate slightly depending on optimizations or compiler
                // differences
                EXPECT_NEAR(num_features, b.num_features, num_features_tolerance)
                    << "Feature count mismatch at frame " << i;
            }
        }
    }

    if (create_baseline) {
        f_out.close();
        std::cout << "Baseline creation complete!\n";
    }

    SLAM.Shutdown();
}
