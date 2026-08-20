#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <vector>

#include "System.h"

namespace fs = std::filesystem;

struct FrameResult {
    double timestamp = 0.0;
    int tracking_state = 0;
    int num_features = 0;
    int num_inliers = 0;
    float tx = 0.0F;
    float ty = 0.0F;
    float tz = 0.0F;
    float qx = 0.0F;
    float qy = 0.0F;
    float qz = 0.0F;
    float qw = 1.0F;
};

struct ExperimentStats {
    double clip_limit = 0.0;
    int grid_size = 0;
    int total_frames = 0;
    int baseline_init_frame = -1;
    int clahe_init_frame = -1;
    int baseline_tracked_frames = 0;
    int clahe_tracked_frames = 0;

    double avg_features_baseline = 0.0;
    double avg_features_clahe = 0.0;
    double avg_features_gain_pct = 0.0;
    double avg_inliers_baseline = 0.0;
    double avg_inliers_clahe = 0.0;
    double avg_inliers_gain_pct = 0.0;

    double pos_rmse = 0.0;
    double pos_mean_delta = 0.0;
    double pos_max_delta = 0.0;

    double rot_mean_deg = 0.0;
    double rot_max_deg = 0.0;

    double baseline_path_length = 0.0;
    double clahe_path_length = 0.0;
};

#ifndef VOCABULARY_PATH
#define VOCABULARY_PATH ""
#endif
#ifndef SETTINGS_PATH
#define SETTINGS_PATH ""
#endif
#ifndef IMAGES_PATH
#define IMAGES_PATH ""
#endif
#ifndef BASELINE_PATH
#define BASELINE_PATH ""
#endif

namespace {
void loadImages(const std::string& dir_path, std::vector<std::string>& image_files,
                std::vector<double>& timestamps) {
    for (const auto& entry : fs::directory_iterator(dir_path)) {
        if (entry.path().extension() == ".png") {
            image_files.push_back(entry.path().string());
        }
    }
    std::sort(image_files.begin(), image_files.end());

    for (const std::string& file : image_files) {
        fs::path p(file);
        std::string filename = p.stem().string();
        double timestamp = std::stod(filename) / 1e9;  // nanoseconds to seconds
        timestamps.push_back(timestamp);
    }
}

std::vector<FrameResult> loadBaseline(const std::string& baseline_file) {
    std::vector<FrameResult> baseline;
    std::ifstream f(baseline_file);
    if (!f.is_open()) {
        std::cerr << "Error: Could not open baseline file at " << baseline_file << "\n";
        return baseline;
    }

    std::string line;
    std::getline(f, line);  // Header
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        FrameResult r;
        char comma;
        std::stringstream ss(line);
        ss >> r.timestamp >> comma >> r.tracking_state >> comma >> r.num_features >> comma >>
            r.num_inliers >> comma >> r.tx >> comma >> r.ty >> comma >> r.tz >> comma >>
            r.qx >> comma >> r.qy >> comma >> r.qz >> comma >> r.qw;
        baseline.push_back(r);
    }
    return baseline;
}

cv::Mat applyCLAHE(const cv::Mat& im, cv::Ptr<cv::CLAHE>& clahe, double clip_limit) {
    cv::Mat gray;
    if (im.channels() == 3) {
        cv::cvtColor(im, gray, cv::COLOR_BGR2GRAY);
    } else if (im.channels() == 4) {
        cv::cvtColor(im, gray, cv::COLOR_BGRA2GRAY);
    } else {
        gray = im;
    }

    if (clip_limit <= 0.0 || !clahe) {
        return gray;
    }

    cv::Mat out;
    clahe->apply(gray, out);
    return out;
}

ExperimentStats runExperiment(const std::string& voc_file, const std::string& settings_file,
                              const std::vector<std::string>& image_files,
                              const std::vector<double>& timestamps,
                              const std::vector<FrameResult>& baseline, double clip_limit,
                              int grid_size, const std::string& output_csv = "") {
    ExperimentStats stats;
    stats.clip_limit = clip_limit;
    stats.grid_size = grid_size;
    stats.total_frames = static_cast<int>(image_files.size());

    cv::Ptr<cv::CLAHE> clahe = nullptr;
    if (clip_limit > 0.0) {
        clahe = cv::createCLAHE(clip_limit, cv::Size(grid_size, grid_size));
    }

    std::cout << "\n=======================================================\n";
    if (clip_limit > 0.0) {
        std::cout << " Running ORB-SLAM3 Mono with CLAHE (ClipLimit=" << clip_limit
                  << ", GridSize=" << grid_size << "x" << grid_size << ")\n";
    } else {
        std::cout << " Running ORB-SLAM3 Mono with Raw Images (No CLAHE)\n";
    }
    std::cout << "=======================================================\n";

    ORB_SLAM3::System SLAM(voc_file, settings_file, ORB_SLAM3::System::MONOCULAR, false);

    std::vector<FrameResult> clahe_results;
    clahe_results.reserve(image_files.size());

    std::vector<double> feature_counts_clahe;
    std::vector<double> feature_counts_base;
    std::vector<double> inlier_counts_clahe;
    std::vector<double> inlier_counts_base;
    std::vector<double> pos_errors_sq;
    std::vector<double> pos_deltas;
    std::vector<double> rot_deltas_deg;

    for (size_t i = 0; i < image_files.size(); ++i) {
        cv::Mat im = cv::imread(image_files[i], cv::IMREAD_UNCHANGED);
        if (im.empty()) {
            std::cerr << "Warning: Failed to load image " << image_files[i] << "\n";
            continue;
        }

        cv::Mat im_clahe = applyCLAHE(im, clahe, clip_limit);
        double tframe = timestamps[i];

        Sophus::SE3f Tcw = SLAM.TrackMonocular(im_clahe, tframe);

        FrameResult res;
        res.timestamp = tframe;
        res.tracking_state = SLAM.GetTrackingState();

        std::vector<ORB_SLAM3::MapPoint*> tracked_mps = SLAM.GetTrackedMapPoints();
        res.num_features = static_cast<int>(tracked_mps.size());
        res.num_inliers = static_cast<int>(
            std::count_if(tracked_mps.begin(), tracked_mps.end(),
                          [](ORB_SLAM3::MapPoint* p) { return p != nullptr; }));

        Eigen::Vector3f trans = Tcw.translation();
        Eigen::Quaternionf q = Tcw.unit_quaternion();
        res.tx = trans.x();
        res.ty = trans.y();
        res.tz = trans.z();
        res.qx = q.x();
        res.qy = q.y();
        res.qz = q.z();
        res.qw = q.w();

        clahe_results.push_back(res);

        if (res.tracking_state == 2 && stats.clahe_init_frame < 0) {
            stats.clahe_init_frame = static_cast<int>(i);
        }

        if (i < baseline.size()) {
            const auto& b = baseline[i];
            if (b.tracking_state == 2 && stats.baseline_init_frame < 0) {
                stats.baseline_init_frame = static_cast<int>(i);
            }
            if (b.tracking_state == 2) {
                stats.baseline_tracked_frames++;
            }
            if (res.tracking_state == 2) {
                stats.clahe_tracked_frames++;
                feature_counts_clahe.push_back(res.num_features);
                inlier_counts_clahe.push_back(res.num_inliers);
                if (b.tracking_state == 2) {
                    feature_counts_base.push_back(b.num_features);
                    inlier_counts_base.push_back(b.num_inliers);

                    // Translation delta
                    float dx = res.tx - b.tx;
                    float dy = res.ty - b.ty;
                    float dz = res.tz - b.tz;
                    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
                    pos_deltas.push_back(dist);
                    pos_errors_sq.push_back(dist * dist);

                    // Rotation delta (angle between quaternions in degrees)
                    float dot = std::abs(res.qx * b.qx + res.qy * b.qy + res.qz * b.qz + res.qw * b.qw);
                    if (dot > 1.0F) dot = 1.0F;
                    double rot_angle_rad = 2.0 * std::acos(dot);
                    double rot_angle_deg = rot_angle_rad * 180.0 / M_PI;
                    rot_deltas_deg.push_back(rot_angle_deg);
                }
            }
        }
    }

    SLAM.Shutdown();

    // Compute path lengths
    for (size_t i = 1; i < clahe_results.size(); ++i) {
        if (clahe_results[i].tracking_state == 2 && clahe_results[i - 1].tracking_state == 2) {
            float dx = clahe_results[i].tx - clahe_results[i - 1].tx;
            float dy = clahe_results[i].ty - clahe_results[i - 1].ty;
            float dz = clahe_results[i].tz - clahe_results[i - 1].tz;
            stats.clahe_path_length += std::sqrt(dx * dx + dy * dy + dz * dz);
        }
    }
    for (size_t i = 1; i < baseline.size(); ++i) {
        if (baseline[i].tracking_state == 2 && baseline[i - 1].tracking_state == 2) {
            float dx = baseline[i].tx - baseline[i - 1].tx;
            float dy = baseline[i].ty - baseline[i - 1].ty;
            float dz = baseline[i].tz - baseline[i - 1].tz;
            stats.baseline_path_length += std::sqrt(dx * dx + dy * dy + dz * dz);
        }
    }

    // Averages and stats
    if (!feature_counts_clahe.empty()) {
        stats.avg_features_clahe =
            std::accumulate(feature_counts_clahe.begin(), feature_counts_clahe.end(), 0.0) /
            feature_counts_clahe.size();
    }
    if (!inlier_counts_clahe.empty()) {
        stats.avg_inliers_clahe =
            std::accumulate(inlier_counts_clahe.begin(), inlier_counts_clahe.end(), 0.0) /
            inlier_counts_clahe.size();
    }
    if (!feature_counts_base.empty()) {
        stats.avg_features_baseline =
            std::accumulate(feature_counts_base.begin(), feature_counts_base.end(), 0.0) /
            feature_counts_base.size();
        if (stats.avg_features_baseline > 0.0) {
            stats.avg_features_gain_pct =
                ((stats.avg_features_clahe - stats.avg_features_baseline) /
                 stats.avg_features_baseline) *
                100.0;
        }
    }
    if (!inlier_counts_base.empty()) {
        stats.avg_inliers_baseline =
            std::accumulate(inlier_counts_base.begin(), inlier_counts_base.end(), 0.0) /
            inlier_counts_base.size();
        if (stats.avg_inliers_baseline > 0.0) {
            stats.avg_inliers_gain_pct =
                ((stats.avg_inliers_clahe - stats.avg_inliers_baseline) /
                 stats.avg_inliers_baseline) *
                100.0;
        }
    }

    if (!pos_errors_sq.empty()) {
        double mean_sq = std::accumulate(pos_errors_sq.begin(), pos_errors_sq.end(), 0.0) /
                         pos_errors_sq.size();
        stats.pos_rmse = std::sqrt(mean_sq);
    }
    if (!pos_deltas.empty()) {
        stats.pos_mean_delta =
            std::accumulate(pos_deltas.begin(), pos_deltas.end(), 0.0) / pos_deltas.size();
        stats.pos_max_delta = *std::max_element(pos_deltas.begin(), pos_deltas.end());
    }
    if (!rot_deltas_deg.empty()) {
        stats.rot_mean_deg =
            std::accumulate(rot_deltas_deg.begin(), rot_deltas_deg.end(), 0.0) /
            rot_deltas_deg.size();
        stats.rot_max_deg =
            *std::max_element(rot_deltas_deg.begin(), rot_deltas_deg.end());
    }

    // Print summary
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\n------------------ EXPERIMENT RESULTS ------------------\n";
    std::cout << " CLAHE Clip Limit:             " << stats.clip_limit << "\n";
    std::cout << " CLAHE Grid Size:              " << stats.grid_size << " x " << stats.grid_size
              << "\n";
    std::cout << " Total Frames:                 " << stats.total_frames << "\n";
    std::cout << " Initialization Frame:         Baseline = " << stats.baseline_init_frame
              << " | CLAHE = " << stats.clahe_init_frame << "\n";
    std::cout << " Successfully Tracked Frames:  Baseline = " << stats.baseline_tracked_frames
              << " (" << (100.0 * stats.baseline_tracked_frames / stats.total_frames) << "%)\n"
              << "                               CLAHE    = " << stats.clahe_tracked_frames
              << " (" << (100.0 * stats.clahe_tracked_frames / stats.total_frames) << "%)\n";
    std::cout << " Average Tracked Features:     Baseline = " << stats.avg_features_baseline
              << "\n"
              << "                               CLAHE    = " << stats.avg_features_clahe
              << " (Gain: " << (stats.avg_features_gain_pct >= 0 ? "+" : "")
              << stats.avg_features_gain_pct << "%)\n";
    std::cout << " Average Matched Inliers:      Baseline = " << stats.avg_inliers_baseline
              << "\n"
              << "                               CLAHE    = " << stats.avg_inliers_clahe
              << " (Gain: " << (stats.avg_inliers_gain_pct >= 0 ? "+" : "")
              << stats.avg_inliers_gain_pct << "%)\n";
    std::cout << " Trajectory Position Delta:    RMSE     = " << stats.pos_rmse << " m\n"
              << "                               Mean     = " << stats.pos_mean_delta << " m\n"
              << "                               Max      = " << stats.pos_max_delta << " m\n";
    std::cout << " Trajectory Rotation Delta:    Mean     = " << stats.rot_mean_deg << " deg\n"
              << "                               Max      = " << stats.rot_max_deg << " deg\n";
    std::cout << " Total Path Length:            Baseline = " << stats.baseline_path_length << " m\n"
              << "                               CLAHE    = " << stats.clahe_path_length << " m\n";
    std::cout << "--------------------------------------------------------\n";

    // Optional CSV export
    if (!output_csv.empty()) {
        std::ofstream out(output_csv);
        if (out.is_open()) {
            out << "frame_idx,timestamp,state_base,state_clahe,features_base,features_clahe,inliers_base,inliers_clahe,"
                << "tx_base,ty_base,tz_base,tx_clahe,ty_clahe,tz_clahe,pos_delta_m,rot_delta_deg\n";
            for (size_t i = 0; i < clahe_results.size(); ++i) {
                const auto& c = clahe_results[i];
                int b_state = (i < baseline.size()) ? baseline[i].tracking_state : 0;
                int b_feat = (i < baseline.size()) ? baseline[i].num_features : 0;
                int b_inliers = (i < baseline.size()) ? baseline[i].num_inliers : 0;
                float b_tx = (i < baseline.size()) ? baseline[i].tx : 0.0F;
                float b_ty = (i < baseline.size()) ? baseline[i].ty : 0.0F;
                float b_tz = (i < baseline.size()) ? baseline[i].tz : 0.0F;

                double pos_d = 0.0;
                double rot_d = 0.0;
                if (i < baseline.size() && b_state == 2 && c.tracking_state == 2) {
                    float dx = c.tx - b_tx;
                    float dy = c.ty - b_ty;
                    float dz = c.tz - b_tz;
                    pos_d = std::sqrt(dx * dx + dy * dy + dz * dz);

                    float dot = std::abs(c.qx * baseline[i].qx + c.qy * baseline[i].qy +
                                         c.qz * baseline[i].qz + c.qw * baseline[i].qw);
                    if (dot > 1.0F) dot = 1.0F;
                    rot_d = 2.0 * std::acos(dot) * 180.0 / M_PI;
                }

                out << i << "," << std::fixed << std::setprecision(9) << c.timestamp << ","
                    << b_state << "," << c.tracking_state << "," << b_feat << "," << c.num_features
                    << "," << b_inliers << "," << c.num_inliers << "," << std::setprecision(6)
                    << b_tx << "," << b_ty << "," << b_tz << "," << c.tx << "," << c.ty << ","
                    << c.tz << "," << pos_d << "," << rot_d << "\n";
            }
            out.close();
            std::cout << "Saved per-frame comparison to " << output_csv << "\n";
        }
    }

    return stats;
}

void printHelp(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n\n"
              << "Options:\n"
              << "  --clip-limit <float>   CLAHE clip limit (default: 3.0)\n"
              << "  --grid-size <int>      CLAHE tile grid size NxN (default: 8)\n"
              << "  --sweep                Run parameter sweep over various clip limits and grid sizes\n"
              << "  --output-csv <file>    Path to write per-frame comparison CSV\n"
              << "  --images-dir <dir>     Path to folder of test PNG images\n"
              << "  --baseline <file>      Path to baseline.csv\n"
              << "  --settings <file>      Path to camera/ORB settings YAML\n"
              << "  --vocabulary <file>    Path to ORBvoc.txt\n"
              << "  -h, --help             Show this help message\n";
}
}  // namespace

int main(int argc, char** argv) {
    double clip_limit = 3.0;
    int grid_size = 8;
    bool do_sweep = false;
    std::string output_csv = "";
    std::string images_dir = IMAGES_PATH;
    std::string baseline_file = BASELINE_PATH;
    std::string settings_file = SETTINGS_PATH;
    std::string voc_file = VOCABULARY_PATH;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--clip-limit" && i + 1 < argc) {
            clip_limit = std::stod(argv[++i]);
        } else if (arg == "--grid-size" && i + 1 < argc) {
            grid_size = std::stoi(argv[++i]);
        } else if (arg == "--sweep") {
            do_sweep = true;
        } else if (arg == "--output-csv" && i + 1 < argc) {
            output_csv = argv[++i];
        } else if (arg == "--images-dir" && i + 1 < argc) {
            images_dir = argv[++i];
        } else if (arg == "--baseline" && i + 1 < argc) {
            baseline_file = argv[++i];
        } else if (arg == "--settings" && i + 1 < argc) {
            settings_file = argv[++i];
        } else if (arg == "--vocabulary" && i + 1 < argc) {
            voc_file = argv[++i];
        } else if (arg == "-h" || arg == "--help") {
            printHelp(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            printHelp(argv[0]);
            return 1;
        }
    }

    std::cout << "Starting CLAHE Mono SLAM Evaluation\n";
    std::cout << "Vocabulary: " << voc_file << "\n";
    std::cout << "Settings:   " << settings_file << "\n";
    std::cout << "Images:     " << images_dir << "\n";
    std::cout << "Baseline:   " << baseline_file << "\n";

    std::vector<std::string> image_files;
    std::vector<double> timestamps;
    loadImages(images_dir, image_files, timestamps);

    if (image_files.empty()) {
        std::cerr << "Error: No images found in " << images_dir << "\n";
        return 1;
    }
    std::cout << "Loaded " << image_files.size() << " images.\n";

    std::vector<FrameResult> baseline = loadBaseline(baseline_file);
    if (baseline.empty()) {
        std::cerr << "Error: Baseline could not be loaded.\n";
        return 1;
    }
    std::cout << "Loaded baseline with " << baseline.size() << " frames.\n";

    if (!do_sweep) {
        runExperiment(voc_file, settings_file, image_files, timestamps, baseline, clip_limit,
                      grid_size, output_csv);
    } else {
        std::vector<double> sweep_clips = {0.0, 1.0, 2.0, 3.0, 4.0, 6.0};
        std::vector<int> sweep_grids = {4, 8, 16};
        std::vector<ExperimentStats> sweep_results;

        // Run Raw baseline first
        sweep_results.push_back(runExperiment(voc_file, settings_file, image_files,
                                              timestamps, baseline, 0.0, 0, ""));

        for (int g : sweep_grids) {
            for (double c : sweep_clips) {
                if (c == 0.0) continue;  // Already ran raw
                sweep_results.push_back(runExperiment(voc_file, settings_file, image_files,
                                                      timestamps, baseline, c, g, ""));
            }
        }

        std::cout << "\n\n";
        std::cout << "=========================================================================================================================\n";
        std::cout << "                                                 PARAMETER SWEEP SUMMARY                                                 \n";
        std::cout << "=========================================================================================================================\n";
        std::cout << std::left << std::setw(8) << "Clip" << std::setw(8) << "Grid"
                  << std::setw(12) << "Init Frame" << std::setw(14) << "Tracked (OK)"
                  << std::setw(14) << "Avg Inliers" << std::setw(14) << "Inlier Gain"
                  << std::setw(14) << "Avg Features"
                  << std::setw(14) << "Pos RMSE (m)" << std::setw(14) << "Pos Max (m)" << "\n";
        std::cout << "-------------------------------------------------------------------------------------------------------------------------\n";

        for (const auto& s : sweep_results) {
            std::string grid_str = (s.clip_limit > 0.0) ? (std::to_string(s.grid_size) + "x" + std::to_string(s.grid_size)) : "None";
            std::string clip_str = (s.clip_limit > 0.0) ? std::to_string(s.clip_limit).substr(0, 3) : "Raw";
            std::string inlier_gain_str = (s.clip_limit > 0.0)
                ? ((s.avg_inliers_gain_pct >= 0 ? "+" : "") + std::to_string(s.avg_inliers_gain_pct).substr(0, 5) + "%")
                : "0.0%";
            std::cout << std::fixed << std::setprecision(1) << std::left << std::setw(8)
                      << clip_str << std::setw(8) << grid_str
                      << std::setw(12) << s.clahe_init_frame << std::setw(14)
                      << s.clahe_tracked_frames << std::setprecision(1) << std::setw(14)
                      << s.avg_inliers_clahe << std::setw(14) << inlier_gain_str
                      << std::setprecision(1) << std::setw(14)
                      << s.avg_features_clahe << std::setprecision(4)
                      << std::setw(14) << s.pos_rmse << std::setw(14) << s.pos_max_delta << "\n";
        }
        std::cout << "=========================================================================================================================\n";
    }

    return 0;
}
