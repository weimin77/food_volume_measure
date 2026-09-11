#include <gtest/gtest.h>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <vector>
#include "volume_baseline.hpp"
#include "volume_component.hpp"
#include "volume_config.hpp"
#include "volume_grid.hpp"
#include "volume_integrator.hpp"
#include "volume_log.hpp"
#include "volume_measurement.hpp"
#include "volume_pointcloudprocess.hpp"



namespace {

constexpr double kCell = 0.005;
constexpr double kCuboidTop = 0.02;
constexpr double kExpectedVolumeCm3 = 200.0;

vm::PointCloud make_baseline(double half = 0.15) {
    vm::PointCloud cloud;
    for (double x = -half; x <= half + 1.0e-9; x += kCell) {
        for (double y = -half; y <= half + 1.0e-9; y += kCell) {
            cloud.points.push_back({static_cast<float>(x), static_cast<float>(y), 0.0F});
        }
    }
    return cloud;
}

vm::PointCloud make_food(double u_min = 0.0025, double u_max = 0.0975, double v_min = 0.0025, double v_max = 0.0975,
                         double hole_u = -1.0, double hole_v = -1.0) {
    vm::PointCloud cloud = make_baseline();
    for (double x = u_min; x <= u_max + 1.0e-9; x += kCell) {
        for (double y = v_min; y <= v_max + 1.0e-9; y += kCell) {
            if (std::fabs(x - hole_u) < 1.0e-6 && std::fabs(y - hole_v) < 1.0e-6) {
                continue;
            }
            cloud.points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(kCuboidTop)});
        }
    }
    return cloud;
}

vm::PointCloud make_food_with_rect_hole(double hole_x_min, double hole_x_max, double hole_y_min, double hole_y_max) {
    vm::PointCloud cloud = make_baseline();
    for (double x = 0.0025; x <= 0.0975 + 1.0e-9; x += kCell) {
        for (double y = 0.0025; y <= 0.0975 + 1.0e-9; y += kCell) {
            if (x >= hole_x_min - 1.0e-9 && x <= hole_x_max + 1.0e-9 && y >= hole_y_min - 1.0e-9 &&
                y <= hole_y_max + 1.0e-9) {
                continue;
            }
            cloud.points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(kCuboidTop)});
        }
    }
    return cloud;
}

vm::MeasurementConfig make_config() {
    vm::MeasurementConfig cfg;
    cfg.voxel_size_m = 0.002F;
    cfg.plane_distance_threshold_m = 0.003F;
    cfg.integration_resolution_m = 0.005F;
    cfg.roi_border_margin_m = 0.02F;
    cfg.min_height_m = 0.0015F;
    cfg.baseline_fill_radius_cells = 2;
    cfg.cluster_min_points = 8;
    cfg.foreground_cluster_eps_m = 0.010F;
    cfg.cluster_eps_m = 0.02F;
    return cfg;
}

} // namespace



TEST(VolumeTypes, LengthUnitScale) {
    EXPECT_DOUBLE_EQ(vm::length_unit_to_meter_scale(vm::LengthUnit::kMeter), 1.0);
    EXPECT_DOUBLE_EQ(vm::length_unit_to_meter_scale(vm::LengthUnit::kMillimeter), 0.001);
}



TEST(VolumeTypes, FiniteCheck) {
    const vm::Point3f ok{1.0F, 2.0F, 3.0F};
    EXPECT_TRUE(vm::is_finite(ok));
    const vm::Point3f bad{1.0F, std::numeric_limits<float>::quiet_NaN(), 3.0F};
    EXPECT_FALSE(vm::is_finite(bad));
}



TEST(VolumeTypes, StatusToStringAllValues) {
    for (int i = 0; i <= static_cast<int>(vm::MeasurementStatus::kUnsupportedPlatform); ++i) {
        EXPECT_NE(vm::status_to_string(static_cast<vm::MeasurementStatus>(i)), nullptr);
    }
    // Out-of-range enum values fall through to the switch default branches.
    EXPECT_STREQ(vm::status_to_string(static_cast<vm::MeasurementStatus>(999)), "unknown");
    EXPECT_DOUBLE_EQ(vm::length_unit_to_meter_scale(static_cast<vm::LengthUnit>(999)), 1.0);
}



TEST(Logging, FileSinkWrites) {
    const std::string path = "unit_log_test.txt";
    vm::log_set_level(vm::LogLevel::kInfo);
    vm::log_set_console(false);
    vm::log_set_file(path, vm::LogFileMode::kTruncate);
    vm::log_info("unit log test message");
    vm::log_close_file();

    std::ifstream file(path);
    ASSERT_TRUE(file.is_open());
    std::string line;
    std::getline(file, line);
    EXPECT_NE(line.find("unit log test message"), std::string::npos);
    file.close();
    vm::log_set_level(vm::LogLevel::kOff);
    std::remove(path.c_str());
}



TEST(Logging, LevelFiltersMessages) {
    const std::string path = "unit_log_test_filter.txt";
    vm::log_set_level(vm::LogLevel::kWarning);
    vm::log_set_console(false);
    vm::log_set_file(path, vm::LogFileMode::kTruncate);
    vm::log_info("this info message must be dropped");
    vm::log_warning("this warning message must be kept");
    vm::log_close_file();

    std::ifstream file(path);
    ASSERT_TRUE(file.is_open());
    const std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    file.close();
    EXPECT_EQ(content.find("this info message must be dropped"), std::string::npos);
    EXPECT_NE(content.find("this warning message must be kept"), std::string::npos);
    vm::log_set_level(vm::LogLevel::kOff);
    std::remove(path.c_str());
}



TEST(VolumeMeasurement, EmptyBaseline) {
    vm::VolumeMeasurement measurement;
    measurement.set_config(make_config());
    const auto est = measurement.measure({}, make_food());
    EXPECT_EQ(est.status, vm::MeasurementStatus::kEmptyBaseline);
}



TEST(VolumeMeasurement, NonFiniteFoodPointsSkipped) {
    // A non-finite depth return must be dropped, not fatal, as long as valid food remains.
    vm::PointCloud cloud = make_food();
    cloud.points.push_back({0.0F, std::numeric_limits<float>::infinity(), 0.0F});
    vm::VolumeMeasurement measurement;
    measurement.set_config(make_config());
    const auto est = measurement.measure({make_baseline()}, cloud);
    EXPECT_EQ(est.status, vm::MeasurementStatus::kSuccess) << est.message;
}



TEST(VolumeMeasurement, AllNonFiniteFoodInput) {
    vm::PointCloud cloud;
    cloud.points.push_back({0.0F, std::numeric_limits<float>::infinity(), 0.0F});
    cloud.points.push_back({std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F});
    vm::VolumeMeasurement measurement;
    measurement.set_config(make_config());
    const auto est = measurement.measure({make_baseline()}, cloud);
    EXPECT_EQ(est.status, vm::MeasurementStatus::kNonFiniteInput);
}



TEST(VolumeMeasurement, CuboidEndToEnd) {
    const std::vector<vm::PointCloud> baselines{make_baseline()};
    vm::VolumeMeasurement measurement;
    measurement.set_config(make_config());
    const auto est = measurement.measure(baselines, make_food());
    ASSERT_EQ(est.status, vm::MeasurementStatus::kSuccess) << est.message;
    EXPECT_NEAR(est.volume_cm3, kExpectedVolumeCm3, 10.0);
    EXPECT_GT(est.coverage_ratio, 0.8);
    EXPECT_EQ(est.baseline_frames, 1u);
    EXPECT_GT(est.baseline_cell_count, 0u);
}



TEST(VolumeMeasurement, EmptyTrayNoFood) {
    const std::vector<vm::PointCloud> baselines{make_baseline()};
    vm::VolumeMeasurement measurement;
    measurement.set_config(make_config());
    const auto est = measurement.measure(baselines, make_baseline());
    EXPECT_EQ(est.status, vm::MeasurementStatus::kInsufficientCoverage);
}



TEST(VolumeMeasurement, MillimeterInput) {
    auto baseline = make_baseline();
    auto food = make_food();
    for (auto& cloud : std::vector<vm::PointCloud*>{&baseline, &food}) {
        for (auto& p : cloud->points) {
            p.x *= 1000.0F;
            p.y *= 1000.0F;
            p.z *= 1000.0F;
        }
    }
    auto cfg = make_config();
    cfg.input_unit = vm::LengthUnit::kMillimeter;
    vm::VolumeMeasurement measurement;
    measurement.set_config(cfg);
    const auto est = measurement.measure({baseline}, food);
    ASSERT_EQ(est.status, vm::MeasurementStatus::kSuccess) << est.message;
    EXPECT_NEAR(est.volume_cm3, kExpectedVolumeCm3, 10.0);
}



TEST(VolumeMeasurement, TiltedTrayDetected) {
    const double theta = 0.5;
    const double s = std::sin(theta);
    const double c = std::cos(theta);

    const auto mk = [&](double u, double v, double h) -> vm::Point3f {
        return {static_cast<float>(u * c - h * s), static_cast<float>(v), static_cast<float>(u * s + h * c)};
    };

    vm::PointCloud baseline;
    for (double u = -0.15; u <= 0.15 + 1.0e-9; u += kCell) {
        for (double v = -0.15; v <= 0.15 + 1.0e-9; v += kCell) {
            baseline.points.push_back(mk(u, v, 0.0));
        }
    }
    vm::PointCloud food = baseline;
    for (double u = 0.0025; u <= 0.0975 + 1.0e-9; u += kCell) {
        for (double v = 0.0025; v <= 0.0975 + 1.0e-9; v += kCell) {
            food.points.push_back(mk(u, v, kCuboidTop));
        }
    }

    vm::VolumeMeasurement measurement;
    measurement.set_config(make_config());
    const auto est = measurement.measure({baseline}, food);
    ASSERT_EQ(est.status, vm::MeasurementStatus::kSuccess) << est.message;
    EXPECT_NEAR(est.volume_cm3, kExpectedVolumeCm3, 25.0);
}



TEST(VolumeMeasurement, MultiComponentVolume) {
    const std::vector<vm::PointCloud> baselines{make_baseline(0.25)};
    vm::PointCloud food = make_baseline(0.25);
    // Two separated cuboids; the second starts far enough to form its own footprint cluster.
    for (double x = 0.0025; x <= 0.0975 + 1.0e-9; x += kCell) {
        for (double y = 0.0025; y <= 0.0975 + 1.0e-9; y += kCell) {
            food.points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(kCuboidTop)});
        }
    }
    for (double x = 0.1125; x <= 0.2075 + 1.0e-9; x += kCell) {
        for (double y = 0.0025; y <= 0.0975 + 1.0e-9; y += kCell) {
            food.points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(kCuboidTop)});
        }
    }

    vm::VolumeMeasurement measurement;
    measurement.set_config(make_config());
    const auto est = measurement.measure(baselines, food);
    ASSERT_EQ(est.status, vm::MeasurementStatus::kSuccess) << est.message;
    EXPECT_EQ(est.component_count, 2u);
    EXPECT_NEAR(est.volume_cm3, 2.0 * kExpectedVolumeCm3, 20.0);

    // Per-component results are exposed aligned with selected_cluster_labels.
    ASSERT_EQ(est.component_estimates.size(), 2u);
    double per_component_sum = 0.0;
    for (const auto& estimate : est.component_estimates) {
        EXPECT_NEAR(estimate.volume_cm3, kExpectedVolumeCm3, 25.0);
        per_component_sum += estimate.volume_cm3;
    }
    EXPECT_NEAR(per_component_sum, est.volume_cm3, 30.0);
}



TEST(Integrator, PerComponentVolumes) {
    const std::vector<vm::PointCloud> baselines{make_baseline(0.25)};
    vm::PointCloud food = make_baseline(0.25);
    // Two separated cuboids; the second starts far enough to form its own footprint cluster.
    for (double x = 0.0025; x <= 0.0975 + 1.0e-9; x += kCell) {
        for (double y = 0.0025; y <= 0.0975 + 1.0e-9; y += kCell) {
            food.points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(kCuboidTop)});
        }
    }
    for (double x = 0.1125; x <= 0.2075 + 1.0e-9; x += kCell) {
        for (double y = 0.0025; y <= 0.0975 + 1.0e-9; y += kCell) {
            food.points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(kCuboidTop)});
        }
    }

    const vm::MeasurementConfig cfg = make_config();

    vm::PreprocessResult pre{};
    ASSERT_EQ(vm::preprocess_cloud(food, cfg, pre), vm::MeasurementStatus::kSuccess);
    vm::PointCloud remaining{};
    ASSERT_EQ(vm::remove_dominant_plane(pre.cloud, cfg, remaining), vm::MeasurementStatus::kSuccess);

    vm::BaselineModel baseline{};
    ASSERT_EQ(vm::build_baseline_model(baselines, remaining, cfg, baseline), vm::MeasurementStatus::kSuccess);

    vm::FoodComponents components{};
    ASSERT_EQ(vm::extract_food_components(remaining, baseline, cfg, components), vm::MeasurementStatus::kSuccess);
    ASSERT_EQ(components.labels.size(), 2u);

    std::vector<vm::ComponentVolumeEstimate> results;
    ASSERT_EQ(vm::measure_component_volumes(components, baseline, cfg, results), vm::MeasurementStatus::kSuccess);
    ASSERT_EQ(results.size(), 2u);

    double total = 0.0;
    for (const auto& estimate : results) {
        EXPECT_NEAR(estimate.volume_cm3, kExpectedVolumeCm3, 25.0);
        total += estimate.volume_cm3;
    }
    EXPECT_NEAR(total, 2.0 * kExpectedVolumeCm3, 40.0);
}



TEST(VolumeMeasurement, SmallHoleInterpolated) {
    const std::vector<vm::PointCloud> baselines{make_baseline()};
    // One missing interior cell (0.0525, 0.0525) surrounded by measured cells.
    vm::VolumeMeasurement measurement;
    measurement.set_config(make_config());
    const auto est = measurement.measure(baselines, make_food(0.0025, 0.0975, 0.0025, 0.0975, 0.0525, 0.0525));
    ASSERT_EQ(est.status, vm::MeasurementStatus::kSuccess) << est.message;
    EXPECT_NEAR(est.volume_cm3, kExpectedVolumeCm3, 10.0);
    EXPECT_EQ(est.interpolated_cells, 1u);
    EXPECT_NEAR(est.interpolated_volume_cm3, kCuboidTop * kCell * kCell * 1.0e6, 0.05);
}



TEST(VolumeMeasurement, QuadraticHoleCompletion) {
    const std::vector<vm::PointCloud> baselines{make_baseline()};
    // A 4x4-cell interior gap exceeds the small-hole cap (9 cells) and forces the
    // quadratic-surface (curve fill) path: collect_rim_cells + fit_quadratic_hole.
    vm::VolumeMeasurement measurement;
    measurement.set_config(make_config());
    const auto est = measurement.measure(baselines, make_food_with_rect_hole(0.04, 0.06, 0.04, 0.06));
    ASSERT_EQ(est.status, vm::MeasurementStatus::kSuccess) << est.message;
    EXPECT_EQ(est.interpolated_cells, 16u);
    EXPECT_NEAR(est.volume_cm3, kExpectedVolumeCm3, 10.0);
}



TEST(VolumeMeasurement, OversizedHoleLeftUnfilled) {
    const std::vector<vm::PointCloud> baselines{make_baseline()};
    // A 10x10-cell gap (25 cm^2) exceeds the 8 cm^2 curve-fill cap, so it must stay unfilled.
    vm::VolumeMeasurement measurement;
    measurement.set_config(make_config());
    const auto est = measurement.measure(baselines, make_food_with_rect_hole(0.025, 0.075, 0.025, 0.075));
    ASSERT_EQ(est.status, vm::MeasurementStatus::kSuccess) << est.message;
    EXPECT_EQ(est.interpolated_cells, 0u);
    EXPECT_EQ(est.unfilled_hole_cells, 100u);
}



TEST(Preprocess, EmptyInput) {
    vm::PreprocessResult out;
    EXPECT_EQ(vm::preprocess_cloud(vm::PointCloud{}, make_config(), out), vm::MeasurementStatus::kEmptyInput);
}



TEST(Preprocess, NonFiniteInput) {
    vm::PointCloud cloud;
    cloud.points.push_back({0.0F, std::numeric_limits<float>::infinity(), 0.0F});
    vm::PreprocessResult out;
    EXPECT_EQ(vm::preprocess_cloud(cloud, make_config(), out), vm::MeasurementStatus::kNonFiniteInput);
}



TEST(Preprocess, UnitNormalizeAndVoxel) {
    vm::PointCloud in;
    in.points.push_back({0.0F, 0.0F, 0.0F});
    in.points.push_back({1.0F, 0.0F, 0.0F}); // 1 mm -> 0.001 m
    auto cfg = make_config();
    cfg.input_unit = vm::LengthUnit::kMillimeter;
    cfg.voxel_size_m = 0.002F;

    vm::PreprocessResult out;
    ASSERT_EQ(vm::preprocess_cloud(in, cfg, out), vm::MeasurementStatus::kSuccess);
    EXPECT_EQ(out.input_points, 2u);
    EXPECT_EQ(out.retained_points, out.cloud.points.size());
    // Both points are finite, so retained points must be positive.
    EXPECT_GT(out.retained_points, 0u);
}



TEST(Plane, FitHorizontalPlane) {
    vm::PointCloud cloud;
    for (float x = -0.1F; x <= 0.1F; x += 0.01F) {
        for (float y = -0.1F; y <= 0.1F; y += 0.01F) {
            cloud.points.push_back({x, y, 0.0F});
        }
    }
    vm::Plane plane;
    std::vector<std::size_t> inliers;
    ASSERT_EQ(vm::fit_plane_ransac(cloud, 0.001, 100, plane, inliers), vm::MeasurementStatus::kSuccess);
    EXPECT_NEAR(std::fabs(plane.nz), 1.0F, 0.05F);
    EXPECT_NEAR(plane.d, 0.0, 0.001);
    EXPECT_GT(inliers.size(), 0u);
}



TEST(Plane, RemoveDominantPlane) {
    vm::PointCloud cloud;
    for (float x = -0.1F; x <= 0.1F; x += 0.01F) {
        for (float y = -0.1F; y <= 0.1F; y += 0.01F) {
            cloud.points.push_back({x, y, 0.0F});
        }
    }
    cloud.points.push_back({0.0F, 0.0F, 0.02F});

    vm::PointCloud remaining;
    ASSERT_EQ(vm::remove_dominant_plane(cloud, make_config(), remaining), vm::MeasurementStatus::kSuccess);
    ASSERT_FALSE(remaining.points.empty());
    for (const auto& p : remaining.points) {
        EXPECT_NEAR(p.z, 0.02F, 1.0e-4F);
    }
}



TEST(Dbscan, ClustersAndNoise) {
    std::vector<vm::Point3f> points;
    points.push_back({0.0F, 0.0F, 0.0F});
    points.push_back({0.01F, 0.0F, 0.0F});
    points.push_back({0.02F, 0.0F, 0.0F});
    points.push_back({1.0F, 1.0F, 1.0F});
    points.push_back({1.01F, 1.0F, 1.0F});
    points.push_back({1.02F, 1.0F, 1.0F});
    points.push_back({5.0F, 5.0F, 5.0F});

    const std::vector<int> labels = vm::dbscan_labels(points, 0.05, 2);
    ASSERT_EQ(labels.size(), points.size());
    EXPECT_EQ(labels[0], labels[1]);
    EXPECT_EQ(labels[1], labels[2]);
    EXPECT_EQ(labels[3], labels[4]);
    EXPECT_EQ(labels[4], labels[5]);
    EXPECT_NE(labels[0], labels[3]);
    EXPECT_EQ(labels[6], -1);
}



TEST(Baseline, EmptyBaseline) {
    vm::BaselineModel model;
    EXPECT_EQ(vm::build_baseline_model({}, {}, make_config(), model), vm::MeasurementStatus::kEmptyBaseline);
}



TEST(Baseline, BuildBaselineModel) {
    vm::BaselineModel model;
    const std::vector<vm::PointCloud> frames{make_baseline()};
    ASSERT_EQ(vm::build_baseline_model(frames, {}, make_config(), model), vm::MeasurementStatus::kSuccess);
    EXPECT_EQ(model.frame_count, 1u);
    EXPECT_GT(model.cell_count, 0u);
    EXPECT_NE(model.data, nullptr);
}



TEST(Atomic, ComponentExtractionAndVolume) {
    const std::vector<vm::PointCloud> frames{make_baseline()};

    vm::PreprocessResult pre;
    ASSERT_EQ(vm::preprocess_cloud(make_food(), make_config(), pre), vm::MeasurementStatus::kSuccess);

    vm::PointCloud remaining;
    ASSERT_EQ(vm::remove_dominant_plane(pre.cloud, make_config(), remaining), vm::MeasurementStatus::kSuccess);

    vm::BaselineModel baseline;
    ASSERT_EQ(vm::build_baseline_model(frames, remaining, make_config(), baseline), vm::MeasurementStatus::kSuccess);

    vm::FoodComponents components;
    ASSERT_EQ(vm::extract_food_components(remaining, baseline, make_config(), components),
              vm::MeasurementStatus::kSuccess);
    EXPECT_FALSE(components.labels.empty());
    EXPECT_EQ(components.labels.size(), components.clouds.size());

    vm::ComponentVolumeEstimate volume;
    ASSERT_EQ(vm::measure_component_volume(components, baseline, make_config(), volume),
              vm::MeasurementStatus::kSuccess);
    EXPECT_NEAR(volume.volume_cm3, kExpectedVolumeCm3, 10.0);
    EXPECT_GT(volume.measured_cells, 0u);
}



TEST(Operators, ScaleToMetersAndCrop) {
    vm::PointCloud mm;
    mm.points.push_back({0.0F, 0.0F, 0.0F});
    mm.points.push_back({1000.0F, 2000.0F, 3000.0F});
    const vm::PointCloud m = vm::scale_to_meters(mm, vm::LengthUnit::kMillimeter);
    ASSERT_EQ(m.points.size(), 2u);
    EXPECT_NEAR(m.points[1].x, 1.0F, 1.0e-6F);
    EXPECT_NEAR(m.points[1].y, 2.0F, 1.0e-6F);
    EXPECT_NEAR(m.points[1].z, 3.0F, 1.0e-6F);

    // Non-finite points are dropped per point.
    vm::PointCloud mixed;
    mixed.points.push_back({0.0F, 0.0F, 0.0F});
    mixed.points.push_back({std::numeric_limits<float>::quiet_NaN(), 1.0F, 1.0F});
    EXPECT_EQ(vm::scale_to_meters(mixed, vm::LengthUnit::kMeter).points.size(), 1u);

    vm::AxisAlignedRoi roi;
    roi.min_x = -1.0F;
    roi.max_x = 1.0F;
    roi.min_y = -1.0F;
    roi.max_y = 1.0F;
    roi.min_z = -1.0F;
    roi.max_z = 1.0F;
    vm::PointCloud in;
    in.points.push_back({0.0F, 0.0F, 0.0F});
    in.points.push_back({5.0F, 0.0F, 0.0F});
    const vm::PointCloud cropped = vm::crop_axis_aligned(in, roi);
    ASSERT_EQ(cropped.points.size(), 1u);
    EXPECT_NEAR(cropped.points[0].x, 0.0F, 1.0e-6F);
}



TEST(Operators, SplitPlaneInliersAndOrient) {
    vm::PointCloud cloud;
    for (float x = -0.1F; x <= 0.1F; x += 0.02F) {
        for (float y = -0.1F; y <= 0.1F; y += 0.02F) {
            cloud.points.push_back({x, y, 0.0F});
        }
    }
    const std::size_t plane_points = cloud.points.size();
    cloud.points.push_back({0.0F, 0.0F, 0.05F}); // above the plane

    const vm::Plane plane{0.0F, 0.0F, 1.0F, 0.0F};
    vm::PointCloud remaining;
    std::vector<std::size_t> inliers;
    vm::split_plane_inliers(cloud, plane, 0.001, remaining, inliers);
    EXPECT_EQ(inliers.size(), plane_points);
    ASSERT_EQ(remaining.points.size(), 1u);
    EXPECT_NEAR(remaining.points[0].z, 0.05F, 1.0e-6F);

    // orient_plane flips the normal so the median signed height of `points` is non-negative.
    vm::PointCloud above;
    above.points.push_back({0.0F, 0.0F, 0.05F});
    const vm::Plane down{0.0F, 0.0F, -1.0F, 0.0F};
    const vm::Plane flipped = vm::orient_plane(down, above);
    EXPECT_GT(flipped.nz, 0.0F);
    const vm::Plane already{0.0F, 0.0F, 1.0F, 0.0F};
    const vm::Plane kept = vm::orient_plane(already, above);
    EXPECT_GT(kept.nz, 0.0F);
}



TEST(Operators, BuildPlaneFrame) {
    const vm::Plane plane{0.0F, 0.0F, 1.0F, 0.0F};
    const vm::Point3f origin{0.1F, 0.2F, 0.3F};
    const vm::PlaneFrame frame = vm::build_plane_frame(plane, origin);
    // The origin is stored as float32, so compare with float32 precision.
    EXPECT_NEAR(frame.ox, 0.1, 1.0e-6);
    EXPECT_NEAR(frame.oy, 0.2, 1.0e-6);
    EXPECT_NEAR(frame.oz, 0.3, 1.0e-6);
    EXPECT_NEAR(frame.nx, 0.0, 1.0e-9);
    EXPECT_NEAR(frame.ny, 0.0, 1.0e-9);
    EXPECT_NEAR(frame.nz, 1.0, 1.0e-9);
    EXPECT_NEAR(frame.ux, 1.0, 1.0e-9);
    EXPECT_NEAR(frame.uy, 0.0, 1.0e-9);
    EXPECT_NEAR(frame.vx, 0.0, 1.0e-9);
    EXPECT_NEAR(frame.vy, 1.0, 1.0e-9);
}



TEST(Operators, RasterizeBaselineAndRoi) {
    const vm::Plane plane{0.0F, 0.0F, 1.0F, 0.0F};
    const vm::Point3f origin{0.0F, 0.0F, 0.0F};
    const vm::PlaneFrame frame = vm::build_plane_frame(plane, origin);

    vm::BaselineData data;
    const std::vector<vm::PointCloud> frames{make_baseline()};
    ASSERT_EQ(vm::rasterize_baseline(frames, frame, kCell, 0.05, data), vm::MeasurementStatus::kSuccess);
    EXPECT_GT(data.height_by_cell.size(), 0u);
    EXPECT_NEAR(data.cell_size_m, kCell, 1.0e-9);
    for (const auto& entry : data.height_by_cell) {
        EXPECT_NEAR(entry.second, 0.0, 1.0e-9);
    }
    EXPECT_LT(data.bbox_u_min_m, 0.0);
    EXPECT_GT(data.bbox_u_max_m, 0.0);
    EXPECT_LT(data.bbox_v_min_m, 0.0);
    EXPECT_GT(data.bbox_v_max_m, 0.0);

    ASSERT_TRUE(vm::build_plane_roi(data, 0.02));
    EXPECT_NEAR(data.roi.border_margin_m, 0.02, 1.0e-9);
    EXPECT_GT(data.roi.u_min_m, data.bbox_u_min_m);
    EXPECT_LT(data.roi.u_max_m, data.bbox_u_max_m);

    // A margin wider than the footprint leaves no ROI.
    vm::BaselineData small = data;
    EXPECT_FALSE(vm::build_plane_roi(small, 1.0));
}



TEST(Operators, FilterProjectSelect) {
    const std::vector<vm::PointCloud> frames{make_baseline()};
    const vm::MeasurementConfig cfg = make_config();

    vm::PreprocessResult pre;
    ASSERT_EQ(vm::preprocess_cloud(make_food(), cfg, pre), vm::MeasurementStatus::kSuccess);
    vm::PointCloud remaining;
    ASSERT_EQ(vm::remove_dominant_plane(pre.cloud, cfg, remaining), vm::MeasurementStatus::kSuccess);
    vm::BaselineModel baseline;
    ASSERT_EQ(vm::build_baseline_model(frames, remaining, cfg, baseline), vm::MeasurementStatus::kSuccess);

    vm::PointCloud dense;
    ASSERT_EQ(vm::filter_baseline_difference(remaining, baseline, cfg, dense), vm::MeasurementStatus::kSuccess);
    EXPECT_GT(dense.points.size(), 0u);

    const vm::PointCloud projected = vm::project_to_plane(dense, baseline);
    EXPECT_EQ(projected.points.size(), dense.points.size());
    for (const auto& p : projected.points) {
        EXPECT_NEAR(p.z, 0.0F, 1.0e-6F);
    }

    const std::vector<int> labels =
        vm::dbscan_labels(projected.points, cfg.foreground_cluster_eps_m, cfg.cluster_min_points);
    vm::FoodComponents components;
    ASSERT_EQ(vm::select_components(labels, dense, cfg, components), vm::MeasurementStatus::kSuccess);
    ASSERT_EQ(components.labels.size(), 1u);
    EXPECT_EQ(components.labels.size(), components.clouds.size());
}



TEST(Operators, TopSurfaceGridHolesEstimate) {
    const std::vector<vm::PointCloud> frames{make_baseline()};
    const vm::MeasurementConfig cfg = make_config();

    vm::PreprocessResult pre;
    ASSERT_EQ(vm::preprocess_cloud(make_food(), cfg, pre), vm::MeasurementStatus::kSuccess);
    vm::PointCloud remaining;
    ASSERT_EQ(vm::remove_dominant_plane(pre.cloud, cfg, remaining), vm::MeasurementStatus::kSuccess);
    vm::BaselineModel baseline;
    ASSERT_EQ(vm::build_baseline_model(frames, remaining, cfg, baseline), vm::MeasurementStatus::kSuccess);
    vm::FoodComponents components;
    ASSERT_EQ(vm::extract_food_components(remaining, baseline, cfg, components), vm::MeasurementStatus::kSuccess);
    ASSERT_FALSE(components.labels.empty());

    // The monolithic integrator must equal the fine-grained operator chain.
    vm::ComponentVolumeEstimate reference;
    ASSERT_EQ(vm::measure_component_volume(components, baseline, cfg, reference), vm::MeasurementStatus::kSuccess);

    const vm::SurfaceMap surface = vm::build_top_surface(components, baseline);
    EXPECT_FALSE(surface.cells.empty());
    EXPECT_EQ(surface.cells.size(), surface.heights_m.size());
    EXPECT_EQ(surface.cells.size(), surface.labels.size());

    vm::HeightGrid grid;
    ASSERT_EQ(vm::build_height_grid(surface, baseline, cfg, grid), vm::MeasurementStatus::kSuccess);
    EXPECT_GT(grid.occupied_cells, 0u);

    vm::HoleFillStats stats;
    ASSERT_EQ(vm::complete_holes(grid, baseline, cfg, stats), vm::MeasurementStatus::kSuccess);
    EXPECT_GE(stats.filled_cell_count, 0u);

    const vm::ComponentVolumeEstimate estimate = vm::compute_grid_estimate(grid);
    EXPECT_NEAR(estimate.volume_cm3, reference.volume_cm3, 1.0e-6);
    EXPECT_EQ(estimate.measured_cells, reference.measured_cells);
    EXPECT_EQ(estimate.interpolated_cells, reference.interpolated_cells);
}



TEST(Operators, ReferenceVolumes) {
    vm::PointCloud cloud;
    for (double x = 0.0; x <= 0.095 + 1.0e-9; x += kCell) {
        for (double y = 0.0; y <= 0.095 + 1.0e-9; y += kCell) {
            cloud.points.push_back({static_cast<float>(x), static_cast<float>(y), 0.0F});
            cloud.points.push_back({static_cast<float>(x), static_cast<float>(y), 0.02F});
        }
    }
    const double expected = 0.095 * 0.095 * 0.02;
    EXPECT_NEAR(vm::compute_aabb_volume(cloud), expected, 1.0e-9);
    // PCL's MomentOfInertia OBB is computed in single precision, so its axes can
    // be slightly rotated and the box slightly oversized (~1.8% here).
    EXPECT_NEAR(vm::compute_obb_volume(cloud), expected, 1.0e-5);
    EXPECT_NEAR(vm::compute_convex_hull_volume(cloud), expected, 1.0e-6);
    EXPECT_TRUE(std::isnan(vm::compute_aabb_volume(vm::PointCloud{})));
}



TEST(Config, LoadFromJsonFile) {
    const std::string path = "unit_config_test.json";
    {
        std::ofstream file(path);
        file << "{\"voxel_size_m\":0.005,\"cluster_min_points\":12,"
                "\"input_unit\":\"millimeter\",\"selected_labels\":[1,2,3]}";
    }
    vm::MeasurementConfig cfg;
    ASSERT_TRUE(vm::load_config_from_json(path, cfg));
    EXPECT_DOUBLE_EQ(cfg.voxel_size_m, 0.005);
    EXPECT_EQ(cfg.cluster_min_points, 12);
    EXPECT_EQ(cfg.input_unit, vm::LengthUnit::kMillimeter);
    ASSERT_EQ(cfg.selected_labels.size(), 3u);
    EXPECT_EQ(cfg.selected_labels[0], 1);
    EXPECT_EQ(cfg.selected_labels[2], 3);
    std::remove(path.c_str());
}



TEST(Config, MissingFieldsKeepDefaults) {
    const std::string path = "unit_config_partial.json";
    {
        std::ofstream file(path);
        file << "{\"voxel_size_m\":0.007}";
    }
    vm::MeasurementConfig cfg;
    const int default_iterations = cfg.plane_ransac_iterations;
    const double default_eps = cfg.cluster_eps_m;
    ASSERT_TRUE(vm::load_config_from_json(path, cfg));
    EXPECT_DOUBLE_EQ(cfg.voxel_size_m, 0.007);
    EXPECT_EQ(cfg.plane_ransac_iterations, default_iterations);
    EXPECT_DOUBLE_EQ(cfg.cluster_eps_m, default_eps);
    std::remove(path.c_str());
}



TEST(Config, InvalidEnumFails) {
    const std::string path = "unit_config_bad.json";
    {
        std::ofstream file(path);
        file << "{\"input_unit\":\"parsec\"}";
    }
    vm::MeasurementConfig cfg;
    EXPECT_FALSE(vm::load_config_from_json(path, cfg));
    std::remove(path.c_str());
}



TEST(Config, MissingFileFails) {
    vm::MeasurementConfig cfg;
    EXPECT_FALSE(vm::load_config_from_json("does_not_exist_config.json", cfg));
}
