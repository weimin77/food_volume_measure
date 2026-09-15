#include <gtest/gtest.h>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <vector>
#include "volume_log.hpp"
#include "volume_measurement.hpp"
#include "volume_types.hpp"



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

vm::FoodVolumeMeasurer make_measurer() {
    vm::FoodVolumeMeasurer measurer;
    measurer.set_config(make_config());
    return measurer;
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



TEST(Measurer, ChainableSetters) {
    vm::FoodVolumeMeasurer measurer;
    vm::AxisAlignedRoi roi;
    roi.min_x = -0.2F;
    roi.max_x = 0.2F;
    roi.min_y = -0.2F;
    roi.max_y = 0.2F;
    roi.min_z = -0.1F;
    roi.max_z = 0.2F;

    measurer.set_input_unit(vm::LengthUnit::kMillimeter)
        .set_voxel_size(0.002)
        .set_integration_resolution(0.005)
        .set_height_range(0.0015, -1.0)
        .set_roi(roi)
        .clear_roi()
        .set_selection_mode(vm::ComponentSelectionMode::kManual)
        .set_selected_labels({1, 2})
        .set_cluster_params(0.010, 8)
        .set_plane_distance_threshold(0.003)
        .set_save_middle_cloud(true)
        .set_middle_cloud_dir("unit_middle_cloud_test");

    const vm::MeasurementConfig& cfg = measurer.config();
    EXPECT_EQ(cfg.input_unit, vm::LengthUnit::kMillimeter);
    EXPECT_DOUBLE_EQ(cfg.voxel_size_m, 0.002);
    EXPECT_DOUBLE_EQ(cfg.integration_resolution_m, 0.005);
    EXPECT_DOUBLE_EQ(cfg.min_height_m, 0.0015);
    EXPECT_DOUBLE_EQ(cfg.max_height_m, -1.0);
    EXPECT_FALSE(cfg.use_roi);
    EXPECT_EQ(cfg.selection_mode, vm::ComponentSelectionMode::kManual);
    ASSERT_EQ(cfg.selected_labels.size(), 2u);
    EXPECT_EQ(cfg.selected_labels[1], 2);
    EXPECT_DOUBLE_EQ(cfg.foreground_cluster_eps_m, 0.010);
    EXPECT_EQ(cfg.cluster_min_points, 8);
    EXPECT_DOUBLE_EQ(cfg.plane_distance_threshold_m, 0.003);
    EXPECT_TRUE(cfg.save_middle_cloud);
    EXPECT_EQ(cfg.middle_cloud_dir, "unit_middle_cloud_test");
}



TEST(Measurer, EmptyBaseline) {
    auto measurer = make_measurer();
    measurer.set_food(make_food());
    const auto est = measurer.run();
    EXPECT_EQ(est.status, vm::MeasurementStatus::kEmptyBaseline);
}



TEST(Measurer, EmptyFoodInput) {
    auto measurer = make_measurer();
    measurer.set_baseline({make_baseline()});
    const auto est = measurer.run();
    EXPECT_EQ(est.status, vm::MeasurementStatus::kEmptyInput);
}



TEST(Measurer, NonFiniteFoodPointsSkipped) {
    // A non-finite depth return must be dropped, not fatal, as long as valid food remains.
    vm::PointCloud cloud = make_food();
    cloud.points.push_back({0.0F, std::numeric_limits<float>::infinity(), 0.0F});
    auto measurer = make_measurer();
    const auto est = measurer.set_baseline({make_baseline()}).set_food(cloud).run();
    EXPECT_EQ(est.status, vm::MeasurementStatus::kSuccess) << est.message;
}



TEST(Measurer, AllNonFiniteFoodInput) {
    vm::PointCloud cloud;
    cloud.points.push_back({0.0F, std::numeric_limits<float>::infinity(), 0.0F});
    cloud.points.push_back({std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F});
    auto measurer = make_measurer();
    const auto est = measurer.set_baseline({make_baseline()}).set_food(cloud).run();
    EXPECT_EQ(est.status, vm::MeasurementStatus::kNonFiniteInput);
}



TEST(Measurer, CuboidEndToEnd) {
    auto measurer = make_measurer();
    const auto est = measurer.set_baseline({make_baseline()}).set_food(make_food()).run();
    ASSERT_EQ(est.status, vm::MeasurementStatus::kSuccess) << est.message;
    EXPECT_NEAR(est.volume_cm3, kExpectedVolumeCm3, 10.0);
    EXPECT_GT(est.coverage_ratio, 0.8);
    EXPECT_EQ(est.baseline_frames, 1u);
    EXPECT_GT(est.baseline_cell_count, 0u);
}



TEST(Measurer, AddBaselineFrameAccumulates) {
    auto measurer = make_measurer();
    measurer.add_baseline_frame(make_baseline());
    measurer.add_baseline_frame(make_baseline());
    measurer.set_food(make_food());
    const auto est = measurer.run();
    ASSERT_EQ(est.status, vm::MeasurementStatus::kSuccess) << est.message;
    EXPECT_EQ(est.baseline_frames, 2u);
    EXPECT_NEAR(est.volume_cm3, kExpectedVolumeCm3, 10.0);
}



TEST(Measurer, EmptyTrayNoFood) {
    auto measurer = make_measurer();
    const auto est = measurer.set_baseline({make_baseline()}).set_food(make_baseline()).run();
    EXPECT_EQ(est.status, vm::MeasurementStatus::kInsufficientCoverage);
}



TEST(Measurer, MillimeterInput) {
    auto baseline = make_baseline();
    auto food = make_food();
    for (auto* cloud : std::vector<vm::PointCloud*>{&baseline, &food}) {
        for (auto& p : cloud->points) {
            p.x *= 1000.0F;
            p.y *= 1000.0F;
            p.z *= 1000.0F;
        }
    }
    auto measurer = make_measurer();
    const auto est = measurer.set_input_unit(vm::LengthUnit::kMillimeter).set_baseline({baseline}).set_food(food).run();
    ASSERT_EQ(est.status, vm::MeasurementStatus::kSuccess) << est.message;
    EXPECT_NEAR(est.volume_cm3, kExpectedVolumeCm3, 10.0);
}



TEST(Measurer, TiltedTrayDetected) {
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

    auto measurer = make_measurer();
    const auto est = measurer.set_baseline({baseline}).set_food(food).run();
    ASSERT_EQ(est.status, vm::MeasurementStatus::kSuccess) << est.message;
    EXPECT_NEAR(est.volume_cm3, kExpectedVolumeCm3, 25.0);
}



TEST(Measurer, RoiCropping) {
    // Food outside the axis-aligned ROI must not contribute to the measured volume.
    vm::AxisAlignedRoi roi;
    roi.min_x = -0.15F;
    roi.max_x = 0.06F; // only the left half of the tray is measured
    roi.min_y = -0.15F;
    roi.max_y = 0.15F;
    roi.min_z = -0.05F;
    roi.max_z = 0.05F;

    auto measurer = make_measurer();
    const auto est = measurer.set_roi(roi).set_baseline({make_baseline()}).set_food(make_food()).run();
    ASSERT_EQ(est.status, vm::MeasurementStatus::kSuccess) << est.message;
    // The cuboid spans u = [0.0025, 0.0975]; the ROI keeps roughly half of it.
    EXPECT_NEAR(est.volume_cm3, 0.5 * kExpectedVolumeCm3, 35.0);
}



TEST(Measurer, HeightCapApplied) {
    // A height cap clamps every cell to max_height_m, bounding the integrated volume.
    auto measurer = make_measurer();
    const auto est =
        measurer.set_height_range(0.0015, 0.01).set_baseline({make_baseline()}).set_food(make_food()).run();
    ASSERT_EQ(est.status, vm::MeasurementStatus::kSuccess) << est.message;
    EXPECT_NEAR(est.max_height_m, 0.01, 1.0e-9);
    // 0.1 m x 0.1 m footprint x 0.01 m cap = 100 cm^3.
    EXPECT_NEAR(est.volume_cm3, 100.0, 15.0);
}



TEST(Measurer, SmallClusterRejected) {
    // A tiny separated cluster below the minimum candidate size must not be reported.
    vm::PointCloud food = make_baseline(0.25);
    for (double x = 0.0025; x <= 0.0975 + 1.0e-9; x += kCell) {
        for (double y = 0.0025; y <= 0.0975 + 1.0e-9; y += kCell) {
            food.points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(kCuboidTop)});
        }
    }
    // A 3x3 speck (~9 points) far from the main cuboid.
    for (double x = -0.15; x <= -0.14 + 1.0e-9; x += kCell) {
        for (double y = -0.15; y <= -0.14 + 1.0e-9; y += kCell) {
            food.points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(kCuboidTop)});
        }
    }

    auto measurer = make_measurer();
    const auto est = measurer.set_baseline({make_baseline(0.25)}).set_food(food).run();
    ASSERT_EQ(est.status, vm::MeasurementStatus::kSuccess) << est.message;
    EXPECT_EQ(est.component_count, 1u);
    EXPECT_NEAR(est.volume_cm3, kExpectedVolumeCm3, 15.0);
}



TEST(Measurer, MultiComponentVolume) {
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

    auto measurer = make_measurer();
    const auto est = measurer.set_baseline({make_baseline(0.25)}).set_food(food).run();
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



TEST(Measurer, ManualSelectionMode) {
    vm::PointCloud food = make_baseline(0.25);
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

    // Manual mode with a label that does not exist finds no food.
    auto measurer = make_measurer();
    const auto missing = measurer.set_selection_mode(vm::ComponentSelectionMode::kManual)
                             .set_selected_labels({42})
                             .set_baseline({make_baseline(0.25)})
                             .set_food(food)
                             .run();
    EXPECT_EQ(missing.status, vm::MeasurementStatus::kFoodNotFound);

    // Manual mode with the first real label measures only that component.
    auto manual = make_measurer();
    manual.set_selection_mode(vm::ComponentSelectionMode::kManual).set_selected_labels({0});
    const auto est = manual.set_baseline({make_baseline(0.25)}).set_food(food).run();
    ASSERT_EQ(est.status, vm::MeasurementStatus::kSuccess) << est.message;
    EXPECT_EQ(est.component_count, 1u);
    EXPECT_NEAR(est.volume_cm3, kExpectedVolumeCm3, 25.0);
}



TEST(Measurer, SmallHoleInterpolated) {
    auto measurer = make_measurer();
    const auto est = measurer.set_baseline({make_baseline()})
                         .set_food(make_food(0.0025, 0.0975, 0.0025, 0.0975, 0.0525, 0.0525))
                         .run();
    ASSERT_EQ(est.status, vm::MeasurementStatus::kSuccess) << est.message;
    EXPECT_NEAR(est.volume_cm3, kExpectedVolumeCm3, 10.0);
    EXPECT_EQ(est.interpolated_cells, 1u);
    EXPECT_NEAR(est.interpolated_volume_cm3, kCuboidTop * kCell * kCell * 1.0e6, 0.05);
}



TEST(Measurer, QuadraticHoleCompletion) {
    auto measurer = make_measurer();
    // A 4x4-cell interior gap exceeds the small-hole cap (9 cells) and forces the
    // quadratic-surface (curve fill) path.
    const auto est =
        measurer.set_baseline({make_baseline()}).set_food(make_food_with_rect_hole(0.04, 0.06, 0.04, 0.06)).run();
    ASSERT_EQ(est.status, vm::MeasurementStatus::kSuccess) << est.message;
    EXPECT_EQ(est.interpolated_cells, 16u);
    EXPECT_NEAR(est.volume_cm3, kExpectedVolumeCm3, 10.0);
}



TEST(Measurer, OversizedHoleLeftUnfilled) {
    auto measurer = make_measurer();
    // A 10x10-cell gap (25 cm^2) exceeds the 8 cm^2 curve-fill cap, so it must stay unfilled.
    const auto est =
        measurer.set_baseline({make_baseline()}).set_food(make_food_with_rect_hole(0.025, 0.075, 0.025, 0.075)).run();
    ASSERT_EQ(est.status, vm::MeasurementStatus::kSuccess) << est.message;
    EXPECT_EQ(est.interpolated_cells, 0u);
    EXPECT_EQ(est.unfilled_hole_cells, 100u);
}



TEST(Measurer, ReferenceVolumesReported) {
    // Reference volumes need a cloud with actual thickness that survives the
    // pipeline: a z=0 bottom layer would be removed with the background plane,
    // so the lower face sits at 0.005 m (above the RANSAC threshold).
    vm::PointCloud food = make_baseline();
    for (double x = 0.0025; x <= 0.0975 + 1.0e-9; x += kCell) {
        for (double y = 0.0025; y <= 0.0975 + 1.0e-9; y += kCell) {
            food.points.push_back({static_cast<float>(x), static_cast<float>(y), 0.005F});
            food.points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(kCuboidTop)});
        }
    }

    auto measurer = make_measurer();
    const auto est = measurer.set_baseline({make_baseline()}).set_food(food).run();
    ASSERT_EQ(est.status, vm::MeasurementStatus::kSuccess) << est.message;
    // Reference volumes from the selected components are finite and positive.
    EXPECT_TRUE(std::isfinite(est.aabb_volume_m3));
    EXPECT_TRUE(std::isfinite(est.obb_volume_m3));
    EXPECT_TRUE(std::isfinite(est.convex_hull_volume_m3));
    EXPECT_GT(est.aabb_volume_m3, 0.0);
    EXPECT_GT(est.obb_volume_m3, 0.0);
    EXPECT_GT(est.convex_hull_volume_m3, 0.0);
    // The AABB encloses the slab: 0.095 x 0.095 x (0.02 - 0.005) m.
    EXPECT_NEAR(est.aabb_volume_m3, 0.095 * 0.095 * 0.015, 1.0e-4);
}



TEST(Measurer, LoadConfigFromJsonFile) {
    const std::string path = "unit_config_test.json";
    {
        std::ofstream file(path);
        file << "{\"voxel_size_m\":0.005,\"cluster_min_points\":12,"
                "\"input_unit\":\"millimeter\",\"selected_labels\":[1,2,3]}";
    }
    vm::FoodVolumeMeasurer measurer;
    ASSERT_TRUE(measurer.load_config_from_json(path));
    const vm::MeasurementConfig& cfg = measurer.config();
    EXPECT_DOUBLE_EQ(cfg.voxel_size_m, 0.005);
    EXPECT_EQ(cfg.cluster_min_points, 12);
    EXPECT_EQ(cfg.input_unit, vm::LengthUnit::kMillimeter);
    ASSERT_EQ(cfg.selected_labels.size(), 3u);
    EXPECT_EQ(cfg.selected_labels[0], 1);
    EXPECT_EQ(cfg.selected_labels[2], 3);
    std::remove(path.c_str());
}



TEST(Measurer, MissingConfigFieldsKeepDefaults) {
    const std::string path = "unit_config_partial.json";
    {
        std::ofstream file(path);
        file << "{\"voxel_size_m\":0.007}";
    }
    vm::FoodVolumeMeasurer measurer;
    const int default_iterations = measurer.config().plane_ransac_iterations;
    const double default_eps = measurer.config().cluster_eps_m;
    ASSERT_TRUE(measurer.load_config_from_json(path));
    EXPECT_DOUBLE_EQ(measurer.config().voxel_size_m, 0.007);
    EXPECT_EQ(measurer.config().plane_ransac_iterations, default_iterations);
    EXPECT_DOUBLE_EQ(measurer.config().cluster_eps_m, default_eps);
    std::remove(path.c_str());
}



TEST(Measurer, InvalidConfigEnumFails) {
    const std::string path = "unit_config_bad.json";
    {
        std::ofstream file(path);
        file << "{\"input_unit\":\"parsec\"}";
    }
    vm::FoodVolumeMeasurer measurer;
    EXPECT_FALSE(measurer.load_config_from_json(path));
    std::remove(path.c_str());
}



TEST(Measurer, MissingConfigFileFails) {
    vm::FoodVolumeMeasurer measurer;
    EXPECT_FALSE(measurer.load_config_from_json("does_not_exist_config.json"));
}



TEST(Measurer, SaveConfigRoundTrip) {
    const std::string path = "unit_config_roundtrip.json";
    auto measurer = make_measurer();
    measurer.set_voxel_size(0.0042).set_plane_distance_threshold(0.0025);
    ASSERT_TRUE(measurer.save_config_to_json(path));

    vm::FoodVolumeMeasurer restored;
    ASSERT_TRUE(restored.load_config_from_json(path));
    EXPECT_DOUBLE_EQ(restored.config().voxel_size_m, 0.0042);
    EXPECT_DOUBLE_EQ(restored.config().plane_distance_threshold_m, 0.0025);
    EXPECT_DOUBLE_EQ(restored.config().integration_resolution_m, measurer.config().integration_resolution_m);
    std::remove(path.c_str());
}

TEST(Measurer, SaveMiddleCloudWritesStageFiles) {
    // Enabling the dump must leave one PCD per pipeline stage on disk.
    const std::string dir = "unit_middle_cloud_test";
    std::filesystem::remove_all(dir);

    auto measurer = make_measurer();
    const auto est = measurer.set_save_middle_cloud(true)
                         .set_middle_cloud_dir(dir)
                         .set_baseline({make_baseline()})
                         .set_food(make_food())
                         .run();
    ASSERT_EQ(est.status, vm::MeasurementStatus::kSuccess) << est.message;

    const char* const kExpected[] = {"0_input_food.pcd",       "1_downsampled.pcd",     "2_remaining.pcd",
                                     "3_baseline_surface.pcd", "4_food_components.pcd", "5_top_surface.pcd"};
    for (const char* name : kExpected) {
        std::ifstream file(dir + "/" + name);
        EXPECT_TRUE(file.is_open()) << "missing stage cloud: " << name;
    }

    std::filesystem::remove_all(dir);
}



TEST(Measurer, SaveMiddleCloudDisabledWritesNothing) {
    const std::string dir = "unit_middle_cloud_off_test";
    std::filesystem::remove_all(dir);

    auto measurer = make_measurer();
    const auto est = measurer.set_middle_cloud_dir(dir).set_baseline({make_baseline()}).set_food(make_food()).run();
    ASSERT_EQ(est.status, vm::MeasurementStatus::kSuccess) << est.message;
    EXPECT_FALSE(std::filesystem::exists(dir));
}
