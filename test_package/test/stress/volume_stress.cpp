#include <gtest/gtest.h>
#include <cmath>
#include <cstddef>
#include <vector>
#include "volume_measurement.hpp"
#include "volume_pointcloudprocess.hpp"
#include "volume_measurement.hpp"



namespace {

constexpr double kStep = 0.003;
constexpr double kFoodTopM = 0.02;

// Large empty-oven tray in the z=0 plane, centred on the origin.
vm::PointCloud make_tray(double half) {
    vm::PointCloud cloud;
    for (double x = -half; x <= half + 1.0e-9; x += kStep) {
        for (double y = -half; y <= half + 1.0e-9; y += kStep) {
            cloud.points.push_back({static_cast<float>(x), static_cast<float>(y), 0.0F});
        }
    }
    return cloud;
}

// Tray plus a broad food top surface, exercising the pipeline at scale.
vm::PointCloud make_food(double half) {
    vm::PointCloud cloud = make_tray(half);
    const double hi = half - kStep;
    for (double x = kStep; x <= hi + 1.0e-9; x += kStep) {
        for (double y = kStep; y <= hi + 1.0e-9; y += kStep) {
            cloud.points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(kFoodTopM)});
        }
    }
    return cloud;
}

} // namespace



TEST(Stress, VoxelDownsampleLargeCloud) {
    const vm::PointCloud tray = make_tray(0.3);
    ASSERT_GT(tray.points.size(), 10000u);
    const vm::PointCloud down = vm::voxel_downsample(tray, 0.006);
    EXPECT_LT(down.points.size(), tray.points.size());
    EXPECT_GT(down.points.size(), 0u);
    for (const auto& p : down.points) {
        EXPECT_TRUE(vm::is_finite(p));
    }
}



TEST(Stress, DbscanLargeCloud) {
    constexpr int kCount = 20000;
    std::vector<vm::Point3f> points;
    points.reserve(kCount);
    for (int i = 0; i < kCount; ++i) {
        points.push_back({static_cast<float>(i) * 0.001F, 0.0F, 0.0F});
    }
    const std::vector<int> labels = vm::dbscan_labels(points, 0.01, 4);
    ASSERT_EQ(labels.size(), points.size());
    EXPECT_GE(labels.front(), 0);
    EXPECT_EQ(labels.front(), labels.back());
}



TEST(Stress, PipelineLargeScene) {
    const double half = 0.3;
    const std::vector<vm::PointCloud> baselines{make_tray(half)};
    vm::VolumeMeasurement measurement;
    measurement.set_config(vm::MeasurementConfig{});
    const auto est = measurement.measure(baselines, make_food(half));
    EXPECT_EQ(est.status, vm::MeasurementStatus::kSuccess);
    EXPECT_GT(est.component_count, 0u);
    EXPECT_GT(est.volume_cm3, 0.0);
    EXPECT_GT(est.coverage_ratio, 0.5);
}
