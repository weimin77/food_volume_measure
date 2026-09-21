// demo.cxx — IM food volume measurement example.
#include "measurement.hpp"

#include <iostream>
#include <vector>

int main() {
    // Load the empty-oven baseline and the food point cloud from PCD files.
    const PointCloud baseline = load_pcd("empty_oven.pcd");
    const PointCloud food = load_pcd("food.pcd");
    if (baseline.points.empty() || food.points.empty()) {
        std::cerr << "failed to load PCD inputs" << std::endl;
        return 1;
    }

    // Run the end-to-end IM measurement.
    const VolumeEstimate est =
        FoodVolumeMeasurer().set_baseline({baseline}).set_food(food).run();

    if (est.status != MeasurementStatus::kSuccess) {
        std::cerr << "measurement failed: " << status_to_string(est.status) << std::endl;
        return 1;
    }

    std::cout << "volume_cm3: " << est.volume_cm3 << std::endl;
    return 0;
}
