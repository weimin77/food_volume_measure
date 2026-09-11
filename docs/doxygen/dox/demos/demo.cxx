// demo.cxx — PCD-IM food volume measurement example.
#include "volume_measurement.hpp"
#include "volume_pointcloudprocess.hpp"

#include <iostream>
#include <vector>

int main() {
    // Load the empty-oven baseline and the food point cloud from PCD files.
    const vm::PointCloud baseline = vm::load_pcd("empty_oven.pcd");
    const vm::PointCloud food = vm::load_pcd("food.pcd");
    if (baseline.points.empty() || food.points.empty()) {
        std::cerr << "failed to load PCD inputs" << std::endl;
        return 1;
    }

    // Run the end-to-end PCD-IM measurement.
    vm::VolumeMeasurement measurement;
    const vm::VolumeEstimate est =
        measurement.measure(std::vector<vm::PointCloud>{baseline}, food);

    if (est.status != vm::MeasurementStatus::kSuccess) {
        std::cerr << "measurement failed: " << vm::status_to_string(est.status) << std::endl;
        return 1;
    }

    std::cout << "volume_cm3: " << est.volume_cm3 << std::endl;
    return 0;
}
