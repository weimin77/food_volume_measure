// demo.cxx — food volume measurement example.
#include "volume_pipeline.hpp"

#include <iostream>
#include <vector>

int main() {
    // Obtain the empty-oven baseline and the food point cloud from a depth camera.
    const vm::PointCloud baseline = /* ... 空炉点云 ... */;
    const vm::PointCloud food = /* ... 食材点云 ... */;
    if (baseline.points.empty() || food.points.empty()) {
        std::cerr << "failed to obtain point-cloud inputs" << std::endl;
        return 1;
    }

    // Run the end-to-end IM measurement.
    vm::VolumePipeline pipeline;
    const vm::VolumeEstimate est =
        pipeline.measure(std::vector<vm::PointCloud>{baseline}, food, vm::MeasurementConfig{});

    if (est.status != vm::MeasurementStatus::kSuccess) {
        std::cerr << "measurement failed: " << vm::status_to_string(est.status) << std::endl;
        return 1;
    }

    std::cout << "volume_cm3: " << est.volume_cm3 << std::endl;
    return 0;
}
