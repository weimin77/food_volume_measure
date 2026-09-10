// Conan::ImportStart
#include <cstddef>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>
#include "volume_pipeline.hpp"
#include "volume_baseline.hpp"
#include "volume_component.hpp"
#include "volume_integrator.hpp"
#include "volume_log.hpp"
#include "volume_pointcloudprocess.hpp"
// Conan::ImportEnd

#include "volume_profiler.hpp"

#ifndef __ARM_EABI__
#include <algorithm>
#include <cstdint>
#include <Eigen/Dense>
#include <pcl/PolygonMesh.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
#endif



namespace vm {



namespace {

VolumeEstimate failure(MeasurementStatus status, std::string message) {
    log_error("pipeline failed: " + message);
    VolumeEstimate est{};
    est.status = status;
    est.message = std::move(message);
    return est;
}

// Chooses orientation points from the largest 3D DBSCAN component, or empty when none qualifies.
PointCloud select_largest_cluster(const PointCloud& cloud_m, const MeasurementConfig& cfg) {
    VM_PROFILE_FUNC();
#ifdef __ARM_EABI__
    (void)cloud_m;
    (void)cfg;
    return PointCloud{};
#else
    PointCloud out{};
    if (cloud_m.points.empty() || cfg.cluster_eps_m <= 0.0F || cfg.cluster_min_points <= 0) {
        return out;
    }

    const std::vector<int> labels = dbscan_labels(cloud_m.points, cfg.cluster_eps_m, cfg.cluster_min_points);

    std::map<int, std::size_t> counts;
    for (const int label : labels) {
        if (label >= 0) {
            counts[label] += 1;
        }
    }
    if (counts.empty()) {
        return out;
    }

    int best_label = -1;
    std::size_t best_count = 0;
    for (const auto& entry : counts) {
        if (entry.second > best_count) {
            best_label = entry.first;
            best_count = entry.second;
        }
    }

    out.points.reserve(best_count);
    for (std::size_t i = 0; i < labels.size(); ++i) {
        if (labels[i] == best_label) {
            out.points.push_back(cloud_m.points[i]);
        }
    }
    return out;
#endif // __ARM_EABI__
}

#ifndef __ARM_EABI__

struct ReferenceVolumes {
    double aabb_volume_m3 = std::numeric_limits<double>::quiet_NaN();
    double obb_volume_m3 = std::numeric_limits<double>::quiet_NaN();
    double convex_hull_volume_m3 = std::numeric_limits<double>::quiet_NaN();
};

// Reference volumes from the selected food points, mirroring the Python POC's AABB / OBB / convex-hull
// comparison block. The OBB here is PCL's moment-based OBB rather than the Python minimum-volume OBB, so
// its value is indicative rather than identical.
ReferenceVolumes compute_reference_volumes(const std::vector<PointCloud>& component_clouds) {
    VM_PROFILE_FUNC();
    ReferenceVolumes out{};

    std::size_t total = 0;
    for (const auto& cloud : component_clouds) {
        total += cloud.points.size();
    }
    if (total == 0) {
        return out;
    }

    PointCloud merged;
    merged.points.reserve(total);
    for (const auto& cloud : component_clouds) {
        merged.points.insert(merged.points.end(), cloud.points.begin(), cloud.points.end());
    }

    out.aabb_volume_m3 = compute_aabb_volume(merged);
    out.obb_volume_m3 = compute_obb_volume(merged);
    out.convex_hull_volume_m3 = compute_convex_hull_volume(merged);
    return out;
}

#endif // __ARM_EABI__

} // namespace



double compute_aabb_volume(const PointCloud& cloud) {
    VM_PROFILE_FUNC();
#ifdef __ARM_EABI__
    (void)cloud;
    return std::numeric_limits<double>::quiet_NaN();
#else
    if (cloud.points.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    pc->points.reserve(cloud.points.size());
    for (const auto& p : cloud.points) {
        pc->points.push_back(pcl::PointXYZ{p.x, p.y, p.z});
    }
    pc->width = static_cast<std::uint32_t>(pc->points.size());
    pc->height = 1;

    pcl::PointXYZ min_pt;
    pcl::PointXYZ max_pt;
    pcl::getMinMax3D(*pc, min_pt, max_pt);
    return static_cast<double>(max_pt.x - min_pt.x) * static_cast<double>(max_pt.y - min_pt.y) *
           static_cast<double>(max_pt.z - min_pt.z);
#endif // __ARM_EABI__
}



double compute_obb_volume(const PointCloud& cloud) {
    VM_PROFILE_FUNC();
#ifdef __ARM_EABI__
    (void)cloud;
    return std::numeric_limits<double>::quiet_NaN();
#else
    if (cloud.points.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    pc->points.reserve(cloud.points.size());
    for (const auto& p : cloud.points) {
        pc->points.push_back(pcl::PointXYZ{p.x, p.y, p.z});
    }
    pc->width = static_cast<std::uint32_t>(pc->points.size());
    pc->height = 1;

    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> moi;
    moi.setInputCloud(pc);
    moi.compute();
    pcl::PointXYZ obb_min;
    pcl::PointXYZ obb_max;
    pcl::PointXYZ obb_position;
    Eigen::Matrix3f obb_rotation = Eigen::Matrix3f::Identity();
    moi.getOBB(obb_min, obb_max, obb_position, obb_rotation);
    return static_cast<double>(obb_max.x - obb_min.x) * static_cast<double>(obb_max.y - obb_min.y) *
           static_cast<double>(obb_max.z - obb_min.z);
#endif // __ARM_EABI__
}



double compute_convex_hull_volume(const PointCloud& cloud) {
    VM_PROFILE_FUNC();
#ifdef __ARM_EABI__
    (void)cloud;
    return std::numeric_limits<double>::quiet_NaN();
#else
    if (cloud.points.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    pc->points.reserve(cloud.points.size());
    for (const auto& p : cloud.points) {
        pc->points.push_back(pcl::PointXYZ{p.x, p.y, p.z});
    }
    pc->width = static_cast<std::uint32_t>(pc->points.size());
    pc->height = 1;

    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud(pc);
    pcl::PolygonMesh mesh;
    hull.reconstruct(mesh);
    pcl::PointCloud<pcl::PointXYZ> hull_vertices;
    pcl::fromPCLPointCloud2(mesh.cloud, hull_vertices);
    if (hull_vertices.points.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (const auto& v : hull_vertices.points) {
        center += Eigen::Vector3d(static_cast<double>(v.x), static_cast<double>(v.y), static_cast<double>(v.z));
    }
    center /= static_cast<double>(hull_vertices.points.size());

    double volume = 0.0;
    for (const auto& poly : mesh.polygons) {
        if (poly.vertices.size() != 3) {
            continue;
        }
        const auto& a = hull_vertices.points[poly.vertices[0]];
        const auto& b = hull_vertices.points[poly.vertices[1]];
        const auto& c = hull_vertices.points[poly.vertices[2]];
        const Eigen::Vector3d va(static_cast<double>(a.x) - center.x(), static_cast<double>(a.y) - center.y(),
                                 static_cast<double>(a.z) - center.z());
        const Eigen::Vector3d vb(static_cast<double>(b.x) - center.x(), static_cast<double>(b.y) - center.y(),
                                 static_cast<double>(b.z) - center.z());
        const Eigen::Vector3d vc(static_cast<double>(c.x) - center.x(), static_cast<double>(c.y) - center.y(),
                                 static_cast<double>(c.z) - center.z());
        // For a convex hull and an interior reference point, the tetrahedra formed by each boundary
        // triangle partition the hull exactly, so their absolute volumes sum to the hull volume.
        volume += std::fabs(va.dot(vb.cross(vc))) / 6.0;
    }
    return volume;
#endif // __ARM_EABI__
}



/**
 * @brief [en] Measures food volume in cubic centimeters from empty-oven baseline frames and a food frame.
 * @brief [zh] 从空炉基线帧与食材帧测量食材体积（立方厘米）。
 * @attacher
 */
VolumeEstimate VolumePipeline::measure(const std::vector<PointCloud>& baseline_frames, const PointCloud& food_frame,
                                       const MeasurementConfig& cfg) const {
    VM_PROFILE_FUNC();
#ifdef __ARM_EABI__
    (void)baseline_frames;
    (void)food_frame;
    (void)cfg;
    return failure(MeasurementStatus::kUnsupportedPlatform, "PCL volume pipeline is unavailable on bare-metal");
#else
    // 1. Preprocess the food frame (unit-normalize, optional crop, voxel downsample).
    PreprocessResult pre{};
    MeasurementStatus st = preprocess_cloud(food_frame, cfg, pre);
    if (st != MeasurementStatus::kSuccess) {
        return failure(st, status_to_string(st));
    }

    // 2. Remove the dominant background plane(s) from the food frame.
    PointCloud remaining{};
    st = remove_dominant_plane(pre.cloud, cfg, remaining);
    if (st != MeasurementStatus::kSuccess) {
        return failure(st, status_to_string(st));
    }

    // 3. Orientation points: largest DBSCAN component, or all remaining when none qualifies.
    PointCloud orientation = select_largest_cluster(remaining, cfg);
    if (orientation.points.empty()) {
        orientation = remaining;
    }

    // 4. Build the reusable empty-oven baseline model.
    BaselineModel baseline{};
    st = build_baseline_model(baseline_frames, orientation, cfg, baseline);
    if (st != MeasurementStatus::kSuccess) {
        return failure(st, status_to_string(st));
    }

    // 5. Extract selected food components.
    FoodComponents components{};
    st = extract_food_components(remaining, baseline, cfg, components);
    if (st != MeasurementStatus::kSuccess) {
        return failure(st, status_to_string(st));
    }

    // 6. Measure the component volume.
    ComponentVolumeEstimate component_volume{};
    st = measure_component_volume(components, baseline, cfg, component_volume);
    if (st != MeasurementStatus::kSuccess) {
        return failure(st, status_to_string(st));
    }

    // 7. Translate the component result and pipeline diagnostics into the public estimate.
    VolumeEstimate est{};
    est.status = MeasurementStatus::kSuccess;
    est.volume_cm3 = component_volume.volume_cm3;
    est.raw_volume_cm3 = component_volume.raw_volume_cm3;
    est.interpolated_volume_cm3 = component_volume.interpolated_volume_cm3;
    est.uncertainty_cm3 = std::numeric_limits<double>::quiet_NaN();
    est.input_points = pre.input_points;
    est.downsampled_points = pre.retained_points;
    est.cluster_count = components.cluster_count;
    est.selected_cluster_labels = components.labels;
    est.selected_cluster_points = 0;
    for (const auto& cloud : components.clouds) {
        est.selected_cluster_points += cloud.points.size();
    }
    est.baseline_frames = baseline.frame_count;
    est.baseline_cell_count = baseline.cell_count;
    est.component_count = components.labels.size();

    // Per-component breakdown (best-effort): the merged total above stays authoritative.
    std::vector<ComponentVolumeEstimate> per_component;
    if (measure_component_volumes(components, baseline, cfg, per_component) == MeasurementStatus::kSuccess) {
        est.component_estimates = std::move(per_component);
    }

    est.top_surface_points = component_volume.top_surface_points;
    est.measured_cells = component_volume.measured_cells;
    est.interpolated_cells = component_volume.interpolated_cells;
    est.occupied_cells = component_volume.occupied_cells;
    est.bbox_cell_count = component_volume.bbox_cell_count;
    est.missing_baseline_cells = component_volume.missing_baseline_cells;
    est.unfilled_hole_cells = component_volume.unfilled_hole_cells;
    est.footprint_area_m2 = component_volume.footprint_area_m2;
    est.coverage_ratio = component_volume.coverage_ratio;
    est.mean_height_m = component_volume.mean_height_m;
    est.max_height_m = component_volume.max_height_m;
    const ReferenceVolumes reference = compute_reference_volumes(components.clouds);
    est.aabb_volume_m3 = reference.aabb_volume_m3;
    est.obb_volume_m3 = reference.obb_volume_m3;
    est.convex_hull_volume_m3 = reference.convex_hull_volume_m3;
    est.message = status_to_string(MeasurementStatus::kSuccess);
    log_info("result: status=" + std::string(status_to_string(est.status)) +
             " volume_cm3=" + std::to_string(est.volume_cm3));
    return est;
#endif // __ARM_EABI__
}



} // namespace vm
