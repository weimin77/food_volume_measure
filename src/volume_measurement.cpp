// Conan::ImportStart
#include <cstddef>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>
#include "volume_measurement.hpp"
#include "volume_baseline.hpp"
#include "volume_component.hpp"
#include "volume_config.hpp"
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

VolumeEstimate failure(MeasurementStatus status, const std::string& message) {
    log_error("pipeline failed: " + message);
    VolumeEstimate est{};
    est.status = status;
    est.message = message;
    return est;
}

// Concatenates the selected component clouds once for the reference-volume helpers.
PointCloud merged_components_cloud(const FoodComponents& components) {
    PointCloud merged;
    std::size_t total = 0;
    for (const auto& cloud : components.clouds) {
        total += cloud.points.size();
    }
    merged.points.reserve(total);
    for (const auto& cloud : components.clouds) {
        merged.points.insert(merged.points.end(), cloud.points.begin(), cloud.points.end());
    }
    return merged;
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

} // namespace



/**
 * @brief [en] Loads a PCD point-cloud file into the library's point model.
 * @brief [zh] 将 PCD 点云文件载入到库的点模型。
 * @attacher
 */
PointCloud load_pcd(const std::string& path) { return load_pcd_impl(path); }



/**
 * @brief [en] Replaces the empty-oven baseline frames (in `input_unit`).
 * @brief [zh] 替换空炉基线帧（单位为 `input_unit`）。
 * @attacher
 */
FoodVolumeMeasurer& FoodVolumeMeasurer::set_baseline(const std::vector<PointCloud>& frames) {
    baseline_frames_ = frames;
    return *this;
}



/**
 * @brief [en] Appends one empty-oven baseline frame (in `input_unit`).
 * @brief [zh] 追加一帧空炉基线（单位为 `input_unit`）。
 * @attacher
 */
FoodVolumeMeasurer& FoodVolumeMeasurer::add_baseline_frame(const PointCloud& frame) {
    baseline_frames_.push_back(frame);
    return *this;
}



/**
 * @brief [en] Sets the food point cloud to measure (in `input_unit`).
 * @brief [zh] 设置待测食材点云（单位为 `input_unit`）。
 * @attacher
 */
FoodVolumeMeasurer& FoodVolumeMeasurer::set_food(const PointCloud& cloud) {
    food_ = cloud;
    return *this;
}



/**
 * @brief [en] Sets the linear unit of all input point clouds.
 * @brief [zh] 设置所有输入点云的线性单位。
 * @attacher
 */
FoodVolumeMeasurer& FoodVolumeMeasurer::set_input_unit(LengthUnit unit) {
    cfg_.input_unit = unit;
    return *this;
}



/**
 * @brief [en] Sets the voxel size used for downsampling, in metres.
 * @brief [zh] 设置降采样体素边长（米）。
 * @attacher
 */
FoodVolumeMeasurer& FoodVolumeMeasurer::set_voxel_size(double size_m) {
    cfg_.voxel_size_m = size_m;
    return *this;
}



/**
 * @brief [en] Sets the baseline/integration raster cell size, in metres.
 * @brief [zh] 设置基线/积分栅格边长（米）。
 * @attacher
 */
FoodVolumeMeasurer& FoodVolumeMeasurer::set_integration_resolution(double resolution_m) {
    cfg_.integration_resolution_m = resolution_m;
    return *this;
}



/**
 * @brief [en] Sets the accepted baseline-relative height range of food points, in metres.
 * @brief [zh] 设置食材点相对基线高度的接受范围（米）。
 * @attacher
 */
FoodVolumeMeasurer& FoodVolumeMeasurer::set_height_range(double min_m, double max_m) {
    cfg_.min_height_m = min_m;
    cfg_.max_height_m = max_m;
    return *this;
}



/**
 * @brief [en] Enables an axis-aligned crop region applied before downsampling.
 * @brief [zh] 启用降采样前应用的轴对齐裁剪区域。
 * @attacher
 */
FoodVolumeMeasurer& FoodVolumeMeasurer::set_roi(const AxisAlignedRoi& roi) {
    cfg_.roi = roi;
    cfg_.use_roi = true;
    return *this;
}



/**
 * @brief [en] Disables the axis-aligned crop region.
 * @brief [zh] 禁用轴对齐裁剪区域。
 * @attacher
 */
FoodVolumeMeasurer& FoodVolumeMeasurer::clear_roi() {
    cfg_.use_roi = false;
    return *this;
}



/**
 * @brief [en] Sets how foreground food components are selected.
 * @brief [zh] 设置前景食材块的选择方式。
 * @attacher
 */
FoodVolumeMeasurer& FoodVolumeMeasurer::set_selection_mode(ComponentSelectionMode mode) {
    cfg_.selection_mode = mode;
    return *this;
}



/**
 * @brief [en] Sets the manually selected component labels (used in manual selection mode).
 * @brief [zh] 设置手动选择的连通块标签（手动模式下使用）。
 * @attacher
 */
FoodVolumeMeasurer& FoodVolumeMeasurer::set_selected_labels(const std::vector<int>& labels) {
    cfg_.selected_labels = labels;
    return *this;
}



/**
 * @brief [en] Sets the footprint clustering radius and minimum cluster size.
 * @brief [zh] 设置足迹聚类半径与最小簇规模。
 * @attacher
 */
FoodVolumeMeasurer& FoodVolumeMeasurer::set_cluster_params(double footprint_eps_m, int min_points) {
    cfg_.foreground_cluster_eps_m = footprint_eps_m;
    cfg_.cluster_min_points = min_points;
    return *this;
}



/**
 * @brief [en] Sets the RANSAC inlier distance threshold for background-plane removal.
 * @brief [zh] 设置背景平面移除的 RANSAC 内点距离阈值。
 * @attacher
 */
FoodVolumeMeasurer& FoodVolumeMeasurer::set_plane_distance_threshold(double threshold_m) {
    cfg_.plane_distance_threshold_m = threshold_m;
    return *this;
}



/**
 * @brief [en] Replaces the full measurement configuration, including advanced fields.
 * @brief [zh] 替换完整测量配置（含高级字段）。
 * @attacher
 */
FoodVolumeMeasurer& FoodVolumeMeasurer::set_config(const MeasurementConfig& cfg) {
    cfg_ = cfg;
    return *this;
}



/**
 * @brief [en] Returns the current measurement configuration.
 * @brief [zh] 返回当前测量配置。
 * @attacher
 */
const MeasurementConfig& FoodVolumeMeasurer::config() const { return cfg_; }



/**
 * @brief [en] Loads the configuration from a JSON file, overriding stored values.
 * @brief [zh] 从 JSON 文件载入配置并覆盖已存值。
 * @attacher
 */
bool FoodVolumeMeasurer::load_config_from_json(const std::string& path) {
    return vm::load_config_from_json(path, cfg_);
}



/**
 * @brief [en] Saves the current configuration to a JSON file.
 * @brief [zh] 将当前配置保存到 JSON 文件。
 * @attacher
 */
bool FoodVolumeMeasurer::save_config_to_json(const std::string& path) const {
    return vm::save_config_to_json(cfg_, path);
}



/**
 * @brief [en] Runs the IM pipeline and returns the volume estimate.
 * @brief [zh] 运行 IM 流水线并返回体积估计。
 * @attacher
 */
VolumeEstimate FoodVolumeMeasurer::run() const {
    VM_PROFILE_FUNC();
    const MeasurementConfig& cfg = cfg_;
#ifdef __ARM_EABI__
    (void)cfg;
    return failure(MeasurementStatus::kUnsupportedPlatform, "PCL volume pipeline is unavailable on bare-metal");
#else
    if (baseline_frames_.empty()) {
        return failure(MeasurementStatus::kEmptyBaseline, status_to_string(MeasurementStatus::kEmptyBaseline));
    }
    if (food_.points.empty()) {
        return failure(MeasurementStatus::kEmptyInput, status_to_string(MeasurementStatus::kEmptyInput));
    }

    // 1. Preprocess the food frame (unit-normalize, optional crop, voxel downsample).
    PreprocessResult pre{};
    MeasurementStatus st = preprocess_cloud(food_, cfg, pre);
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
    st = build_baseline_model(baseline_frames_, orientation, cfg, baseline);
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
    // Reference volumes from the merged selected-component cloud.
    const PointCloud merged = merged_components_cloud(components);
    est.aabb_volume_m3 = compute_aabb_volume(merged);
    est.obb_volume_m3 = compute_obb_volume(merged);
    est.convex_hull_volume_m3 = compute_convex_hull_volume(merged);
    est.message = status_to_string(MeasurementStatus::kSuccess);
    log_info("result: status=" + std::string(status_to_string(est.status)) +
             " volume_cm3=" + std::to_string(est.volume_cm3));
    return est;
#endif // __ARM_EABI__
}

} // namespace vm
