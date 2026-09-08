// Conan::ImportStart
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <vector>
#include "volume_component.hpp"
#include "volume_internal.hpp"
#include "volume_log.hpp"
#include "volume_pointcloudprocess.hpp"
// Conan::ImportEnd



namespace vm {



namespace {

// Filters food points by baseline-relative height and the inset plane ROI, producing the dense foreground cloud.
MeasurementStatus build_valid_difference_points(const PointCloud& food_m, const BaselineData& baseline,
                                                const PlaneRoi& roi, const MeasurementConfig& cfg,
                                                PointCloud& dense_out) {
    dense_out = PointCloud{};

    if (cfg.baseline_fill_radius_cells < 0 || !(cfg.min_height_m >= 0.0F)) {
        return MeasurementStatus::kInvalidConfig;
    }

    dense_out.points.reserve(food_m.points.size());
    for (const auto& p : food_m.points) {
        double u = 0.0;
        double v = 0.0;
        double h = 0.0;
        project_to_plane_frame(baseline.frame, p, u, v, h);
        if (!std::isfinite(h) || h < kFoodMinHeightM) {
            continue;
        }

        const CellKey key = cell_index_of(u, v, baseline.cell_size_m);
        const double center_u = (static_cast<double>(key.first) + 0.5) * baseline.cell_size_m;
        const double center_v = (static_cast<double>(key.second) + 0.5) * baseline.cell_size_m;
        if (!inside_roi(center_u, center_v, roi)) {
            continue;
        }

        double baseline_height = 0.0;
        if (!lookup_baseline_height(baseline, key, cfg.baseline_fill_radius_cells, baseline_height)) {
            continue;
        }

        double diff = h - baseline_height;
        if (diff < cfg.min_height_m) {
            continue;
        }
        if (cfg.max_height_m > 0.0F && diff > cfg.max_height_m) {
            diff = cfg.max_height_m;
        }
        (void)diff;
        dense_out.points.push_back(p);
    }

    if (dense_out.points.empty()) {
        return MeasurementStatus::kInsufficientCoverage;
    }
    return MeasurementStatus::kSuccess;
}

} // namespace



/**
 * @brief [en] Filters food points by baseline-relative height, clusters them in the baseline plane, and selects components.
 * @brief [zh] 按相对基线高度过滤食材点，在基准面内聚类并选择连通块。
 * @attacher
 */
MeasurementStatus extract_food_components(const PointCloud& food_m, const BaselineModel& baseline,
                                          const MeasurementConfig& cfg, FoodComponents& out) {
#ifdef __ARM_EABI__
    (void)food_m;
    (void)baseline;
    (void)cfg;
    (void)out;
    return MeasurementStatus::kUnsupportedPlatform;
#else
    out = FoodComponents{};

    if (!baseline.data) {
        return MeasurementStatus::kInvalidConfig;
    }
    if (food_m.points.empty()) {
        return MeasurementStatus::kInsufficientCoverage;
    }
    if (!(cfg.foreground_cluster_eps_m > 0.0F) || cfg.cluster_min_points <= 0 || cfg.baseline_fill_radius_cells < 0 ||
        !(cfg.min_height_m >= 0.0F)) {
        return MeasurementStatus::kInvalidConfig;
    }

    PointCloud dense{};
    MeasurementStatus st = filter_baseline_difference(food_m, baseline, cfg, dense);
    if (st != MeasurementStatus::kSuccess) {
        return st;
    }

    // Project valid points onto the baseline plane and cluster with Open3D-equivalent DBSCAN.
    const PointCloud projected = project_to_plane(dense, baseline);
    const std::vector<int> labels =
        dbscan_labels(projected.points, cfg.foreground_cluster_eps_m, cfg.cluster_min_points);

    return select_components(labels, dense, cfg, out);
#endif // __ARM_EABI__
}



MeasurementStatus filter_baseline_difference(const PointCloud& food_m, const BaselineModel& baseline,
                                             const MeasurementConfig& cfg, PointCloud& dense_out) {
#ifdef __ARM_EABI__
    (void)food_m;
    (void)baseline;
    (void)cfg;
    (void)dense_out;
    return MeasurementStatus::kUnsupportedPlatform;
#else
    if (!baseline.data) {
        return MeasurementStatus::kInvalidConfig;
    }
    const BaselineData& data = *baseline.data;
    return build_valid_difference_points(food_m, data, data.roi, cfg, dense_out);
#endif // __ARM_EABI__
}



PointCloud project_to_plane(const PointCloud& cloud, const BaselineModel& baseline) {
#ifdef __ARM_EABI__
    (void)cloud;
    (void)baseline;
    return PointCloud{};
#else
    PointCloud out;
    if (!baseline.data) {
        return out;
    }
    const PlaneFrame& frame = baseline.data->frame;
    out.points.reserve(cloud.points.size());
    for (const auto& p : cloud.points) {
        double u = 0.0;
        double v = 0.0;
        double h = 0.0;
        project_to_plane_frame(frame, p, u, v, h);
        out.points.push_back(Point3f{static_cast<float>(u), static_cast<float>(v), 0.0F});
    }
    return out;
#endif // __ARM_EABI__
}



MeasurementStatus select_components(const std::vector<int>& labels, const PointCloud& cloud,
                                    const MeasurementConfig& cfg, FoodComponents& out) {
#ifdef __ARM_EABI__
    (void)labels;
    (void)cloud;
    (void)cfg;
    (void)out;
    return MeasurementStatus::kUnsupportedPlatform;
#else
    out = FoodComponents{};
    if (labels.size() != cloud.points.size() || cfg.cluster_min_points <= 0) {
        return MeasurementStatus::kInvalidConfig;
    }

    const int min_candidate_size = std::max(32, cfg.cluster_min_points * 4);
    std::map<int, std::size_t> counts;
    for (const int label : labels) {
        if (label >= 0) {
            counts[label] += 1;
        }
    }
    if (counts.empty()) {
        return MeasurementStatus::kFoodNotFound;
    }

    out.cluster_count = counts.size();

    std::vector<int> selected;
    if (cfg.selection_mode == ComponentSelectionMode::kAllEligible) {
        for (const auto& entry : counts) {
            if (entry.second >= static_cast<std::size_t>(min_candidate_size)) {
                selected.push_back(entry.first);
            }
        }
    } else {
        for (const int label : cfg.selected_labels) {
            const auto it = counts.find(label);
            if (it != counts.end() && it->second >= static_cast<std::size_t>(min_candidate_size)) {
                selected.push_back(label);
            }
        }
        std::sort(selected.begin(), selected.end());
    }
    if (selected.empty()) {
        return MeasurementStatus::kFoodNotFound;
    }

    std::map<int, std::size_t> slot_by_label;
    for (std::size_t i = 0; i < selected.size(); ++i) {
        slot_by_label[selected[i]] = i;
    }

    std::vector<std::size_t> per_point_slot(cloud.points.size(), static_cast<std::size_t>(-1));
    for (std::size_t i = 0; i < labels.size(); ++i) {
        if (labels[i] >= 0) {
            const auto it = slot_by_label.find(labels[i]);
            if (it != slot_by_label.end()) {
                per_point_slot[i] = it->second;
            }
        }
    }

    out.labels = selected;
    out.clouds.resize(selected.size());
    std::vector<std::size_t> counts_per_slot(selected.size(), 0);
    for (const std::size_t slot : per_point_slot) {
        if (slot != static_cast<std::size_t>(-1)) {
            counts_per_slot[slot] += 1;
        }
    }
    for (std::size_t s = 0; s < selected.size(); ++s) {
        out.clouds[s].points.reserve(counts_per_slot[s]);
    }
    for (std::size_t i = 0; i < per_point_slot.size(); ++i) {
        if (per_point_slot[i] != static_cast<std::size_t>(-1)) {
            out.clouds[per_point_slot[i]].points.push_back(cloud.points[i]);
        }
    }
    log_info("components: clusters=" + std::to_string(out.cluster_count) +
             " selected=" + std::to_string(out.labels.size()));
    return MeasurementStatus::kSuccess;
#endif // __ARM_EABI__
}



} // namespace vm