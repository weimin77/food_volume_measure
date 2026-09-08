// Conan::ImportStart
#pragma once
#include <cstddef>
#include <vector>
#include "volume_baseline.hpp"
#include "volume_types.hpp"
// Conan::ImportEnd



namespace vm {



/**
 * @brief [en] Selected foreground food components: stable public labels plus their point clouds.
 * @brief [zh] 选中的前景食材块：稳定的公开标签及其点云。
 * @exporter
 */
struct FoodComponents {
    std::size_t cluster_count = 0;  // total non-noise DBSCAN clusters found before selection
    std::vector<int> labels;        // ascending, one entry per component
    std::vector<PointCloud> clouds; // one cloud per label, same order as `labels`
};



/**
 * @brief [en] Filters food points by baseline-relative height, clusters them in the baseline plane, and selects components.
 * @brief [zh] 按相对基线高度过滤食材点，在基准面内聚类并选择连通块。
 * @param food_m [en] Food points in metres after background-plane removal.
 * @param food_m [zh] 移除背景平面后以米为单位的食材点。
 * @param baseline [en] Empty-oven baseline model.
 * @param baseline [zh] 空炉基线模型。
 * @param cfg [en] Measurement configuration.
 * @param cfg [zh] 测量配置。
 * @param out [en] Selected component labels and their point clouds.
 * @param out [zh] 选中的连通块标签及其点云。
 * @return [en] kSuccess, kInsufficientCoverage, kFoodNotFound, or kInvalidConfig.
 * @return [zh] kSuccess、kInsufficientCoverage、kFoodNotFound 或 kInvalidConfig。
 * @exporter
 */
MeasurementStatus extract_food_components(const PointCloud& food_m, const BaselineModel& baseline,
                                          const MeasurementConfig& cfg, FoodComponents& out);



/**
 * @brief [en] Filters food points by baseline-relative height and the inset plane ROI.
 * @brief [zh] 按相对基线高度和内缩平面 ROI 过滤食材点。
 * @param food_m [en] Food points in metres after background-plane removal.
 * @param food_m [zh] 移除背景平面后以米为单位的食材点。
 * @param baseline [en] Empty-oven baseline model.
 * @param baseline [zh] 空炉基线模型。
 * @param cfg [en] Measurement configuration.
 * @param cfg [zh] 测量配置。
 * @param dense_out [en] Dense valid foreground points.
 * @param dense_out [zh] 稠密的有效前景点。
 * @return [en] kSuccess, kInvalidConfig, or kInsufficientCoverage.
 * @return [zh] kSuccess、kInvalidConfig 或 kInsufficientCoverage。
 * @exporter
 */
MeasurementStatus filter_baseline_difference(const PointCloud& food_m, const BaselineModel& baseline,
                                             const MeasurementConfig& cfg, PointCloud& dense_out);



/**
 * @brief [en] Projects points into the baseline plane frame as (u, v, 0) for planar clustering.
 * @brief [zh] 把点投影到基准面坐标系为 (u,v,0)，用于平面内聚类。
 * @param cloud [en] Input cloud in metres.
 * @param cloud [zh] 以米为单位的输入点云。
 * @param baseline [en] Empty-oven baseline model.
 * @param baseline [zh] 空炉基线模型。
 * @return [en] Projected points with z = 0.
 * @return [zh] z=0 的投影点。
 * @exporter
 */
PointCloud project_to_plane(const PointCloud& cloud, const BaselineModel& baseline);



/**
 * @brief [en] Selects components from DBSCAN labels by minimum size and selection mode.
 * @brief [zh] 按最小尺寸与选择模式从 DBSCAN 标签中选取连通块。
 * @param labels [en] Per-point cluster labels (-1 marks noise).
 * @param labels [zh] 逐点簇标签（-1 表示噪声）。
 * @param cloud [en] Points corresponding one-to-one to `labels`.
 * @param cloud [zh] 与 `labels` 一一对应的点。
 * @param cfg [en] Measurement configuration.
 * @param cfg [zh] 测量配置。
 * @param out [en] Selected component labels and their point clouds.
 * @param out [zh] 选中的连通块标签及其点云。
 * @return [en] kSuccess, kInvalidConfig, or kFoodNotFound.
 * @return [zh] kSuccess、kInvalidConfig 或 kFoodNotFound。
 * @exporter
 */
MeasurementStatus select_components(const std::vector<int>& labels, const PointCloud& cloud,
                                    const MeasurementConfig& cfg, FoodComponents& out);



} // namespace vm