// Conan::ImportStart
#pragma once
#include <cstddef>
#include <vector>
#include "volume_baseline.hpp"
#include "volume_component.hpp"
#include "volume_grid.hpp"
#include "volume_types.hpp"
// Conan::ImportEnd



namespace vm {



/**
 * @brief [en] Measures component volume by integrating baseline-relative heights with conservative hole completion.
 * @brief [zh] 通过积分相对基线高度并保守补洞来测量连通块体积。
 * @param components [en] Selected food components produced by `extract_food_components`.
 * @param components [zh] 由 `extract_food_components` 产生的选中食材块。
 * @param baseline [en] Empty-oven baseline model.
 * @param baseline [zh] 空炉基线模型。
 * @param cfg [en] Measurement configuration.
 * @param cfg [zh] 测量配置。
 * @param out [en] Integrated volume and diagnostics.
 * @param out [zh] 积分体积与诊断信息。
 * @return [en] kSuccess, kInvalidConfig, kInsufficientCoverage, or kFoodNotFound.
 * @return [zh] kSuccess、kInvalidConfig、kInsufficientCoverage 或 kFoodNotFound。
 * @exporter
 */
MeasurementStatus measure_component_volume(const FoodComponents& components, const BaselineModel& baseline,
                                           const MeasurementConfig& cfg, ComponentVolumeEstimate& out);



/**
 * @brief [en] Measures the volume of each selected component independently.
 * @brief [zh] 独立测量每个选中连通块的体积。
 * @param components [en] Selected food components produced by `extract_food_components`.
 * @param components [zh] 由 `extract_food_components` 产生的选中食材块。
 * @param baseline [en] Empty-oven baseline model.
 * @param baseline [zh] 空炉基线模型。
 * @param cfg [en] Measurement configuration.
 * @param cfg [zh] 测量配置。
 * @param out [en] One integration result per component, in the same order as `components.labels`.
 * @param out [zh] 每个连通块各一条积分结果，顺序与 `components.labels` 一致。
 * @return [en] kSuccess, or the first per-component failure status.
 * @return [zh] kSuccess，或首个失败连通块的状态。
 * @exporter
 */
MeasurementStatus measure_component_volumes(const FoodComponents& components, const BaselineModel& baseline,
                                            const MeasurementConfig& cfg, std::vector<ComponentVolumeEstimate>& out);



/**
 * @brief [en] Builds the per-cell top surface of the selected components.
 * @brief [zh] 构建选中连通块的逐格顶表面。
 * @param components [en] Selected food components produced by `extract_food_components`.
 * @param components [zh] 由 `extract_food_components` 产生的选中食材块。
 * @param baseline [en] Empty-oven baseline model.
 * @param baseline [zh] 空炉基线模型。
 * @return [en] The top-surface map.
 * @return [zh] 顶表面图。
 * @exporter
 */
SurfaceMap build_top_surface(const FoodComponents& components, const BaselineModel& baseline);



/**
 * @brief [en] Builds the baseline-difference height grid from a top-surface map.
 * @brief [zh] 从顶表面图构建基线差分高度栅格。
 * @param surface [en] Per-cell top surface produced by `build_top_surface`.
 * @param surface [zh] 由 `build_top_surface` 产生的逐格顶表面。
 * @param baseline [en] Empty-oven baseline model.
 * @param baseline [zh] 空炉基线模型。
 * @param cfg [en] Measurement configuration.
 * @param cfg [zh] 测量配置。
 * @param out [en] Height grid with derived statistics.
 * @param out [zh] 带派生统计的高度栅格。
 * @return [en] kSuccess, kInvalidConfig, or kInsufficientCoverage.
 * @return [zh] kSuccess、kInvalidConfig 或 kInsufficientCoverage。
 * @exporter
 */
MeasurementStatus build_height_grid(const SurfaceMap& surface, const BaselineModel& baseline,
                                    const MeasurementConfig& cfg, HeightGrid& out);



/**
 * @brief [en] Completes enclosed holes conservatively inside single food components, in place.
 * @brief [zh] 在单一食材块内部保守地补全封闭孔（原地修改）。
 * @param grid [en] The height grid; interpolated cells are appended in place.
 * @param grid [zh] 高度栅格；补洞格原地追加。
 * @param baseline [en] Empty-oven baseline model.
 * @param baseline [zh] 空炉基线模型。
 * @param cfg [en] Measurement configuration.
 * @param cfg [zh] 测量配置。
 * @param stats [en] Hole-filling statistics.
 * @param stats [zh] 补洞统计。
 * @return [en] kSuccess or kInvalidConfig.
 * @return [zh] kSuccess 或 kInvalidConfig。
 * @exporter
 */
MeasurementStatus complete_holes(HeightGrid& grid, const BaselineModel& baseline, const MeasurementConfig& cfg,
                                 HoleFillStats& stats);



/**
 * @brief [en] Derives a component volume estimate from a height grid.
 * @brief [zh] 从高度栅格派生连通块体积估计。
 * @param grid [en] The height grid with derived statistics.
 * @param grid [zh] 带派生统计的高度栅格。
 * @return [en] The component volume estimate.
 * @return [zh] 连通块体积估计。
 * @exporter
 */
ComponentVolumeEstimate compute_grid_estimate(const HeightGrid& grid);



} // namespace vm
