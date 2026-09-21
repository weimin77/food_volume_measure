// Conan::ImportStart
#pragma once
#include <cstddef>
#include <vector>
#include "baseline.hpp"
#include "component.hpp"
#include "grid.hpp"
#include "types.hpp"
// Conan::ImportEnd



/**
 * @brief [en] Measures component volume by integrating baseline-relative heights with conservative hole completion.
 * @brief [zh] 通过积分相对基线高度并保守补洞来测量连通块体积。
 * @param components [en] Selected food components produced by `extract_food_components`.
 * @param components [zh] 由 `extract_food_components` 产生的选中食材块。
 * @param baseline [en] Empty-oven baseline model.
 * @param baseline [zh] 空炉基线模型。
 * @param cfg [en] Measurement configuration.
 * @param cfg [zh] 测量配置。
 * @param out [en] Integrated volume and diagnostics, merged over all selected components.
 * @param out [zh] 合并所有选中连通块后的积分体积与诊断信息。
 * @param per_component [en] When non-null, receives one estimate per selected component, in the
 *     same order as `components.labels`. Derived from the same rasterisation pass, so requesting
 *     it costs no extra integration work.
 * @param per_component [zh] 非空时按 `components.labels` 顺序逐连通块写入一条估计；
 *     与合并结果共用同一遍栅格化，因此不产生额外积分开销。
 * @return [en] kSuccess, kInvalidConfig, kInsufficientCoverage, or kFoodNotFound.
 * @return [zh] kSuccess、kInvalidConfig、kInsufficientCoverage 或 kFoodNotFound。

 */
MeasurementStatus measure_component_volume(const FoodComponents& components, const BaselineModel& baseline,
                                           const MeasurementConfig& cfg, ComponentVolumeEstimate& out,
                                           std::vector<ComponentVolumeEstimate>* per_component = nullptr);



/**
 * @brief [en] Builds the per-cell top surface of the selected components.
 * @brief [zh] 构建选中连通块的逐格顶表面。
 * @param components [en] Selected food components produced by `extract_food_components`.
 * @param components [zh] 由 `extract_food_components` 产生的选中食材块。
 * @param baseline [en] Empty-oven baseline model.
 * @param baseline [zh] 空炉基线模型。
 * @return [en] The top-surface map.
 * @return [zh] 顶表面图。

 */
SurfaceMap build_top_surface(const FoodComponents& components, const BaselineModel& baseline);
