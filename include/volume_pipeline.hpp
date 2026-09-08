// Conan::ImportStart
#pragma once
#include <vector>
#include "volume_types.hpp"
// Conan::ImportEnd



namespace vm {



/**
 * @brief [en] End-to-end IM food volume measurement pipeline.
 * @brief [zh] 端到端的 IM 食材体积测量流水线。
 * @exporter
 */
class VolumePipeline {
  public:
    VolumePipeline() = default;

    /**
     * @brief [en] Measures food volume in cubic centimeters from empty-oven baseline frames and a food frame.
     * @brief [zh] 从空炉基线帧与食材帧测量食材体积（立方厘米）。
     * @param baseline_frames [en] One or more empty-oven point clouds in `cfg.input_unit`.
     * @param baseline_frames [zh] 一帧或多帧以 `cfg.input_unit` 为单位的空炉点云。
     * @param food_frame [en] The food point cloud in `cfg.input_unit`.
     * @param food_frame [zh] 以 `cfg.input_unit` 为单位的食材点云。
     * @param cfg [en] Measurement configuration.
     * @param cfg [zh] 测量配置。
     * @return [en] A VolumeEstimate whose status indicates success or the first failing stage.
     * @return [zh] 一个 VolumeEstimate，其状态指示成功或第一个失败阶段。
     */
    VolumeEstimate measure(const std::vector<PointCloud>& baseline_frames, const PointCloud& food_frame,
                           const MeasurementConfig& cfg) const;
};



/**
 * @brief [en] Computes the axis-aligned bounding-box volume of a point cloud in cubic metres.
 * @brief [zh] 计算点云的轴对齐包围盒体积（立方米）。
 * @param cloud [en] Points in metres.
 * @param cloud [zh] 以米为单位的点。
 * @return [en] AABB volume, or NaN when the cloud is empty.
 * @return [zh] AABB 体积；点云为空时为 NaN。
 * @exporter
 */
double compute_aabb_volume(const PointCloud& cloud);



/**
 * @brief [en] Computes the moment-based oriented bounding-box volume of a point cloud in cubic metres.
 * @brief [zh] 计算点云基于矩的定向包围盒体积（立方米）。
 * @param cloud [en] Points in metres.
 * @param cloud [zh] 以米为单位的点。
 * @return [en] OBB volume, or NaN when the cloud is empty.
 * @return [zh] OBB 体积；点云为空时为 NaN。
 * @exporter
 */
double compute_obb_volume(const PointCloud& cloud);



/**
 * @brief [en] Computes the convex-hull volume of a point cloud in cubic metres.
 * @brief [zh] 计算点云的凸包体积（立方米）。
 * @param cloud [en] Points in metres.
 * @param cloud [zh] 以米为单位的点。
 * @return [en] Convex-hull volume, or NaN when it cannot be computed.
 * @return [zh] 凸包体积；无法计算时为 NaN。
 * @exporter
 */
double compute_convex_hull_volume(const PointCloud& cloud);



} // namespace vm
