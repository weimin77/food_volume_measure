// Conan::ImportStart
#pragma once
#include <string>
#include <vector>
#include "volume_types.hpp"
// Conan::ImportEnd



namespace vm {



/**
 * @brief [en] End-to-end IM food volume measurement pipeline.
 * @brief [zh] 端到端的 IM 食材体积测量流水线。
 * @exporter
 */
class VolumeMeasurement {
  public:
    VolumeMeasurement() = default;

    /**
     * @brief [en] Sets the measurement configuration used by `measure`.
     * @brief [zh] 设置 `measure` 使用的测量配置。
     * @param cfg [en] The configuration to use.
     * @param cfg [zh] 要使用的配置。
     * @exporter
     */
    void set_config(const MeasurementConfig& cfg);

    /**
     * @brief [en] Returns the current measurement configuration.
     * @brief [zh] 返回当前测量配置。
     * @return [en] The stored configuration.
     * @return [zh] 存储的配置。
     * @exporter
     */
    const MeasurementConfig& config() const;

    /**
     * @brief [en] Saves the current configuration to a JSON file.
     * @brief [zh] 将当前配置保存到 JSON 文件。
     * @param path [en] Output JSON file path.
     * @param path [zh] 输出的 JSON 文件路径。
     * @return [en] True when the file was written successfully.
     * @return [zh] 文件成功写入时为真。
     * @exporter
     */
    bool save_config_to_json(const std::string& path) const;

    /**
     * @brief [en] Loads the configuration from a JSON file into this instance.
     * @brief [zh] 从 JSON 文件载入配置到本实例。
     * @param path [en] JSON config file path.
     * @param path [zh] JSON 配置文件路径。
     * @return [en] True when the file was parsed successfully.
     * @return [zh] 文件解析成功时为真。
     * @exporter
     */
    bool load_config_from_json(const std::string& path);

    /**
     * @brief [en] Measures food volume in cubic centimeters from empty-oven baseline frames and a food frame.
     * @brief [zh] 从空炉基线帧与食材帧测量食材体积（立方厘米）。
     * @param baseline_frames [en] One or more empty-oven point clouds in `cfg.input_unit`.
     * @param baseline_frames [zh] 一帧或多帧以 `cfg.input_unit` 为单位的空炉点云。
     * @param food_frame [en] The food point cloud in `cfg.input_unit`.
     * @param food_frame [zh] 以 `cfg.input_unit` 为单位的食材点云。
     * @note [en] The measurement configuration is taken from the instance's `cfg_` member.
     * @note [zh] 测量配置取自实例的 `cfg_` 成员。
     * @return [en] A VolumeEstimate whose status indicates success or the first failing stage.
     * @return [zh] 一个 VolumeEstimate，其状态指示成功或第一个失败阶段。
     * @exporter
     */
    VolumeEstimate measure(const std::vector<PointCloud>& baseline_frames, const PointCloud& food_frame) const;

  private:
    MeasurementConfig cfg_; // 测量配置
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
