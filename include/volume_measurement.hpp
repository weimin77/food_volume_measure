// Conan::ImportStart
#pragma once
#include <memory>
#include <string>
#include <vector>
#include "volume_types.hpp"
// Conan::ImportEnd



namespace vm {



/**
 * @brief [en] Chain-configurable food volume measurer: set inputs and parameters, then call `run`.
 * @brief [zh] 链式配置的食材体积量测器：设置输入与参数后调用 `run` 即可测量。
 *
 * @details [en] All setters return a reference to this instance so calls can be chained.
 *     Inputs and parameters are stored on the instance; `run` executes the full IM pipeline
 *     (preprocess → background-plane removal → baseline → component extraction → integration
 *     with hole completion) and returns a `VolumeEstimate`.
 * @details [zh] 所有 setter 返回本实例引用，可链式调用。输入与参数保存在实例中；
 *     `run` 执行完整 IM 流水线（预处理 → 背景平面移除 → 基线 → 连通块提取 → 含补洞的积分），
 *     返回 `VolumeEstimate`。
 * @exporter
 */
class FoodVolumeMeasurer {
  public:
    FoodVolumeMeasurer() = default;

    // ---------- inputs ----------

    /**
     * @brief [en] Replaces the empty-oven baseline frames (in `input_unit`).
     * @brief [zh] 替换空炉基线帧（单位为 `input_unit`）。
     * @exporter
     */
    FoodVolumeMeasurer& set_baseline(const std::vector<PointCloud>& frames);

    /**
     * @brief [en] Appends one empty-oven baseline frame (in `input_unit`).
     * @brief [zh] 追加一帧空炉基线（单位为 `input_unit`）。
     * @exporter
     */
    FoodVolumeMeasurer& add_baseline_frame(const PointCloud& frame);

    /**
     * @brief [en] Sets the food point cloud to measure (in `input_unit`).
     * @brief [zh] 设置待测食材点云（单位为 `input_unit`）。
     * @exporter
     */
    FoodVolumeMeasurer& set_food(const PointCloud& cloud);

    // ---------- common parameters ----------

    /**
     * @brief [en] Sets the linear unit of all input point clouds.
     * @brief [zh] 设置所有输入点云的线性单位。
     * @exporter
     */
    FoodVolumeMeasurer& set_input_unit(LengthUnit unit);

    /**
     * @brief [en] Sets the voxel size used for downsampling, in metres.
     * @brief [zh] 设置降采样体素边长（米）。
     * @exporter
     */
    FoodVolumeMeasurer& set_voxel_size(double size_m);

    /**
     * @brief [en] Sets the baseline/integration raster cell size, in metres.
     * @brief [zh] 设置基线/积分栅格边长（米）。
     * @exporter
     */
    FoodVolumeMeasurer& set_integration_resolution(double resolution_m);

    /**
     * @brief [en] Sets the accepted baseline-relative height range of food points, in metres.
     * @brief [zh] 设置食材点相对基线高度的接受范围（米）。
     * @param min_m [en] Minimum accepted height; must be non-negative.
     * @param min_m [zh] 最小接受高度，必须非负。
     * @param max_m [en] Maximum accepted height; values <= 0 disable the cap.
     * @param max_m [zh] 最大接受高度；<= 0 表示关闭上限。
     * @exporter
     */
    FoodVolumeMeasurer& set_height_range(double min_m, double max_m);

    /**
     * @brief [en] Enables an axis-aligned crop region applied before downsampling.
     * @brief [zh] 启用降采样前应用的轴对齐裁剪区域。
     * @exporter
     */
    FoodVolumeMeasurer& set_roi(const AxisAlignedRoi& roi);

    /**
     * @brief [en] Disables the axis-aligned crop region.
     * @brief [zh] 禁用轴对齐裁剪区域。
     * @exporter
     */
    FoodVolumeMeasurer& clear_roi();

    /**
     * @brief [en] Sets how foreground food components are selected.
     * @brief [zh] 设置前景食材块的选择方式。
     * @exporter
     */
    FoodVolumeMeasurer& set_selection_mode(ComponentSelectionMode mode);

    /**
     * @brief [en] Sets the manually selected component labels (used in manual selection mode).
     * @brief [zh] 设置手动选择的连通块标签（手动模式下使用）。
     * @exporter
     */
    FoodVolumeMeasurer& set_selected_labels(const std::vector<int>& labels);

    /**
     * @brief [en] Sets the footprint clustering radius and minimum cluster size.
     * @brief [zh] 设置足迹聚类半径与最小簇规模。
     * @param footprint_eps_m [en] DBSCAN neighbourhood radius in the baseline plane, in metres.
     * @param footprint_eps_m [zh] 基准面内 DBSCAN 邻域半径（米）。
     * @param min_points [en] Minimum cluster size including the point itself; must be positive.
     * @param min_points [zh] 构成簇所需的最小邻域点数（含自身），必须为正。
     * @exporter
     */
    FoodVolumeMeasurer& set_cluster_params(double footprint_eps_m, int min_points);

    /**
     * @brief [en] Sets the RANSAC inlier distance threshold for background-plane removal.
     * @brief [zh] 设置背景平面移除的 RANSAC 内点距离阈值。
     * @exporter
     */
    FoodVolumeMeasurer& set_plane_distance_threshold(double threshold_m);

    /**
     * @brief [en] Enables or disables the dump of intermediate stage point clouds.
     * @brief [zh] 启用或禁用中间阶段点云的落盘。
     * @details [en] When enabled, `run` writes one PCD per pipeline stage
     *     (input, downsampled, plane removed, baseline surface, food components, top surface)
     *     into the directory set by `set_middle_cloud_dir`. Every selected food component is
     *     also written separately, as `4_food_component_label<NN>.pcd`, so a multi-food frame
     *     can be inspected one connected region at a time.
     * @details [zh] 启用后 `run` 会把流水线各阶段（输入、降采样后、去平面后、基线表面、
     *     食材块、顶表面）各写入一个 PCD 到 `set_middle_cloud_dir` 指定的目录。
     *     每个选中的食材连通域还会单独写出为 `4_food_component_label<NN>.pcd`，
     *     便于对多食材帧逐个连通域排查。
     * @param enabled [en] True to write the stage clouds.
     * @param enabled [zh] 为真时写出各阶段点云。
     * @exporter
     */
    FoodVolumeMeasurer& set_save_middle_cloud(bool enabled);

    /**
     * @brief [en] Sets the output directory of the intermediate stage point clouds.
     * @brief [zh] 设置中间阶段点云的输出目录。
     * @param dir [en] Directory path; created when missing.
     * @param dir [zh] 目录路径；不存在时自动创建。
     * @exporter
     */
    FoodVolumeMeasurer& set_middle_cloud_dir(const std::string& dir);

    // ---------- full configuration ----------

    /**
     * @brief [en] Replaces the full measurement configuration, including advanced fields.
     * @brief [zh] 替换完整测量配置（含高级字段）。
     * @exporter
     */
    FoodVolumeMeasurer& set_config(const MeasurementConfig& cfg);

    /**
     * @brief [en] Returns the current measurement configuration.
     * @brief [zh] 返回当前测量配置。
     * @exporter
     */
    const MeasurementConfig& config() const;

    /**
     * @brief [en] Loads the configuration from a JSON file, overriding stored values.
     * @brief [zh] 从 JSON 文件载入配置并覆盖已存值。
     * @exporter
     */
    bool load_config_from_json(const std::string& path);

    /**
     * @brief [en] Saves the current configuration to a JSON file.
     * @brief [zh] 将当前配置保存到 JSON 文件。
     * @exporter
     */
    bool save_config_to_json(const std::string& path) const;

    // ---------- execution ----------

    /**
     * @brief [en] Builds and caches the empty-oven baseline model so repeated `run` calls skip it.
     * @brief [zh] 构建并缓存空炉基线模型，使后续 `run` 调用无需重复构建。
     *
     * @details [en] The baseline depends only on the baseline frames (and the parameters that
     *     shape them), not on the food frame, so it is worth building once when several food
     *     frames are measured against the same empty oven. The food frame set by `set_food` is
     *     used only to orient the baseline plane normal, so `set_food` must be called first.
     *     Any later call that changes the baseline frames or the baseline-shaping parameters
     *     (input unit, voxel size, integration resolution, plane threshold, cluster parameters,
     *     or a whole-config replacement) discards the cache; `set_food` does not, which is what
     *     lets one prepared baseline serve several food frames.
     *     `run` works with or without a prepared baseline; when one is cached it reuses it.
     * @details [zh] 基线只取决于基线帧（以及塑造基线的参数），与食材帧无关，因此在「同一个空炉、
     *     多次测量不同食材」的场景下值得只构建一次。`set_food` 设置的食材帧仅用于确定基准面法向的
     *     朝向，所以必须先调用 `set_food`。之后任何改变基线帧或基线相关参数的调用（输入单位、体素
     *     边长、积分分辨率、平面阈值、聚类参数，或整体替换配置）都会丢弃缓存；`set_food` 不会丢弃，
     *     这正是「一份基线服务多个食材帧」的关键。有缓存时 `run` 复用，没有时 `run` 自行构建。
     * @return [en] kSuccess, or the first failing stage status.
     * @return [zh] kSuccess，或首个失败阶段的状态。
     * @exporter
     */
    MeasurementStatus prepare_baseline();

    /**
     * @brief [en] Returns whether a prepared baseline model is currently cached.
     * @brief [zh] 返回当前是否已缓存了准备好的基线模型。
     * @exporter
     */
    bool has_prepared_baseline() const;

    /**
     * @brief [en] Runs the IM pipeline and returns the volume estimate.
     * @brief [zh] 运行 IM 流水线并返回体积估计。
     * @return [en] A VolumeEstimate whose status indicates success or the first failing stage.
     * @return [zh] 一个 VolumeEstimate，其状态指示成功或第一个失败阶段。
     * @exporter
     */
    VolumeEstimate run() const;

  private:
    // Opaque cache holding the prebuilt baseline model; defined in the implementation.
    struct PreparedBaseline;

    // Drops the cached baseline; called by every setter that shapes the baseline inputs.
    void invalidate_prepared_baseline();

    MeasurementConfig cfg_;
    std::vector<PointCloud> baseline_frames_;
    PointCloud food_;
    std::shared_ptr<PreparedBaseline> prepared_baseline_;
};



/**
 * @brief [en] Loads a PCD point-cloud file into the library's point model.
 * @brief [zh] 将 PCD 点云文件载入到库的点模型。
 * @param path [en] Path to the PCD file.
 * @param path [zh] PCD 文件路径。
 * @return [en] The loaded points, or an empty cloud when the file cannot be read.
 * @return [zh] 载入的点；文件无法读取时返回空点云。
 * @exporter
 */
PointCloud load_pcd(const std::string& path);



} // namespace vm
