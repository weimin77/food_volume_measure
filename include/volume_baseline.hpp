// Conan::ImportStart
#pragma once
#include <cstddef>
#include <memory>
#include <vector>
#include "volume_grid.hpp"
#include "volume_types.hpp"
// Conan::ImportEnd



namespace vm {



/**
 * @brief [en] A reusable empty-oven baseline for the IM pipeline.
 * @brief [zh] IM 流水线可复用的空炉基线。
 * @exporter
 */
struct BaselineModel {
    std::size_t frame_count = 0;
    std::size_t cell_count = 0;
    std::shared_ptr<BaselineData> data;
};



/**
 * @brief [en] Builds the empty-oven baseline model from one or more baseline frames.
 * @brief [zh] 从一帧或多帧空炉点云建立空炉基线模型。
 * @param baseline_frames [en] Empty-oven point clouds in `cfg.input_unit`.
 * @param baseline_frames [zh] 以 `cfg.input_unit` 为单位的空炉点云。
 * @param orientation_points [en] Food-side points used to orient the plane normal; may be empty.
 * @param orientation_points [zh] 用于确定平面法向的食材侧点；可为空。
 * @param cfg [en] Measurement configuration.
 * @param cfg [zh] 测量配置。
 * @param out [en] Receives the built baseline model.
 * @param out [zh] 接收构建好的基线模型。
 * @return [en] kSuccess, kEmptyBaseline, kNonFiniteBaseline, kInvalidConfig, kPlaneNotFound, or kBaselineNoCells.
 * @return [zh] kSuccess、kEmptyBaseline、kNonFiniteBaseline、kInvalidConfig、kPlaneNotFound 或 kBaselineNoCells。
 * @exporter
 */
MeasurementStatus build_baseline_model(const std::vector<PointCloud>& baseline_frames,
                                       const PointCloud& orientation_points, const MeasurementConfig& cfg,
                                       BaselineModel& out);



/**
 * @brief [en] Builds the local orthonormal (u,v,n) frame for a plane and an origin.
 * @brief [zh] 为平面和原点构建局部正交 (u,v,n) 坐标系。
 * @param plane [en] Normalized Hessian plane; its normal defines the n axis.
 * @param plane [zh] 归一化的 Hessian 平面；其法向定义 n 轴。
 * @param origin [en] Frame origin, typically the mean of the plane inliers.
 * @param origin [zh] 坐标系原点，通常取平面内点均值。
 * @return [en] The orthonormal frame.
 * @return [zh] 正交坐标系。
 * @exporter
 */
PlaneFrame build_plane_frame(const Plane& plane, const Point3f& origin);



/**
 * @brief [en] Projects baseline frames into a plane frame and rasterizes per-cell median heights.
 * @brief [zh] 把基线帧投影到平面坐标系，并栅格化出逐格高度中位数。
 * @param baseline_frames [en] Empty-oven frames in metres.
 * @param baseline_frames [zh] 以米为单位的空炉帧。
 * @param frame [en] The plane frame to project into.
 * @param frame [zh] 投影目标平面坐标系。
 * @param cell_size_m [en] Raster cell edge length in metres.
 * @param cell_size_m [zh] 栅格边长（米）。
 * @param max_surface_height_m [en] Maximum accepted height above the plane.
 * @param max_surface_height_m [zh] 平面以上可接受的最大高度。
 * @param out [en] Receives the per-cell median heightmap and its bounding box.
 * @param out [zh] 接收逐格高度中位数图及其外接框。
 * @return [en] kSuccess or kBaselineNoCells.
 * @return [zh] kSuccess 或 kBaselineNoCells。
 * @exporter
 */
MeasurementStatus rasterize_baseline(const std::vector<PointCloud>& baseline_frames, const PlaneFrame& frame,
                                     double cell_size_m, double max_surface_height_m, BaselineData& out);



/**
 * @brief [en] Computes the inset plane ROI from a baseline footprint and stores it in the baseline.
 * @brief [zh] 从基线足迹计算内缩平面 ROI 并写回基线。
 * @param baseline [en] Baseline whose `roi` is filled and whose bbox bounds are used.
 * @param baseline [zh] 要填充 `roi` 的基线；使用其 bbox 边界。
 * @param border_margin_m [en] Inset distance from the footprint bbox.
 * @param border_margin_m [zh] 相对足迹外接框的内缩距离。
 * @return [en] True when a non-empty ROI was produced.
 * @return [zh] 产生非空 ROI 时为真。
 * @exporter
 */
bool build_plane_roi(BaselineData& baseline, double border_margin_m);



} // namespace vm