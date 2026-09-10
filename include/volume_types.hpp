// Conan::ImportStart
#pragma once
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>
// Conan::ImportEnd



namespace vm {



/**
 * @brief [en] Default parameter values shared by the IM volume pipeline.
 * @brief [zh] IM 体积流水线共享的默认参数值。
 */
namespace volume_defaults {

constexpr double kVoxelSizeM = 0.003;
constexpr double kPlaneDistanceM = 0.003;
constexpr int kPlaneRansacIterations = 1000;
constexpr double kBaselineMaxSurfaceHeightM = 0.03;
constexpr double kClusterEpsM = 0.02;
constexpr double kForegroundClusterEpsM = 0.010;
constexpr int kClusterMinPoints = 8;
constexpr double kIntegrationResolutionM = 0.003;
constexpr double kRoiBorderMarginM = 0.02;
constexpr double kMinHeightM = 0.0015;
constexpr int kBaselineFillRadiusCells = 2;
constexpr int kHoleFillMaxCells = 9;
constexpr int kHoleFillNeighborRadiusCells = 2;
constexpr double kHoleFillMaxNeighborDeltaM = 0.012;
constexpr double kCurveFillMaxHoleAreaCm2 = 8.0;
constexpr double kCurveFillMaxComponentAreaRatio = 0.30;
constexpr double kCurveFillMaxImputedRatio = 0.25;
constexpr int kCurveFillRimRadiusCells = 4;
constexpr int kCurveFillMinRimSamples = 16;
constexpr double kCurveFillMinRimCoverage = 0.75;
constexpr double kCurveFillMaxFitRmseM = 0.004;
constexpr double kCurveFillMaxPredictionRiseM = 0.015;

} // namespace volume_defaults



/**
 * @brief [en] Linear unit of the input point cloud.
 * @brief [zh] 输入点云的线性单位。
 * @exporter
 */
enum class LengthUnit : std::uint8_t {
    kMeter = 0,
    kMillimeter = 1,
};



/**
 * @brief [en] A single 3D point with single precision.
 * @brief [zh] 单精度三维点。
 * @exporter
 */
struct Point3f {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
};



/**
 * @brief [en] An ordered collection of 3D points.
 * @brief [zh] 有序三维点集合。
 * @exporter
 */
struct PointCloud {
    std::vector<Point3f> points;
};



/**
 * @brief [en] A plane in Hessian form: dot(n, p) = d, with unit normal n.
 *        Points on the food side satisfy dot(n, p) > d.
 * @brief [zh] Hessian 形式的平面：dot(n,p)=d，法向量 n 为单位向量，食材一侧满足 dot(n,p)>d。
 * @exporter
 */
struct Plane {
    float nx = 0.0F;
    float ny = 0.0F;
    float nz = 1.0F;
    float d = 0.0F;
};



/**
 * @brief [en] Axis-aligned region of interest bounds.
 * @brief [zh] 轴对齐的感兴趣区域边界。
 * @exporter
 */
struct AxisAlignedRoi {
    float min_x = 0.0F;
    float max_x = 0.0F;
    float min_y = 0.0F;
    float max_y = 0.0F;
    float min_z = 0.0F;
    float max_z = 0.0F;
};



/**
 * @brief [en] Outcome status of a volume measurement.
 * @brief [zh] 体积测量的结果状态。
 * @exporter
 */
enum class MeasurementStatus : std::uint8_t {
    kSuccess = 0,
    kEmptyInput = 1,
    kNonFiniteInput = 2,
    kInvalidConfig = 3,
    kEmptyBaseline = 4,
    kNonFiniteBaseline = 5,
    kPlaneNotFound = 6,
    kPlaneLowQuality = 7,
    kBaselineNoCells = 8,
    kFoodNotFound = 9,
    kInsufficientCoverage = 10,
    kUnsupportedPlatform = 11,
};



/**
 * @brief [en] How foreground food components are selected.
 * @brief [zh] 前景食材块的选择方式。
 * @exporter
 */
enum class ComponentSelectionMode : std::uint8_t {
    kAllEligible = 0,
    kManual = 1,
};



/**
 * @brief [en] Configuration for the IM volume pipeline.
 * @brief [zh] IM 体积流水线的配置。
 * @exporter
 */
struct MeasurementConfig {
    LengthUnit input_unit = LengthUnit::kMeter;

    bool use_roi = false;
    AxisAlignedRoi roi;

    double voxel_size_m = volume_defaults::kVoxelSizeM;

    // Baseline plane fitting.
    double plane_distance_threshold_m = volume_defaults::kPlaneDistanceM;
    int plane_ransac_iterations = volume_defaults::kPlaneRansacIterations;
    double baseline_max_surface_height_m = volume_defaults::kBaselineMaxSurfaceHeightM;

    // Optional second dominant background plane removal.
    bool remove_secondary_plane = false;
    double secondary_plane_distance_threshold_m = volume_defaults::kPlaneDistanceM;

    // Food-frame background clustering (3D) and footprint clustering (2D).
    double cluster_eps_m = volume_defaults::kClusterEpsM;
    double foreground_cluster_eps_m = volume_defaults::kForegroundClusterEpsM;
    int cluster_min_points = volume_defaults::kClusterMinPoints;

    // Foreground component selection.
    ComponentSelectionMode selection_mode = ComponentSelectionMode::kAllEligible;
    std::vector<int> selected_labels;

    // Integration.
    double integration_resolution_m = volume_defaults::kIntegrationResolutionM;
    double roi_border_margin_m = volume_defaults::kRoiBorderMarginM;
    double min_height_m = volume_defaults::kMinHeightM;
    double max_height_m = -1.0; // negative disables the cap

    // Baseline lookup.
    int baseline_fill_radius_cells = volume_defaults::kBaselineFillRadiusCells;

    // Small-hole interpolation.
    int hole_fill_max_cells = volume_defaults::kHoleFillMaxCells;
    int hole_fill_neighbor_radius_cells = volume_defaults::kHoleFillNeighborRadiusCells;
    double hole_fill_max_neighbor_height_delta_m = volume_defaults::kHoleFillMaxNeighborDeltaM;

    // Quadratic-surface hole completion.
    double curve_fill_max_hole_area_cm2 = volume_defaults::kCurveFillMaxHoleAreaCm2;
    double curve_fill_max_component_area_ratio = volume_defaults::kCurveFillMaxComponentAreaRatio;
    double curve_fill_max_imputed_ratio = volume_defaults::kCurveFillMaxImputedRatio;
    int curve_fill_rim_radius_cells = volume_defaults::kCurveFillRimRadiusCells;
    int curve_fill_min_rim_samples = volume_defaults::kCurveFillMinRimSamples;
    double curve_fill_min_rim_coverage = volume_defaults::kCurveFillMinRimCoverage;
    double curve_fill_max_fit_rmse_m = volume_defaults::kCurveFillMaxFitRmseM;
    double curve_fill_max_prediction_rise_m = volume_defaults::kCurveFillMaxPredictionRiseM;
};



/**
 * @brief [en] Per-component IM integration result.
 * @brief [zh] 单个连通块 IM 积分结果。
 * @exporter
 */
struct ComponentVolumeEstimate {
    double raw_volume_cm3 = 0.0;
    double interpolated_volume_cm3 = 0.0;
    double volume_cm3 = 0.0;

    std::size_t top_surface_points = 0;
    std::size_t measured_cells = 0;
    std::size_t interpolated_cells = 0;
    std::size_t occupied_cells = 0;
    std::size_t bbox_cell_count = 0;
    std::size_t missing_baseline_cells = 0;
    std::size_t unfilled_hole_cells = 0;

    double footprint_area_m2 = 0.0;
    double coverage_ratio = 0.0;
    double mean_height_m = 0.0;
    double max_height_m = 0.0;
};



/**
 * @brief [en] Result of a IM volume measurement. Numeric fields are NaN unless status is kSuccess.
 * @brief [zh] IM 体积测量结果。除非状态为 kSuccess，否则数值字段为 NaN。
 * @exporter
 */
struct VolumeEstimate {
    MeasurementStatus status = MeasurementStatus::kSuccess;
    double volume_cm3 = std::numeric_limits<double>::quiet_NaN();
    double raw_volume_cm3 = std::numeric_limits<double>::quiet_NaN();
    double interpolated_volume_cm3 = std::numeric_limits<double>::quiet_NaN();
    double uncertainty_cm3 = std::numeric_limits<double>::quiet_NaN();

    std::size_t input_points = 0;
    std::size_t downsampled_points = 0;
    std::size_t cluster_count = 0;
    std::vector<int> selected_cluster_labels;
    std::size_t selected_cluster_points = 0;

    /// [en] Per-component integration results, aligned with `selected_cluster_labels`.
    /// [zh] 逐连通块积分结果，与 `selected_cluster_labels` 一一对应。
    std::vector<ComponentVolumeEstimate> component_estimates;

    std::size_t baseline_frames = 0;
    std::size_t baseline_cell_count = 0;
    std::size_t component_count = 0;

    std::size_t top_surface_points = 0;
    std::size_t measured_cells = 0;
    std::size_t interpolated_cells = 0;
    std::size_t occupied_cells = 0;
    std::size_t bbox_cell_count = 0;
    std::size_t missing_baseline_cells = 0;
    std::size_t unfilled_hole_cells = 0;

    double footprint_area_m2 = std::numeric_limits<double>::quiet_NaN();
    double coverage_ratio = std::numeric_limits<double>::quiet_NaN();
    double mean_height_m = std::numeric_limits<double>::quiet_NaN();
    double max_height_m = std::numeric_limits<double>::quiet_NaN();

    double aabb_volume_m3 = std::numeric_limits<double>::quiet_NaN();
    double obb_volume_m3 = std::numeric_limits<double>::quiet_NaN();
    double convex_hull_volume_m3 = std::numeric_limits<double>::quiet_NaN();

    std::string message;
};



/**
 * @brief [en] Converts a length unit to the meters scale factor.
 * @brief [zh] 将长度单位转换为米的比例因子。
 * @param unit [en] The input length unit.
 * @param unit [zh] 输入长度单位。
 * @return [en] The factor that converts a coordinate in `unit` to meters.
 * @return [zh] 把 `unit` 坐标换算成米的因子。
 * @exporter
 */
double length_unit_to_meter_scale(LengthUnit unit);



/**
 * @brief [en] Returns a human-readable description of a measurement status.
 * @brief [zh] 返回测量状态的可读描述。
 * @param status [en] The measurement status.
 * @param status [zh] 测量状态。
 * @return [en] A static description string.
 * @return [zh] 静态描述字符串。
 * @exporter
 */
const char* status_to_string(MeasurementStatus status);



/**
 * @brief [en] Checks whether a point has finite coordinates.
 * @brief [zh] 检查点坐标是否有限。
 * @param p [en] The point to test.
 * @param p [zh] 待检查的点。
 * @return [en] True when all coordinates are finite.
 * @return [zh] 所有坐标都有限时为真。
 * @exporter
 */
bool is_finite(const Point3f& p);



} // namespace vm
