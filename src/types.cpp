// Conan::ImportStart
#include <cmath>
#include "types.hpp"
// Conan::ImportEnd



namespace {

constexpr double kMeterScale = 1.0;
constexpr double kMillimeterToMeter = 1.0e-3;

} // namespace



/**
 * @brief [en] Converts a length unit to the meters scale factor.
 * @brief [zh] 将长度单位转换为米的比例因子。
 * @attacher
 */
double length_unit_to_meter_scale(LengthUnit unit) {
    switch (unit) {
    case LengthUnit::kMeter:
        return kMeterScale;
    case LengthUnit::kMillimeter:
        return kMillimeterToMeter;
    }
    return kMeterScale;
}



/**
 * @brief [en] Returns a human-readable description of a measurement status.
 * @brief [zh] 返回测量状态的可读描述。
 * @attacher
 */
const char* status_to_string(MeasurementStatus status) {
    switch (status) {
    case MeasurementStatus::kSuccess:
        return "success";
    case MeasurementStatus::kEmptyInput:
        return "empty input";
    case MeasurementStatus::kNonFiniteInput:
        return "non-finite input";
    case MeasurementStatus::kInvalidConfig:
        return "invalid configuration";
    case MeasurementStatus::kEmptyBaseline:
        return "empty baseline";
    case MeasurementStatus::kNonFiniteBaseline:
        return "non-finite baseline";
    case MeasurementStatus::kPlaneNotFound:
        return "baseline plane not found";
    case MeasurementStatus::kPlaneLowQuality:
        return "baseline plane low quality";
    case MeasurementStatus::kBaselineNoCells:
        return "no valid baseline cells";
    case MeasurementStatus::kFoodNotFound:
        return "food component not found";
    case MeasurementStatus::kInsufficientCoverage:
        return "insufficient grid coverage";
    case MeasurementStatus::kUnsupportedPlatform:
        return "unsupported platform";
    }
    return "unknown";
}



/**
 * @brief [en] Checks whether a point has finite coordinates.
 * @brief [zh] 检查点坐标是否有限。
 * @attacher
 */
bool is_finite(const Point3f& p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); }
