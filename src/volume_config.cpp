// Conan::ImportStart
#include <fstream>
#include <string>
#include <nlohmann/json.hpp>
#include "volume_config.hpp"
#include "volume_log.hpp"
// Conan::ImportEnd



namespace vm {



namespace {

using Json = nlohmann::json;

bool parse_length_unit(const std::string& text, LengthUnit& out) {
    if (text == "meter") {
        out = LengthUnit::kMeter;
        return true;
    }
    if (text == "millimeter") {
        out = LengthUnit::kMillimeter;
        return true;
    }
    return false;
}

bool parse_selection_mode(const std::string& text, ComponentSelectionMode& out) {
    if (text == "all_eligible") {
        out = ComponentSelectionMode::kAllEligible;
        return true;
    }
    if (text == "manual") {
        out = ComponentSelectionMode::kManual;
        return true;
    }
    return false;
}

// Reads a scalar member only when the key is present; leaves the target untouched otherwise.
// A type-mismatched field is ignored (the target keeps its previous value).
template <typename T>
void read_if_present(const Json& j, const char* key, T& target) {
    if (!j.contains(key)) {
        return;
    }
    try {
        j.at(key).get_to(target);
    } catch (const Json::exception&) {
        // 类型不匹配：忽略该字段，保留原值。
    }
}

} // namespace



bool load_config_from_json(const std::string& path, MeasurementConfig& out) {
    std::ifstream file(path);
    if (!file.is_open()) {
        log_error("load_config_from_json: cannot open file " + path);
        return false;
    }

    Json root;
    try {
        file >> root;
    } catch (const Json::parse_error& err) {
        log_error("load_config_from_json: parse error " + std::string(err.what()));
        return false;
    }
    if (!root.is_object()) {
        log_error("load_config_from_json: root is not a JSON object");
        return false;
    }

    // Enum fields (string -> enum).
    if (root.contains("input_unit")) {
        const Json& unit_json = root.at("input_unit");
        if (!unit_json.is_string()) {
            log_error("load_config_from_json: input_unit must be a string");
            return false;
        }
        LengthUnit unit{};
        if (!parse_length_unit(unit_json.get<std::string>(), unit)) {
            log_error("load_config_from_json: invalid input_unit");
            return false;
        }
        out.input_unit = unit;
    }
    if (root.contains("selection_mode")) {
        const Json& mode_json = root.at("selection_mode");
        if (!mode_json.is_string()) {
            log_error("load_config_from_json: selection_mode must be a string");
            return false;
        }
        ComponentSelectionMode mode{};
        if (!parse_selection_mode(mode_json.get<std::string>(), mode)) {
            log_error("load_config_from_json: invalid selection_mode");
            return false;
        }
        out.selection_mode = mode;
    }

    // Nested ROI object.
    if (root.contains("roi")) {
        const Json& roi = root.at("roi");
        if (!roi.is_object()) {
            log_error("load_config_from_json: roi is not an object");
            return false;
        }
        read_if_present(roi, "min_x", out.roi.min_x);
        read_if_present(roi, "max_x", out.roi.max_x);
        read_if_present(roi, "min_y", out.roi.min_y);
        read_if_present(roi, "max_y", out.roi.max_y);
        read_if_present(roi, "min_z", out.roi.min_z);
        read_if_present(roi, "max_z", out.roi.max_z);
    }

    // Scalar fields.
    read_if_present(root, "use_roi", out.use_roi);
    read_if_present(root, "voxel_size_m", out.voxel_size_m);
    read_if_present(root, "plane_distance_threshold_m", out.plane_distance_threshold_m);
    read_if_present(root, "plane_ransac_iterations", out.plane_ransac_iterations);
    read_if_present(root, "baseline_max_surface_height_m", out.baseline_max_surface_height_m);
    read_if_present(root, "remove_secondary_plane", out.remove_secondary_plane);
    read_if_present(root, "secondary_plane_distance_threshold_m", out.secondary_plane_distance_threshold_m);
    read_if_present(root, "cluster_eps_m", out.cluster_eps_m);
    read_if_present(root, "foreground_cluster_eps_m", out.foreground_cluster_eps_m);
    read_if_present(root, "cluster_min_points", out.cluster_min_points);
    read_if_present(root, "integration_resolution_m", out.integration_resolution_m);
    read_if_present(root, "roi_border_margin_m", out.roi_border_margin_m);
    read_if_present(root, "min_height_m", out.min_height_m);
    read_if_present(root, "max_height_m", out.max_height_m);
    read_if_present(root, "baseline_fill_radius_cells", out.baseline_fill_radius_cells);
    read_if_present(root, "hole_fill_max_cells", out.hole_fill_max_cells);
    read_if_present(root, "hole_fill_neighbor_radius_cells", out.hole_fill_neighbor_radius_cells);
    read_if_present(root, "hole_fill_max_neighbor_height_delta_m", out.hole_fill_max_neighbor_height_delta_m);
    read_if_present(root, "curve_fill_max_hole_area_cm2", out.curve_fill_max_hole_area_cm2);
    read_if_present(root, "curve_fill_max_component_area_ratio", out.curve_fill_max_component_area_ratio);
    read_if_present(root, "curve_fill_max_imputed_ratio", out.curve_fill_max_imputed_ratio);
    read_if_present(root, "curve_fill_rim_radius_cells", out.curve_fill_rim_radius_cells);
    read_if_present(root, "curve_fill_min_rim_samples", out.curve_fill_min_rim_samples);
    read_if_present(root, "curve_fill_min_rim_coverage", out.curve_fill_min_rim_coverage);
    read_if_present(root, "curve_fill_max_fit_rmse_m", out.curve_fill_max_fit_rmse_m);
    read_if_present(root, "curve_fill_max_prediction_rise_m", out.curve_fill_max_prediction_rise_m);

    if (root.contains("selected_labels")) {
        if (!root.at("selected_labels").is_array()) {
            log_error("load_config_from_json: selected_labels must be an array");
            return false;
        }
        root.at("selected_labels").get_to(out.selected_labels);
    }

    return true;
}



} // namespace vm
