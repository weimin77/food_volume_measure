// Conan::ImportStart
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "volume_baseline.hpp"
#include "volume_component.hpp"
#include "volume_grid.hpp"
#include "volume_integrator.hpp"
#include "volume_log.hpp"
#include "volume_measurement.hpp"
#include "volume_pointcloudprocess.hpp"
#include "volume_types.hpp"
// Conan::ImportEnd



namespace py = pybind11;



namespace {



py::dict height_map_to_dict(const std::map<vm::CellKey, double>& cells) {
    py::dict out;
    for (const auto& entry : cells) {
        out[py::make_tuple(entry.first.first, entry.first.second)] = entry.second;
    }
    return out;
}



std::map<vm::CellKey, double> height_map_from_dict(const py::dict& dict_in) {
    std::map<vm::CellKey, double> out;
    for (const auto& item : dict_in) {
        const py::tuple key = py::cast<py::tuple>(item.first);
        out[{py::cast<std::int64_t>(key[0]), py::cast<std::int64_t>(key[1])}] = py::cast<double>(item.second);
    }
    return out;
}



vm::VolumeEstimate measure_from_pcd(const std::vector<std::string>& baseline_paths, const std::string& food_path,
                                    const vm::MeasurementConfig& cfg) {
    std::vector<vm::PointCloud> baseline_frames;
    baseline_frames.reserve(baseline_paths.size());
    for (const auto& path : baseline_paths) {
        baseline_frames.push_back(vm::load_pcd(path));
    }
    const vm::PointCloud food = vm::load_pcd(food_path);
    vm::VolumeMeasurement measurement;
    measurement.set_config(cfg);
    return measurement.measure(baseline_frames, food);
}



void bind_enums(py::module& m) {
    py::enum_<vm::LengthUnit>(m, "LengthUnit")
        .value("kMeter", vm::LengthUnit::kMeter)
        .value("kMillimeter", vm::LengthUnit::kMillimeter)
        .export_values();

    py::enum_<vm::MeasurementStatus>(m, "MeasurementStatus")
        .value("kSuccess", vm::MeasurementStatus::kSuccess)
        .value("kEmptyInput", vm::MeasurementStatus::kEmptyInput)
        .value("kNonFiniteInput", vm::MeasurementStatus::kNonFiniteInput)
        .value("kInvalidConfig", vm::MeasurementStatus::kInvalidConfig)
        .value("kEmptyBaseline", vm::MeasurementStatus::kEmptyBaseline)
        .value("kNonFiniteBaseline", vm::MeasurementStatus::kNonFiniteBaseline)
        .value("kPlaneNotFound", vm::MeasurementStatus::kPlaneNotFound)
        .value("kPlaneLowQuality", vm::MeasurementStatus::kPlaneLowQuality)
        .value("kBaselineNoCells", vm::MeasurementStatus::kBaselineNoCells)
        .value("kFoodNotFound", vm::MeasurementStatus::kFoodNotFound)
        .value("kInsufficientCoverage", vm::MeasurementStatus::kInsufficientCoverage)
        .value("kUnsupportedPlatform", vm::MeasurementStatus::kUnsupportedPlatform)
        .export_values();

    py::enum_<vm::ComponentSelectionMode>(m, "ComponentSelectionMode")
        .value("kAllEligible", vm::ComponentSelectionMode::kAllEligible)
        .value("kManual", vm::ComponentSelectionMode::kManual)
        .export_values();

    py::enum_<vm::LogLevel>(m, "LogLevel")
        .value("kTrace", vm::LogLevel::kTrace)
        .value("kDebug", vm::LogLevel::kDebug)
        .value("kInfo", vm::LogLevel::kInfo)
        .value("kWarning", vm::LogLevel::kWarning)
        .value("kError", vm::LogLevel::kError)
        .value("kOff", vm::LogLevel::kOff)
        .export_values();

    py::enum_<vm::LogFileMode>(m, "LogFileMode")
        .value("kAppend", vm::LogFileMode::kAppend)
        .value("kTruncate", vm::LogFileMode::kTruncate)
        .export_values();
}



void bind_structures(py::module& m) {
    py::class_<vm::Point3f>(m, "Point3f")
        .def(py::init<>())
        .def(py::init<float, float, float>(), py::arg("x"), py::arg("y"), py::arg("z"))
        .def_readwrite("x", &vm::Point3f::x)
        .def_readwrite("y", &vm::Point3f::y)
        .def_readwrite("z", &vm::Point3f::z)
        .def("__repr__", [](const vm::Point3f& p) {
            return "Point3f(" + std::to_string(p.x) + ", " + std::to_string(p.y) + ", " + std::to_string(p.z) + ")";
        });

    py::class_<vm::PointCloud>(m, "PointCloud")
        .def(py::init<>())
        .def(py::init([](const std::vector<vm::Point3f>& points) {
                 vm::PointCloud cloud;
                 cloud.points = points;
                 return cloud;
             }),
             py::arg("points"))
        .def_readwrite("points", &vm::PointCloud::points)
        .def(
            "append", [](vm::PointCloud& cloud, const vm::Point3f& point) { cloud.points.push_back(point); },
            py::arg("point"))
        .def("__len__", [](const vm::PointCloud& cloud) { return cloud.points.size(); });

    py::class_<vm::Plane>(m, "Plane")
        .def(py::init<>())
        .def(py::init<float, float, float, float>(), py::arg("nx"), py::arg("ny"), py::arg("nz"), py::arg("d"))
        .def_readwrite("nx", &vm::Plane::nx)
        .def_readwrite("ny", &vm::Plane::ny)
        .def_readwrite("nz", &vm::Plane::nz)
        .def_readwrite("d", &vm::Plane::d);

    py::class_<vm::AxisAlignedRoi>(m, "AxisAlignedRoi")
        .def(py::init<>())
        .def_readwrite("min_x", &vm::AxisAlignedRoi::min_x)
        .def_readwrite("max_x", &vm::AxisAlignedRoi::max_x)
        .def_readwrite("min_y", &vm::AxisAlignedRoi::min_y)
        .def_readwrite("max_y", &vm::AxisAlignedRoi::max_y)
        .def_readwrite("min_z", &vm::AxisAlignedRoi::min_z)
        .def_readwrite("max_z", &vm::AxisAlignedRoi::max_z);

    py::class_<vm::MeasurementConfig>(m, "MeasurementConfig")
        .def(py::init<>())
        .def_readwrite("input_unit", &vm::MeasurementConfig::input_unit)
        .def_readwrite("use_roi", &vm::MeasurementConfig::use_roi)
        .def_readwrite("roi", &vm::MeasurementConfig::roi)
        .def_readwrite("voxel_size_m", &vm::MeasurementConfig::voxel_size_m)
        .def_readwrite("plane_distance_threshold_m", &vm::MeasurementConfig::plane_distance_threshold_m)
        .def_readwrite("plane_ransac_iterations", &vm::MeasurementConfig::plane_ransac_iterations)
        .def_readwrite("baseline_max_surface_height_m", &vm::MeasurementConfig::baseline_max_surface_height_m)
        .def_readwrite("remove_secondary_plane", &vm::MeasurementConfig::remove_secondary_plane)
        .def_readwrite("secondary_plane_distance_threshold_m",
                       &vm::MeasurementConfig::secondary_plane_distance_threshold_m)
        .def_readwrite("cluster_eps_m", &vm::MeasurementConfig::cluster_eps_m)
        .def_readwrite("foreground_cluster_eps_m", &vm::MeasurementConfig::foreground_cluster_eps_m)
        .def_readwrite("cluster_min_points", &vm::MeasurementConfig::cluster_min_points)
        .def_readwrite("selection_mode", &vm::MeasurementConfig::selection_mode)
        .def_readwrite("selected_labels", &vm::MeasurementConfig::selected_labels)
        .def_readwrite("integration_resolution_m", &vm::MeasurementConfig::integration_resolution_m)
        .def_readwrite("roi_border_margin_m", &vm::MeasurementConfig::roi_border_margin_m)
        .def_readwrite("min_height_m", &vm::MeasurementConfig::min_height_m)
        .def_readwrite("max_height_m", &vm::MeasurementConfig::max_height_m)
        .def_readwrite("baseline_fill_radius_cells", &vm::MeasurementConfig::baseline_fill_radius_cells)
        .def_readwrite("hole_fill_max_cells", &vm::MeasurementConfig::hole_fill_max_cells)
        .def_readwrite("hole_fill_neighbor_radius_cells", &vm::MeasurementConfig::hole_fill_neighbor_radius_cells)
        .def_readwrite("hole_fill_max_neighbor_height_delta_m",
                       &vm::MeasurementConfig::hole_fill_max_neighbor_height_delta_m)
        .def_readwrite("curve_fill_max_hole_area_cm2", &vm::MeasurementConfig::curve_fill_max_hole_area_cm2)
        .def_readwrite("curve_fill_max_component_area_ratio",
                       &vm::MeasurementConfig::curve_fill_max_component_area_ratio)
        .def_readwrite("curve_fill_max_imputed_ratio", &vm::MeasurementConfig::curve_fill_max_imputed_ratio)
        .def_readwrite("curve_fill_rim_radius_cells", &vm::MeasurementConfig::curve_fill_rim_radius_cells)
        .def_readwrite("curve_fill_min_rim_samples", &vm::MeasurementConfig::curve_fill_min_rim_samples)
        .def_readwrite("curve_fill_min_rim_coverage", &vm::MeasurementConfig::curve_fill_min_rim_coverage)
        .def_readwrite("curve_fill_max_fit_rmse_m", &vm::MeasurementConfig::curve_fill_max_fit_rmse_m)
        .def_readwrite("curve_fill_max_prediction_rise_m", &vm::MeasurementConfig::curve_fill_max_prediction_rise_m);

    py::class_<vm::VolumeEstimate>(m, "VolumeEstimate")
        .def_readonly("status", &vm::VolumeEstimate::status)
        .def_readonly("volume_cm3", &vm::VolumeEstimate::volume_cm3)
        .def_readonly("raw_volume_cm3", &vm::VolumeEstimate::raw_volume_cm3)
        .def_readonly("interpolated_volume_cm3", &vm::VolumeEstimate::interpolated_volume_cm3)
        .def_readonly("uncertainty_cm3", &vm::VolumeEstimate::uncertainty_cm3)
        .def_readonly("input_points", &vm::VolumeEstimate::input_points)
        .def_readonly("downsampled_points", &vm::VolumeEstimate::downsampled_points)
        .def_readonly("cluster_count", &vm::VolumeEstimate::cluster_count)
        .def_readonly("selected_cluster_labels", &vm::VolumeEstimate::selected_cluster_labels)
        .def_readonly("component_estimates", &vm::VolumeEstimate::component_estimates)
        .def_readonly("selected_cluster_points", &vm::VolumeEstimate::selected_cluster_points)
        .def_readonly("baseline_frames", &vm::VolumeEstimate::baseline_frames)
        .def_readonly("baseline_cell_count", &vm::VolumeEstimate::baseline_cell_count)
        .def_readonly("component_count", &vm::VolumeEstimate::component_count)
        .def_readonly("top_surface_points", &vm::VolumeEstimate::top_surface_points)
        .def_readonly("measured_cells", &vm::VolumeEstimate::measured_cells)
        .def_readonly("interpolated_cells", &vm::VolumeEstimate::interpolated_cells)
        .def_readonly("occupied_cells", &vm::VolumeEstimate::occupied_cells)
        .def_readonly("bbox_cell_count", &vm::VolumeEstimate::bbox_cell_count)
        .def_readonly("missing_baseline_cells", &vm::VolumeEstimate::missing_baseline_cells)
        .def_readonly("unfilled_hole_cells", &vm::VolumeEstimate::unfilled_hole_cells)
        .def_readonly("footprint_area_m2", &vm::VolumeEstimate::footprint_area_m2)
        .def_readonly("coverage_ratio", &vm::VolumeEstimate::coverage_ratio)
        .def_readonly("mean_height_m", &vm::VolumeEstimate::mean_height_m)
        .def_readonly("max_height_m", &vm::VolumeEstimate::max_height_m)
        .def_readonly("aabb_volume_m3", &vm::VolumeEstimate::aabb_volume_m3)
        .def_readonly("obb_volume_m3", &vm::VolumeEstimate::obb_volume_m3)
        .def_readonly("convex_hull_volume_m3", &vm::VolumeEstimate::convex_hull_volume_m3)
        .def_readonly("message", &vm::VolumeEstimate::message)
        .def("__repr__", [](const vm::VolumeEstimate& estimate) {
            return "<VolumeEstimate status=" + std::string(vm::status_to_string(estimate.status)) +
                   " volume_cm3=" + std::to_string(estimate.volume_cm3) + ">";
        });

    py::class_<vm::BaselineModel>(m, "BaselineModel")
        .def(py::init<>())
        .def_readonly("frame_count", &vm::BaselineModel::frame_count)
        .def_readonly("cell_count", &vm::BaselineModel::cell_count)
        .def_property_readonly(
            "data", [](const vm::BaselineModel& model) -> const vm::BaselineData* { return model.data.get(); },
            py::return_value_policy::reference_internal);

    py::class_<vm::FoodComponents>(m, "FoodComponents")
        .def(py::init<>())
        .def_readonly("cluster_count", &vm::FoodComponents::cluster_count)
        .def_readonly("labels", &vm::FoodComponents::labels)
        .def_readonly("clouds", &vm::FoodComponents::clouds);

    py::class_<vm::PreprocessResult>(m, "PreprocessResult")
        .def(py::init<>())
        .def_readonly("cloud", &vm::PreprocessResult::cloud)
        .def_readonly("input_points", &vm::PreprocessResult::input_points)
        .def_readonly("retained_points", &vm::PreprocessResult::retained_points);

    py::class_<vm::ComponentVolumeEstimate>(m, "ComponentVolumeEstimate")
        .def(py::init<>())
        .def_readonly("raw_volume_cm3", &vm::ComponentVolumeEstimate::raw_volume_cm3)
        .def_readonly("interpolated_volume_cm3", &vm::ComponentVolumeEstimate::interpolated_volume_cm3)
        .def_readonly("volume_cm3", &vm::ComponentVolumeEstimate::volume_cm3)
        .def_readonly("top_surface_points", &vm::ComponentVolumeEstimate::top_surface_points)
        .def_readonly("measured_cells", &vm::ComponentVolumeEstimate::measured_cells)
        .def_readonly("interpolated_cells", &vm::ComponentVolumeEstimate::interpolated_cells)
        .def_readonly("occupied_cells", &vm::ComponentVolumeEstimate::occupied_cells)
        .def_readonly("bbox_cell_count", &vm::ComponentVolumeEstimate::bbox_cell_count)
        .def_readonly("missing_baseline_cells", &vm::ComponentVolumeEstimate::missing_baseline_cells)
        .def_readonly("unfilled_hole_cells", &vm::ComponentVolumeEstimate::unfilled_hole_cells)
        .def_readonly("footprint_area_m2", &vm::ComponentVolumeEstimate::footprint_area_m2)
        .def_readonly("coverage_ratio", &vm::ComponentVolumeEstimate::coverage_ratio)
        .def_readonly("mean_height_m", &vm::ComponentVolumeEstimate::mean_height_m)
        .def_readonly("max_height_m", &vm::ComponentVolumeEstimate::max_height_m);

    py::class_<vm::PlaneFrame>(m, "PlaneFrame")
        .def(py::init<>())
        .def_readwrite("ox", &vm::PlaneFrame::ox)
        .def_readwrite("oy", &vm::PlaneFrame::oy)
        .def_readwrite("oz", &vm::PlaneFrame::oz)
        .def_readwrite("ux", &vm::PlaneFrame::ux)
        .def_readwrite("uy", &vm::PlaneFrame::uy)
        .def_readwrite("uz", &vm::PlaneFrame::uz)
        .def_readwrite("vx", &vm::PlaneFrame::vx)
        .def_readwrite("vy", &vm::PlaneFrame::vy)
        .def_readwrite("vz", &vm::PlaneFrame::vz)
        .def_readwrite("nx", &vm::PlaneFrame::nx)
        .def_readwrite("ny", &vm::PlaneFrame::ny)
        .def_readwrite("nz", &vm::PlaneFrame::nz);

    py::class_<vm::PlaneRoi>(m, "PlaneRoi")
        .def(py::init<>())
        .def_readwrite("u_min_m", &vm::PlaneRoi::u_min_m)
        .def_readwrite("u_max_m", &vm::PlaneRoi::u_max_m)
        .def_readwrite("v_min_m", &vm::PlaneRoi::v_min_m)
        .def_readwrite("v_max_m", &vm::PlaneRoi::v_max_m)
        .def_readwrite("border_margin_m", &vm::PlaneRoi::border_margin_m);

    py::class_<vm::BaselineData>(m, "BaselineData")
        .def(py::init<>())
        .def_readwrite("plane", &vm::BaselineData::plane)
        .def_readwrite("frame", &vm::BaselineData::frame)
        .def_readwrite("roi", &vm::BaselineData::roi)
        .def_readwrite("cell_size_m", &vm::BaselineData::cell_size_m)
        .def_property(
            "height_by_cell",
            [](const vm::BaselineData& baseline) { return height_map_to_dict(baseline.height_by_cell); },
            [](vm::BaselineData& baseline, const py::dict& dict_in) {
                baseline.height_by_cell = height_map_from_dict(dict_in);
            })
        .def_readwrite("bbox_u_min_m", &vm::BaselineData::bbox_u_min_m)
        .def_readwrite("bbox_u_max_m", &vm::BaselineData::bbox_u_max_m)
        .def_readwrite("bbox_v_min_m", &vm::BaselineData::bbox_v_min_m)
        .def_readwrite("bbox_v_max_m", &vm::BaselineData::bbox_v_max_m);

    py::class_<vm::HeightGrid>(m, "HeightGrid")
        .def(py::init<>())
        .def_readwrite("cells", &vm::HeightGrid::cells)
        .def_readwrite("baseline_heights_m", &vm::HeightGrid::baseline_heights_m)
        .def_readwrite("heights_m", &vm::HeightGrid::heights_m)
        .def_property(
            "is_interpolated",
            [](const vm::HeightGrid& grid) {
                return std::vector<int>(grid.is_interpolated.begin(), grid.is_interpolated.end());
            },
            [](vm::HeightGrid& grid, const std::vector<int>& values) {
                grid.is_interpolated.assign(values.begin(), values.end());
            })
        .def_readwrite("component_labels", &vm::HeightGrid::component_labels)
        .def_readwrite("cell_size_m", &vm::HeightGrid::cell_size_m)
        .def_readwrite("frame", &vm::HeightGrid::frame)
        .def_readonly("measured_cells", &vm::HeightGrid::measured_cells)
        .def_readonly("interpolated_cells", &vm::HeightGrid::interpolated_cells)
        .def_readonly("occupied_cells", &vm::HeightGrid::occupied_cells)
        .def_readonly("bbox_cells", &vm::HeightGrid::bbox_cells)
        .def_readonly("missing_baseline_cells", &vm::HeightGrid::missing_baseline_cells)
        .def_readonly("raw_volume_m3", &vm::HeightGrid::raw_volume_m3)
        .def_readonly("interpolated_volume_m3", &vm::HeightGrid::interpolated_volume_m3)
        .def_readonly("volume_m3", &vm::HeightGrid::volume_m3)
        .def_readonly("footprint_area_m2", &vm::HeightGrid::footprint_area_m2)
        .def_readonly("coverage_ratio", &vm::HeightGrid::coverage_ratio)
        .def_readonly("mean_height_m", &vm::HeightGrid::mean_height_m)
        .def_readonly("max_height_m", &vm::HeightGrid::max_height_m);

    py::class_<vm::HoleFillStats>(m, "HoleFillStats")
        .def(py::init<>())
        .def_readonly("candidate_hole_count", &vm::HoleFillStats::candidate_hole_count)
        .def_readonly("filled_hole_count", &vm::HoleFillStats::filled_hole_count)
        .def_readonly("filled_cell_count", &vm::HoleFillStats::filled_cell_count)
        .def_readonly("small_filled_hole_count", &vm::HoleFillStats::small_filled_hole_count)
        .def_readonly("curve_filled_hole_count", &vm::HoleFillStats::curve_filled_hole_count)
        .def_readonly("unfilled_hole_cells", &vm::HoleFillStats::unfilled_hole_cells)
        .def_readonly("max_component_inferred_ratio", &vm::HoleFillStats::max_component_inferred_ratio);

    py::class_<vm::SurfaceMap>(m, "SurfaceMap")
        .def(py::init<>())
        .def_readwrite("cells", &vm::SurfaceMap::cells)
        .def_readwrite("heights_m", &vm::SurfaceMap::heights_m)
        .def_readwrite("labels", &vm::SurfaceMap::labels);
}



void bind_functions(py::module& m) {
    m.def("length_unit_to_meter_scale", &vm::length_unit_to_meter_scale, py::arg("unit"));
    m.def("status_to_string", &vm::status_to_string, py::arg("status"));
    m.def("is_finite", &vm::is_finite, py::arg("point"));

    m.def("load_pcd", &vm::load_pcd, py::arg("path"));
    m.def("voxel_downsample", &vm::voxel_downsample, py::arg("cloud"), py::arg("voxel_size"));
    m.def("dbscan_labels", &vm::dbscan_labels, py::arg("points"), py::arg("eps"), py::arg("min_points"));

    m.def(
        "fit_plane_ransac",
        [](const vm::PointCloud& cloud, double distance_threshold_m, int iterations) {
            vm::Plane plane;
            std::vector<std::size_t> inlier_indices;
            const vm::MeasurementStatus status =
                vm::fit_plane_ransac(cloud, distance_threshold_m, iterations, plane, inlier_indices);
            return py::make_tuple(status, plane, inlier_indices);
        },
        py::arg("cloud"), py::arg("distance_threshold_m"), py::arg("iterations"));

    m.def(
        "remove_dominant_plane",
        [](const vm::PointCloud& cloud, const vm::MeasurementConfig& cfg) {
            vm::PointCloud remaining;
            const vm::MeasurementStatus status = vm::remove_dominant_plane(cloud, cfg, remaining);
            return py::make_tuple(status, remaining);
        },
        py::arg("cloud"), py::arg("cfg"));

    m.def(
        "preprocess_cloud",
        [](const vm::PointCloud& input, const vm::MeasurementConfig& cfg) {
            vm::PreprocessResult out;
            const vm::MeasurementStatus status = vm::preprocess_cloud(input, cfg, out);
            return py::make_tuple(status, out);
        },
        py::arg("cloud"), py::arg("cfg"));

    m.def(
        "build_baseline_model",
        [](const std::vector<vm::PointCloud>& baseline_frames, const vm::PointCloud& orientation_points,
           const vm::MeasurementConfig& cfg) {
            vm::BaselineModel out;
            const vm::MeasurementStatus status =
                vm::build_baseline_model(baseline_frames, orientation_points, cfg, out);
            return py::make_tuple(status, out);
        },
        py::arg("baseline_frames"), py::arg("orientation_points"), py::arg("cfg"));

    m.def(
        "extract_food_components",
        [](const vm::PointCloud& food_m, const vm::BaselineModel& baseline, const vm::MeasurementConfig& cfg) {
            vm::FoodComponents out;
            const vm::MeasurementStatus status = vm::extract_food_components(food_m, baseline, cfg, out);
            return py::make_tuple(status, out);
        },
        py::arg("food_m"), py::arg("baseline"), py::arg("cfg"));

    m.def(
        "measure_component_volume",
        [](const vm::FoodComponents& components, const vm::BaselineModel& baseline, const vm::MeasurementConfig& cfg) {
            vm::ComponentVolumeEstimate out;
            const vm::MeasurementStatus status = vm::measure_component_volume(components, baseline, cfg, out);
            return py::make_tuple(status, out);
        },
        py::arg("components"), py::arg("baseline"), py::arg("cfg"));

    m.def(
        "measure_component_volumes",
        [](const vm::FoodComponents& components, const vm::BaselineModel& baseline, const vm::MeasurementConfig& cfg) {
            std::vector<vm::ComponentVolumeEstimate> out;
            const vm::MeasurementStatus status = vm::measure_component_volumes(components, baseline, cfg, out);
            return py::make_tuple(status, out);
        },
        py::arg("components"), py::arg("baseline"), py::arg("cfg"));

    m.def("measure_from_pcd", &measure_from_pcd, py::arg("baseline_pcd_paths"), py::arg("food_pcd_path"),
          py::arg("cfg") = vm::MeasurementConfig{});

    // Fine-grained operators (point-cloud preprocessing).
    m.def("scale_to_meters", &vm::scale_to_meters, py::arg("cloud"), py::arg("unit"));
    m.def("crop_axis_aligned", &vm::crop_axis_aligned, py::arg("cloud"), py::arg("roi"));
    m.def("orient_plane", &vm::orient_plane, py::arg("plane"), py::arg("points"));
    m.def(
        "split_plane_inliers",
        [](const vm::PointCloud& cloud, const vm::Plane& plane, double distance_threshold_m) {
            vm::PointCloud remaining;
            std::vector<std::size_t> inlier_indices;
            vm::split_plane_inliers(cloud, plane, distance_threshold_m, remaining, inlier_indices);
            return py::make_tuple(remaining, inlier_indices);
        },
        py::arg("cloud"), py::arg("plane"), py::arg("distance_threshold_m"));

    // Fine-grained operators (baseline).
    m.def("build_plane_frame", &vm::build_plane_frame, py::arg("plane"), py::arg("origin"));
    m.def("build_plane_roi", &vm::build_plane_roi, py::arg("baseline"), py::arg("border_margin_m"));
    m.def(
        "rasterize_baseline",
        [](const std::vector<vm::PointCloud>& baseline_frames, const vm::PlaneFrame& frame, double cell_size_m,
           double max_surface_height_m) {
            vm::BaselineData out;
            const vm::MeasurementStatus status =
                vm::rasterize_baseline(baseline_frames, frame, cell_size_m, max_surface_height_m, out);
            return py::make_tuple(status, out);
        },
        py::arg("baseline_frames"), py::arg("frame"), py::arg("cell_size_m"), py::arg("max_surface_height_m"));

    // Fine-grained operators (foreground extraction).
    m.def(
        "filter_baseline_difference",
        [](const vm::PointCloud& food_m, const vm::BaselineModel& baseline, const vm::MeasurementConfig& cfg) {
            vm::PointCloud dense;
            const vm::MeasurementStatus status = vm::filter_baseline_difference(food_m, baseline, cfg, dense);
            return py::make_tuple(status, dense);
        },
        py::arg("food_m"), py::arg("baseline"), py::arg("cfg"));
    m.def("project_to_plane", &vm::project_to_plane, py::arg("cloud"), py::arg("baseline"));
    m.def(
        "select_components",
        [](const std::vector<int>& labels, const vm::PointCloud& cloud, const vm::MeasurementConfig& cfg) {
            vm::FoodComponents out;
            const vm::MeasurementStatus status = vm::select_components(labels, cloud, cfg, out);
            return py::make_tuple(status, out);
        },
        py::arg("labels"), py::arg("cloud"), py::arg("cfg"));

    // Fine-grained operators (integration + hole completion).
    m.def("build_top_surface", &vm::build_top_surface, py::arg("components"), py::arg("baseline"));
    m.def(
        "build_height_grid",
        [](const vm::SurfaceMap& surface, const vm::BaselineModel& baseline, const vm::MeasurementConfig& cfg) {
            vm::HeightGrid out;
            const vm::MeasurementStatus status = vm::build_height_grid(surface, baseline, cfg, out);
            return py::make_tuple(status, out);
        },
        py::arg("surface"), py::arg("baseline"), py::arg("cfg"));
    m.def(
        "complete_holes",
        [](vm::HeightGrid& grid, const vm::BaselineModel& baseline, const vm::MeasurementConfig& cfg) {
            vm::HoleFillStats stats;
            const vm::MeasurementStatus status = vm::complete_holes(grid, baseline, cfg, stats);
            return py::make_tuple(status, stats);
        },
        py::arg("grid"), py::arg("baseline"), py::arg("cfg"));
    m.def("compute_grid_estimate", &vm::compute_grid_estimate, py::arg("grid"));

    // Fine-grained operators (reference volumes).
    m.def("compute_aabb_volume", &vm::compute_aabb_volume, py::arg("cloud"));
    m.def("compute_obb_volume", &vm::compute_obb_volume, py::arg("cloud"));
    m.def("compute_convex_hull_volume", &vm::compute_convex_hull_volume, py::arg("cloud"));

    py::class_<vm::VolumeMeasurement>(m, "VolumeMeasurement")
        .def(py::init<>())
        .def("set_config", &vm::VolumeMeasurement::set_config, py::arg("cfg"))
        .def("config", &vm::VolumeMeasurement::config, py::return_value_policy::reference_internal)
        .def("save_config_to_json", &vm::VolumeMeasurement::save_config_to_json, py::arg("path"))
        .def("load_config_from_json", &vm::VolumeMeasurement::load_config_from_json, py::arg("path"))
        .def("measure", &vm::VolumeMeasurement::measure, py::arg("baseline_frames"), py::arg("food_frame"));

    // Logging.
    m.def("log_set_level", &vm::log_set_level, py::arg("level"));
    m.def("log_get_level", &vm::log_get_level);
    m.def("log_set_console", &vm::log_set_console, py::arg("enabled"));
    m.def("log_get_console", &vm::log_get_console);
    m.def("log_set_file", &vm::log_set_file, py::arg("path"), py::arg("mode") = vm::LogFileMode::kAppend);
    m.def("log_close_file", &vm::log_close_file);
    m.def("log_write", &vm::log_write, py::arg("level"), py::arg("message"));
    m.def("log_trace", &vm::log_trace, py::arg("message"));
    m.def("log_debug", &vm::log_debug, py::arg("message"));
    m.def("log_info", &vm::log_info, py::arg("message"));
    m.def("log_warning", &vm::log_warning, py::arg("message"));
    m.def("log_error", &vm::log_error, py::arg("message"));
}



} // namespace



PYBIND11_MODULE(food_volume_measure_python, m) {
    m.doc() = "Python bindings for the vm IM food volume measurement library";
    m.attr("__version__") = "0.1.0";

    bind_enums(m);
    bind_structures(m);
    bind_functions(m);
}
