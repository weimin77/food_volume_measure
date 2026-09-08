// Conan::ImportStart
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "volume_pointcloudprocess.hpp"
#include "volume_log.hpp"
#ifndef __ARM_EABI__
#include <Eigen/Dense>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#endif
// Conan::ImportEnd



namespace vm {



namespace {

double signed_height(const Plane& plane, const Point3f& p) {
    return static_cast<double>(plane.nx) * p.x + static_cast<double>(plane.ny) * p.y +
           static_cast<double>(plane.nz) * p.z - static_cast<double>(plane.d);
}

} // namespace



/**
 * @brief [en] Loads a PCD point-cloud file into the library's PCL-free point model.
 * @brief [zh] 将 PCD 点云文件载入到库的与 PCL 无关的点模型。
 * @attacher
 */
PointCloud load_pcd(const std::string& path) {
#ifdef __ARM_EABI__
    (void)path;
    return PointCloud{};
#else
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) < 0) {
        log_error("load_pcd failed: " + path);
        return PointCloud{};
    }

    PointCloud out;
    out.points.reserve(cloud->size());
    for (const auto& p : cloud->points) {
        out.points.push_back(Point3f{p.x, p.y, p.z});
    }
    log_info("load_pcd: " + path + " points=" + std::to_string(out.points.size()));
    return out;
#endif // __ARM_EABI__
}



PointCloud voxel_downsample(const PointCloud& cloud, double voxel_size) {
#ifdef __ARM_EABI__
    (void)cloud;
    (void)voxel_size;
    return PointCloud{};
#else
    PointCloud out;
    if (cloud.points.empty() || !(voxel_size > 0.0)) {
        return out;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>);
    input->points.reserve(cloud.points.size());
    for (const auto& p : cloud.points) {
        input->points.push_back(pcl::PointXYZ{p.x, p.y, p.z});
    }
    input->width = static_cast<std::uint32_t>(input->points.size());
    input->height = 1;

    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(input);
    voxel_grid.setLeafSize(static_cast<float>(voxel_size), static_cast<float>(voxel_size),
                           static_cast<float>(voxel_size));

    pcl::PointCloud<pcl::PointXYZ> filtered;
    voxel_grid.filter(filtered);

    out.points.reserve(filtered.points.size());
    for (const auto& p : filtered.points) {
        out.points.push_back(Point3f{p.x, p.y, p.z});
    }
    log_debug("voxel_downsample: " + std::to_string(cloud.points.size()) + " -> " + std::to_string(out.points.size()));
    return out;
#endif // __ARM_EABI__
}



PointCloud scale_to_meters(const PointCloud& cloud, LengthUnit unit) {
    const double scale = length_unit_to_meter_scale(unit);
    PointCloud out;
    out.points.reserve(cloud.points.size());
    for (const auto& p : cloud.points) {
        if (!is_finite(p)) {
            continue;
        }
        out.points.push_back(Point3f{static_cast<float>(static_cast<double>(p.x) * scale),
                                     static_cast<float>(static_cast<double>(p.y) * scale),
                                     static_cast<float>(static_cast<double>(p.z) * scale)});
    }
    return out;
}



PointCloud crop_axis_aligned(const PointCloud& cloud, const AxisAlignedRoi& roi) {
#ifdef __ARM_EABI__
    (void)cloud;
    (void)roi;
    return PointCloud{};
#else
    PointCloud out;
    if (cloud.points.empty()) {
        return out;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>);
    input->points.reserve(cloud.points.size());
    for (const auto& p : cloud.points) {
        input->points.push_back(pcl::PointXYZ{p.x, p.y, p.z});
    }
    input->width = static_cast<std::uint32_t>(input->points.size());
    input->height = 1;

    pcl::CropBox<pcl::PointXYZ> crop_box;
    crop_box.setInputCloud(input);
    crop_box.setMin(Eigen::Vector4f(roi.min_x, roi.min_y, roi.min_z, 1.0f));
    crop_box.setMax(Eigen::Vector4f(roi.max_x, roi.max_y, roi.max_z, 1.0f));

    pcl::PointCloud<pcl::PointXYZ> filtered;
    crop_box.filter(filtered);

    out.points.reserve(filtered.points.size());
    for (const auto& p : filtered.points) {
        out.points.push_back(Point3f{p.x, p.y, p.z});
    }
    return out;
#endif // __ARM_EABI__
}



void split_plane_inliers(const PointCloud& cloud, const Plane& plane, double distance_threshold_m,
                         PointCloud& remaining, std::vector<std::size_t>& inlier_indices) {
#ifdef __ARM_EABI__
    (void)cloud;
    (void)plane;
    (void)distance_threshold_m;
    (void)remaining;
    (void)inlier_indices;
#else
    remaining = PointCloud{};
    inlier_indices.clear();
    if (cloud.points.empty()) {
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>);
    input->points.reserve(cloud.points.size());
    for (const auto& p : cloud.points) {
        input->points.push_back(pcl::PointXYZ{p.x, p.y, p.z});
    }
    input->width = static_cast<std::uint32_t>(input->points.size());
    input->height = 1;

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    inliers->indices.reserve(cloud.points.size());
    for (std::size_t i = 0; i < cloud.points.size(); ++i) {
        if (std::fabs(signed_height(plane, cloud.points[i])) < distance_threshold_m) {
            inliers->indices.push_back(static_cast<int>(i));
        }
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input);
    extract.setIndices(inliers);
    extract.setNegative(true); // keep the outliers as `remaining`

    pcl::PointCloud<pcl::PointXYZ> filtered;
    extract.filter(filtered);

    remaining.points.reserve(filtered.points.size());
    for (const auto& p : filtered.points) {
        remaining.points.push_back(Point3f{p.x, p.y, p.z});
    }
    inlier_indices.assign(inliers->indices.begin(), inliers->indices.end());
#endif // __ARM_EABI__
}



Plane orient_plane(const Plane& plane, const PointCloud& points) {
    std::vector<double> heights;
    heights.reserve(points.points.size());
    for (const auto& p : points.points) {
        if (!is_finite(p)) {
            continue;
        }
        heights.push_back(signed_height(plane, p));
    }
    if (heights.empty()) {
        return plane;
    }
    std::sort(heights.begin(), heights.end());
    const std::size_t n = heights.size();
    const double med = (n % 2 == 1) ? heights[n / 2] : 0.5 * (heights[n / 2 - 1] + heights[n / 2]);
    if (med < 0.0) {
        return Plane{-plane.nx, -plane.ny, -plane.nz, -plane.d};
    }
    return plane;
}



/**
 * @brief [en] Validates, unit-normalizes, optionally crops, and voxel-downsamples the input cloud.
 * @brief [zh] 校验、单位归一、可选裁剪并体素降采样输入点云。
 * @attacher
 */
MeasurementStatus preprocess_cloud(const PointCloud& input, const MeasurementConfig& cfg, PreprocessResult& out) {
#ifdef __ARM_EABI__
    (void)input;
    (void)cfg;
    (void)out;
    return MeasurementStatus::kUnsupportedPlatform;
#else
    out = PreprocessResult{};

    if (input.points.empty()) {
        return MeasurementStatus::kEmptyInput;
    }
    // Depth sensors encode invalid returns as NaN/Inf. Those points are dropped one by one, as the
    // reference implementation masks them, instead of aborting the whole measurement. Rejecting is
    // reserved for input that has no usable point at all.
    if (std::none_of(input.points.begin(), input.points.end(), [](const Point3f& p) { return is_finite(p); })) {
        return MeasurementStatus::kNonFiniteInput;
    }

    const double scale = length_unit_to_meter_scale(cfg.input_unit);
    if (!(scale > 0.0) || !(cfg.voxel_size_m > 0.0F)) {
        return MeasurementStatus::kInvalidConfig;
    }
    if (cfg.use_roi &&
        !(cfg.roi.min_x < cfg.roi.max_x && cfg.roi.min_y < cfg.roi.max_y && cfg.roi.min_z < cfg.roi.max_z)) {
        return MeasurementStatus::kInvalidConfig;
    }

    // Unit-normalize to metres, dropping non-finite points on the way.
    const PointCloud scaled = scale_to_meters(input, cfg.input_unit);

    // Optional axis-aligned ROI crop.
    const PointCloud cropped = cfg.use_roi ? crop_axis_aligned(scaled, cfg.roi) : scaled;

    // Voxel downsample (PCL VoxelGrid centroid rule).
    const PointCloud voxeled = voxel_downsample(cropped, cfg.voxel_size_m);

    out.cloud = voxeled;
    out.input_points = input.points.size();
    out.retained_points = out.cloud.points.size();

    if (out.cloud.points.empty()) {
        return MeasurementStatus::kEmptyInput;
    }
    log_info("preprocess_cloud: input=" + std::to_string(out.input_points) +
             " downsampled=" + std::to_string(out.retained_points));
    return MeasurementStatus::kSuccess;
#endif // __ARM_EABI__
}



/**
 * @brief [en] Density-based spatial clustering (DBSCAN) matching Open3D `cluster_dbscan`.
 * @brief [zh] 与 Open3D `cluster_dbscan` 语义一致的密度聚类（DBSCAN）。
 * @attacher
 */
std::vector<int> dbscan_labels(const std::vector<Point3f>& points, double eps, int min_points) {
#ifdef __ARM_EABI__
    (void)points;
    (void)eps;
    (void)min_points;
    return std::vector<int>(points.size(), -1);
#else
    const std::size_t point_count = points.size();
    if (point_count == 0) {
        return {};
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->reserve(point_count);
    for (const auto& p : points) {
        cloud->push_back(pcl::PointXYZ(p.x, p.y, p.z));
    }

    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    tree->setInputCloud(cloud);

    // Precompute the eps-neighborhood (including the query point itself) for every point.
    std::vector<std::vector<int>> neighborhoods(point_count);
    for (std::size_t i = 0; i < point_count; ++i) {
        std::vector<int> indices;
        std::vector<float> squared_distances;
        tree->radiusSearch((*cloud)[i], eps, indices, squared_distances);
        neighborhoods[i] = std::move(indices);
    }

    std::vector<int> labels(point_count, -2); // -2 unprocessed, -1 noise
    int cluster_label = 0;
    for (std::size_t i = 0; i < point_count; ++i) {
        if (labels[i] != -2) {
            continue;
        }
        if (neighborhoods[i].size() < static_cast<std::size_t>(min_points)) {
            labels[i] = -1; // not a core point
            continue;
        }

        labels[i] = cluster_label;
        std::vector<int> queue = neighborhoods[i];
        std::size_t head = 0;
        while (head < queue.size()) {
            const int neighbor = queue[head++];
            if (labels[static_cast<std::size_t>(neighbor)] == -1) {
                labels[static_cast<std::size_t>(neighbor)] = cluster_label; // border point
            }
            if (labels[static_cast<std::size_t>(neighbor)] != -2) {
                continue;
            }
            labels[static_cast<std::size_t>(neighbor)] = cluster_label;
            if (neighborhoods[static_cast<std::size_t>(neighbor)].size() >= static_cast<std::size_t>(min_points)) {
                const auto& next = neighborhoods[static_cast<std::size_t>(neighbor)];
                queue.insert(queue.end(), next.begin(), next.end());
            }
        }
        ++cluster_label;
    }
    int cluster_count = 0;
    for (const int label : labels) {
        if (label >= 0) {
            cluster_count = std::max(cluster_count, label + 1);
        }
    }
    log_debug("dbscan_labels: points=" + std::to_string(point_count) + " clusters=" + std::to_string(cluster_count));
    return labels;
#endif // __ARM_EABI__
}



/**
 * @brief [en] Fits a dominant plane from a point cloud with PCL RANSAC (SACMODEL_PLANE).
 * @brief [zh] 用 PCL RANSAC（SACMODEL_PLANE）从点云拟合主平面。
 * @attacher
 */
MeasurementStatus fit_plane_ransac(const PointCloud& cloud_m, double distance_threshold_m, int iterations,
                                   Plane& out_plane, std::vector<std::size_t>& inlier_indices) {
#ifdef __ARM_EABI__
    (void)cloud_m;
    (void)distance_threshold_m;
    (void)iterations;
    (void)out_plane;
    (void)inlier_indices;
    return MeasurementStatus::kUnsupportedPlatform;
#else
    out_plane = Plane{};
    inlier_indices.clear();

    if (cloud_m.points.empty() || !(distance_threshold_m > 0.0) || iterations <= 0 || cloud_m.points.size() < 3) {
        return MeasurementStatus::kPlaneNotFound;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.reserve(cloud_m.points.size());
    for (const auto& p : cloud_m.points) {
        cloud->points.push_back(pcl::PointXYZ{p.x, p.y, p.z});
    }
    cloud->width = static_cast<std::uint32_t>(cloud->points.size());
    cloud->height = 1;

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(distance_threshold_m);
    segmentation.setMaxIterations(iterations);
    segmentation.setInputCloud(cloud);
    segmentation.segment(*inliers, *coefficients);

    if (inliers->indices.empty() || coefficients->values.size() < 4) {
        return MeasurementStatus::kPlaneNotFound;
    }

    // PCL plane model: a*x + b*y + c*z + d = 0 with unit normal (a,b,c).
    // Our Hessian plane uses n·x = d (i.e. n·x - d = 0), so d = -pcl_d.
    out_plane.nx = static_cast<float>(coefficients->values[0]);
    out_plane.ny = static_cast<float>(coefficients->values[1]);
    out_plane.nz = static_cast<float>(coefficients->values[2]);
    out_plane.d = static_cast<float>(-coefficients->values[3]);

    inlier_indices.assign(inliers->indices.begin(), inliers->indices.end());
    log_info("fit_plane_ransac: inliers=" + std::to_string(inlier_indices.size()));
    return MeasurementStatus::kSuccess;
#endif // __ARM_EABI__
}



/**
 * @brief [en] Removes the dominant background plane(s) from a preprocessed food cloud.
 * @brief [zh] 从预处理食物点云中移除主背景平面（及可选的次级平面）。
 * @attacher
 */
MeasurementStatus remove_dominant_plane(const PointCloud& cloud_m, const MeasurementConfig& cfg,
                                        PointCloud& remaining) {
#ifdef __ARM_EABI__
    (void)cloud_m;
    (void)cfg;
    (void)remaining;
    return MeasurementStatus::kUnsupportedPlatform;
#else
    remaining = PointCloud{};
    if (!(cfg.plane_distance_threshold_m > 0.0F) ||
        (cfg.remove_secondary_plane && !(cfg.secondary_plane_distance_threshold_m > 0.0F))) {
        return MeasurementStatus::kInvalidConfig;
    }

    auto remove_plane = [&](const Plane& plane, double threshold) {
        PointCloud kept;
        std::vector<std::size_t> inliers;
        split_plane_inliers(remaining, plane, threshold, kept, inliers);
        remaining = std::move(kept);
    };

    remaining = cloud_m;

    Plane plane{};
    std::vector<std::size_t> inliers;
    if (fit_plane_ransac(cloud_m, cfg.plane_distance_threshold_m, cfg.plane_ransac_iterations, plane, inliers) ==
        MeasurementStatus::kSuccess) {
        remove_plane(plane, cfg.plane_distance_threshold_m);
    }

    if (cfg.remove_secondary_plane) {
        Plane secondary{};
        std::vector<std::size_t> secondary_inliers;
        if (fit_plane_ransac(remaining, cfg.secondary_plane_distance_threshold_m, cfg.plane_ransac_iterations,
                             secondary, secondary_inliers) == MeasurementStatus::kSuccess) {
            remove_plane(secondary, cfg.secondary_plane_distance_threshold_m);
        }
    }
    log_info("remove_dominant_plane: remaining=" + std::to_string(remaining.points.size()));
    return MeasurementStatus::kSuccess;
#endif // __ARM_EABI__
}



} // namespace vm
