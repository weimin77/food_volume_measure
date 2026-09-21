// Conan::ImportStart
#pragma once
#include <string>
#include "grid.hpp"
// Conan::ImportEnd



// PCD I/O backends shared by the public `load_pcd` wrapper and the intermediate-cloud
// debug dump. They stay private to `src/` and are never part of the installed API.
// 公开 `load_pcd` 与中间点云落盘共用的 PCD I/O 后端，保持私有，不进入安装头。

/**
 * @brief [en] PCD loading implementation shared by the public `load_pcd`.
 * @brief [zh] 公开 `load_pcd` 共用的 PCD 载入实现。
 */
PointCloud load_pcd_impl(const std::string& path);



/**
 * @brief [en] PCD writing implementation backing the intermediate-cloud dump.
 * @brief [zh] 中间点云落盘所用的 PCD 写出实现。
 * @param path [en] Output PCD path.
 * @param path [zh] 输出 PCD 路径。
 * @param cloud [en] Cloud to write, in metres.
 * @param cloud [zh] 待写出的点云（米）。
 * @return [en] True when the file was written.
 * @return [zh] 文件成功写出时为真。
 */
bool save_pcd_impl(const std::string& path, const PointCloud& cloud);
