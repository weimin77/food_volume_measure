// Conan::ImportStart
#pragma once
#include <string>
#include "volume_types.hpp"
// Conan::ImportEnd



namespace vm {



/**
 * @brief [en] Loads a `MeasurementConfig` from a JSON file.
 * @brief [zh] 从 JSON 文件载入 `MeasurementConfig`。
 *
 * @param path [en] Path to the JSON config file.
 * @param path [zh] JSON 配置文件路径。
 * @param out [en] Config filled from the file. Missing fields keep their
 *     existing values, so callers may start from a default-initialized config
 *     and override only the keys present in the file.
 * @param out [zh] 从文件填充的配置；缺失字段保留原值，因此调用方可用默认配置
 *     初始化，仅覆盖文件中出现的键。
 * @return [en] True when the file was parsed and all present fields were valid.
 * @return [zh] 文件解析成功且所有出现的字段均有效时为真。
 *
 * @details [en] The JSON schema mirrors `MeasurementConfig` field names:
 *     - `"input_unit"`: `"meter"` or `"millimeter"`
 *     - `"selection_mode"`: `"all_eligible"` or `"manual"`
 *     - `"roi"`: object with `min_x` / `max_x` / `min_y` / `max_y` / `min_z` / `max_z`
 *     - `"selected_labels"`: array of integers
 *     - all other fields are scalars matching the struct member names.
 * @details [zh] JSON 结构与 `MeasurementConfig` 字段同名：
 *     - `"input_unit"`：`"meter"` 或 `"millimeter"`
 *     - `"selection_mode"`：`"all_eligible"` 或 `"manual"`
 *     - `"roi"`：含 `min_x` / `max_x` / `min_y` / `max_y` / `min_z` / `max_z` 的对象
 *     - `"selected_labels"`：整数数组
 *     - 其余字段为与结构体成员同名的标量。
 * @exporter
 */
bool load_config_from_json(const std::string& path, MeasurementConfig& out);



} // namespace vm
