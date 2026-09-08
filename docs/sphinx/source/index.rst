_`food_volume_measure Documentation`
=====================================

_`Introduction`
---------------

food_volume_measure is a C++17 library for point-cloud food volume measurement on
oven trays. It implements the IM (baseline-plane height-difference integral)
method with PCL: an empty-oven baseline is discretized into a local-plane height
map, the food cloud is voxel-downsampled and clustered in the baseline plane, and
per-cell height differences are integrated into a volume in cubic centimeters.

_`Main Features`
----------------

- Empty-oven baseline model with RANSAC plane fitting
- Open3D-equivalent voxel downsampling and DBSCAN clustering
- Component-aware conservative hole completion
- Reference volumes (AABB / OBB / convex hull)
- PCD loading via ``vm::load_pcd``
- Host-only point-cloud processing (PCL); bare-metal stubs report an unsupported status

_`Quick Start`
--------------

.. code-block:: cpp

   #include "volume_pipeline.hpp"
   #include "volume_pointcloudprocess.hpp"

   int main() {
       vm::PointCloud baseline = vm::load_pcd("empty_oven.pcd");
       vm::PointCloud food = vm::load_pcd("food.pcd");
       vm::VolumePipeline pipeline;
       vm::VolumeEstimate est =
           pipeline.measure({baseline}, food, vm::MeasurementConfig{});
       return est.status == vm::MeasurementStatus::kSuccess ? 0 : 1;
   }

_`Project Structure`
--------------------

::

   food_volume_measure/
   ├── include/         # Public headers (volume_*.hpp)
   ├── src/             # Implementation (volume_*.cpp)
   ├── test_package/    # Consumer demo and unit/stress tests
   ├── benchmark/       # Cross-platform benchmark scaffolding
   ├── CMakeLists.txt   # CMake build configuration
   ├── conanfile.py     # Conan package configuration
   ├── metadata.json    # Project metadata
   └── LICENSE          # License file

_`Detailed Settings`
--------------------

For detailed settings in project-level meta, conan recipe, or cmake file, see:

.. toctree::
   CMakeList <components/cmake>
   Conanfile <components/conan>
   Metadata <components/meta>
   User concern contents <export>
   :numbered:
   :maxdepth: 2

_`License`
----------

This project is licensed under the Apache-2.0 License. See the LICENSE file for details.
