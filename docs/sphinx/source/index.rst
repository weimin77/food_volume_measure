_`food_volume_measure Documentation`
=====================================

_`Introduction`
---------------

food_volume_measure is a C++17 library for point-cloud food volume measurement on
oven trays. It implements the IM (baseline-plane height-difference integral)
method: an empty-oven baseline is discretized into a local-plane height
map, the food cloud is voxel-downsampled and clustered in the baseline plane, and
per-cell height differences are integrated into a volume in cubic centimeters.

_`Main Features`
----------------

- Empty-oven baseline model with RANSAC plane fitting
- Voxel downsampling and density-based (DBSCAN) clustering
- Conservative hole completion within each food item
- Reference volumes (AABB / OBB / convex hull)
- Point-cloud input from a fixed depth camera
- Host-only point-cloud processing; bare-metal stubs report an unsupported status

_`Quick Start`
--------------

.. code-block:: cpp

   #include "volume_pipeline.hpp"

   int main() {
       vm::PointCloud baseline = /* 从深度相机读入的空炉点云 */;
       vm::PointCloud food = /* 从深度相机读入的食材点云 */;
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
