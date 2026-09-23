_`food_volume_measure Documentation`
=====================================

_`Introduction`
---------------

food_volume_measure is a C++17 library that measures food volume on oven trays from point
clouds. It follows the IM method (baseline-plane height-difference integral): the empty oven
becomes a height map on a fitted plane, the food cloud is voxel-downsampled and clustered in
that plane, and the per-cell height differences are integrated into a volume in cubic
centimeters.

_`Algorithm Steps`
------------------

The measurement runs in six stages. Each figure below is the output of one stage for a real
oven-tray scan.

.. figure:: ../images/1_origin_cloud.png
   :alt: Original input cloud
   :width: 100%

   **Step 1 - Raw input cloud.** The frame from the depth camera above the tray. Invalid points are dropped and coordinates are converted to metres.

.. figure:: ../images/2_downsample_cloud.png
   :alt: Voxel-downsampled cloud
   :width: 100%

   **Step 2 - Voxel downsampling.** Every occupied voxel is replaced by its centroid. This smooths out sensor noise and makes the later stages independent of the raw point density.

.. figure:: ../images/3_baseline_plane.png
   :alt: Empty-oven baseline surface
   :width: 100%

   **Step 3 - Empty-oven baseline.** A RANSAC plane fit and per-cell median heights turn the empty tray into the reference surface that food heights are measured against.

.. figure:: ../images/4_remove_bottom_plane.png
   :alt: Cloud after background-plane removal
   :width: 100%

   **Step 4 - Background-plane removal.** The dominant tray plane is removed, leaving mostly food points.

.. figure:: ../images/5_segment_food_component.png
   :alt: Segmented food components
   :width: 100%

   **Step 5 - Component segmentation.** The remaining points are projected onto the baseline plane and clustered with DBSCAN, which splits different food items into separate components.

.. figure:: ../images/6_height_integral.png
   :alt: Top surface used for the height integral
   :width: 100%

   **Step 6 - Height integral.** Each raster cell keeps one top-surface point. Its height above the baseline is integrated over the cell area, and enclosed holes are filled conservatively.

_`Main Features`
----------------

- Empty-oven baseline model with RANSAC plane fitting
- Voxel downsampling and DBSCAN clustering, matching Open3D
- Conservative hole completion, kept inside a single component
- Reference volumes (AABB / OBB / convex hull)
- PCD loading via ``load_pcd``
- Point-cloud processing runs on the host only; bare-metal stubs report "unsupported"

_`Quick Start`
--------------

.. code-block:: cpp

   #include "measurement.hpp"

   int main() {
       PointCloud baseline = load_pcd("empty_oven.pcd");
       PointCloud food = load_pcd("food.pcd");
       VolumeEstimate est =
           FoodVolumeMeasurer().set_baseline({baseline}).set_food(food).run();
       return est.status == MeasurementStatus::kSuccess ? 0 : 1;
   }

_`Project Structure`
--------------------

::

   food_volume_measure/
   ├── include/         # Public headers (*.hpp)
   ├── src/             # Implementation (*.cpp)
   ├── test_package/    # Consumer demo and unit/stress tests
   ├── benchmark/       # Cross-platform benchmark scaffolding
   ├── CMakeLists.txt   # CMake build configuration
   ├── conanfile.py     # Conan package configuration
   ├── metadata.json    # Project metadata
   └── LICENSE          # License file

_`Detailed Settings`
--------------------

Details on the project metadata, the Conan recipe and the CMake file:

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
